import os
from datetime import datetime
import argparse
import rclpy
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image, Imu
from cv_bridge import CvBridge
import csv
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import cv2
import numpy as np
import yaml


class ROS2bagProcessor:
    def __init__(self, bag_file, output_path, topics):
        """
        Initialize the ROS2bagProcessor for bag processing and chunking.

        Args:
            bag_file (str): Path to the input ROS 2 bag file.
            output_path (str): Path to save extracted or chunked data.
        """
        self.bag_file = bag_file
        self.output_path = output_path
        os.makedirs(output_path, exist_ok=True)
        self.topics = topics

        # ROS2 bag reader setup
        self.storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
        self.converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(self.storage_options, self.converter_options)

        # Writer for chunking
        self.writer = None
        self.split_count = 0
        self.topics = {}
        self.get_topics_from_bag()

    def initialize_imu_csv(self):
        """Initialize the IMU CSV file with headers."""
        with open(self.imu_csv_file, 'w', newline='') as csvfile:
            fieldnames = [
                'time_sec', 'time_nanosec', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def get_topics_from_bag(self):
        """Retrieve all topics and their types from the bag file."""
        for topic_metadata in self.reader.get_all_topics_and_types():
            self.topics[topic_metadata.name] = topic_metadata.type

    def extract(self):
        """
        Extract data from the ROS 2 bag file and save it to the output directory.
        """
        try:
            print("Starting extraction process...")

            # Reset the reader to the start of the bag
            self.reader = rosbag2_py.SequentialReader()
            self.reader.open(self.storage_options, self.converter_options)

            output_path = os.path.join(self.output_path, 'raw')
            os.makedirs(output_path, exist_ok=True)
            
            if '/lucid/image_raw' in self.topics:
                # Initialize CvBridge for image conversion
                self.bridge = CvBridge()
                self.first_camera_inner_time = []
                print("Getting camera outer time (ROS time)")
                self.image_outer_time = self.get_image_outer_time()
                print("Aligning camera inner time (sensor time) with outer time (ROS time)")
                self.aligned_camera_inner_starting_time = self.calculate_camera_inner_starting_time()

            first_point_cloud = True
            while self.reader.has_next():
                topic, data, t = self.reader.read_next()
                if topic == '/ouster/imu' and topic in self.topics: 
                    imu_msg = deserialize_message(data, Imu)
                    self.save_imu(imu_msg, output_path)
                elif topic == '/ouster/points' and topic in self.topics:
                    point_cloud_msg = deserialize_message(data, PointCloud2)
                    self.save_point_cloud(point_cloud_msg, output_path, first_point_cloud)
                    first_point_cloud = False
                elif topic == '/lucid/image_raw' and topic in self.topics:
                    image_msg = deserialize_message(data, Image)
                    self.save_image(image_msg, output_path)

            print("Extraction completed!")

        except Exception as e:
            print(f"Extraction failed: {e}")
        finally:
            self.cleanup()

        


    def save_imu(self, imu_msg, output_path):
        """Save IMU data to a CSV file."""
        self.imu_csv_file = os.path.join(output_path, 'ouster_lidar_imu.csv')
        self.initialize_imu_csv()
        try:
            with open(self.imu_csv_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec,
                    imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w,
                    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
                ])
            print("Saved IMU data")
        except Exception as e:
            print(f"Failed to save IMU data: {str(e)}")

    #def save_point_cloud(self, point_cloud_msg, output_path):
    #    """Save point cloud data to a PCD file using local time for filenames."""
    #    try:
    #        points = [
    #            [point[0], point[1], point[2]]
    #            for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    #        ]
    #        cloud = o3d.geometry.PointCloud()
    #        cloud.points = o3d.utility.Vector3dVector(points)

    #        # Convert ROS time to seconds
    #        secs = point_cloud_msg.header.stamp.sec + point_cloud_msg.header.stamp.nanosec * 1e-9
    #        dt = datetime.fromtimestamp(secs)
    #        print(secs)
    #        print(f"point_cloud_{dt.strftime('%Y%m%d_%H%M%S.%f')}.pcd")
    #        assert 0

    #        # Convert to local datetime
    #        dt = datetime.fromtimestamp(secs)

    #        # Format filename
    #        filename = os.path.join(output_path, f"point_cloud_{dt.strftime('%Y%m%d_%H%M%S.%f')}.pcd")

    #        print(filename)
    #        o3d.io.write_point_cloud(filename, cloud)
    #        print(f"Saved point cloud: {filename}")

    #    except Exception as e:
    #        print(f"Failed to save point cloud: {str(e)}")

    def save_point_cloud(self, point_cloud_msg, output_path, first_point_cloud):
        """Save point cloud data to a PCD file."""
        try:
            points = [
                [point[0], point[1], point[2]]
                for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            ]
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            
            metadata_path = os.path.join(self.bag_file, 'metadata.yaml')
            with open(metadata_path, 'r') as file:
                data = yaml.safe_load(file)
                start_time = data['rosbag2_bagfile_information']['starting_time']['nanoseconds_since_epoch']

            if first_point_cloud:
                self.min_sec = point_cloud_msg.header.stamp.sec
                self.min_nanosec = point_cloud_msg.header.stamp.nanosec

            absolute_nanosec_time = start_time + (point_cloud_msg.header.stamp.sec - self.min_sec)*1000000000 + (point_cloud_msg.header.stamp.nanosec - self.min_nanosec)
            print(point_cloud_msg.header.stamp.sec, point_cloud_msg.header.stamp.nanosec)
            print(absolute_nanosec_time)
            filename = os.path.join(output_path, f"point_cloud_{absolute_nanosec_time}.pcd")
            #filename = os.path.join(output_path, f"point_cloud_{start_time}_{point_cloud_msg.header.stamp.sec}_{point_cloud_msg.header.stamp.nanosec}.pcd")
            print(filename)
            o3d.io.write_point_cloud(filename, cloud)
            print(f"Saved point cloud: {filename}")
        except Exception as e:
            print(f"Failed to save point cloud: {str(e)}")
        

    def save_image(self, image_msg, output_path):
        if self.first_camera_inner_time == []:
            self.first_camera_inner_time.extend([image_msg.header.stamp.sec, image_msg.header.stamp.nanosec])
            print(self.first_camera_inner_time[0], self.first_camera_inner_time[1])
        try:
            img_msg = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            original_inner_timestamp = image_msg.header.stamp.sec * 1e9 + image_msg.header.stamp.nanosec
            original_first_inner_time = self.first_camera_inner_time[0] * 1e9 + self.first_camera_inner_time[1]
            aligned_inner_time_stamp = original_inner_timestamp - original_first_inner_time + self.aligned_camera_inner_starting_time
            sec = int(aligned_inner_time_stamp // 1e9)
            nanosec = str(int(aligned_inner_time_stamp %1e9)).zfill(9)
            img_filename = os.path.join(output_path, f"image_{sec}_{nanosec}.png")
            print("image_filename", img_filename)
            cv2.imwrite(img_filename, img_msg)
            print(f"Saved image: {img_filename}")
        except Exception as e:
            print(f"Failed to save image: {str(e)}")

    def get_image_outer_time(self):
        image_outer_time = []
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            if topic == '/lucid/image_raw':
                image_outer_time.append(t)
        # Reset the reader to the start of the bag
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(self.storage_options, self.converter_options)
        return image_outer_time

    def calculate_camera_inner_starting_time(self):
        '''
        linear regression outer time to align inner time
        '''
        x = np.arange(len(self.image_outer_time))
        slope, intercept = np.polyfit(x, self.image_outer_time, 1)
        print(f"camera_inner_starting_time: {intercept}")
        return intercept



    def chunk(self, chunk_time=10, chunk_start_time=0):
        """
        Split the ROS 2 bag file into smaller chunks based on the specified duration and start time.

        Args:
            chunk_time (int): Duration of each chunk in seconds.
            chunk_start_time (int): Start time for chunking in seconds.
        """
        chunk_start_time = chunk_start_time * 1e9  # Convert seconds to nanoseconds

        try:
            print("Starting chunking process...")
            start_found = False  # Flag to indicate if the start time is found

            # Reset the reader to the start of the bag
            self.reader = rosbag2_py.SequentialReader()
            self.reader.open(self.storage_options, self.converter_options)

            while self.reader.has_next():
                topic, data, t = self.reader.read_next()

                # Look for the start time frame
                if not start_found:
                    print(f"Bag start time: {t / 1e9}s, Chunk start time: {chunk_start_time / 1e9}s")
                    if t >= chunk_start_time:
                        start_found = True
                        self.create_new_writer()  # Create the first writer after finding the start time
                        self.chunk_start_time = t  # Initialize chunk start time
                    else:
                        continue  # Skip frames until the start time is found

                # Split based on chunk duration
                if t - self.chunk_start_time > chunk_time * 1e9:
                    self.chunk_start_time = t
                    self.create_new_writer()

                self.writer.write(topic, data, t)

        except Exception as e:
            print(f"Chunking failed: {e}")
        finally:
            self.cleanup()


    def create_new_writer(self):
        """Create a new writer for the next chunk."""
        if self.writer is not None:
            self.writer = None

        self.split_count += 1
        output_bag_file = os.path.join(self.output_path, f"chunks/chunk_{self.split_count}")
        os.makedirs(output_bag_file, exist_ok=True)
        print(f"Creating new chunk: {output_bag_file}")
        if os.path.exists(output_bag_file):
            os.system(f"rm -rf {output_bag_file}")

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_bag_file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.writer.open(storage_options, converter_options)

        # Create topics
        for topic, type_name in self.topics.items():
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic,
                type=type_name,
                serialization_format='cdr'
            ))

    def cleanup(self):
        """Ensure proper cleanup of resources."""
        if self.writer is not None:
            self.writer = None

def main():
    parser = argparse.ArgumentParser(description='Extracts ros2 lidar data from bag')
    parser.add_argument('--dir',
                        type=str,
                        help='directory to extract')
    args = parser.parse_args()

    print("starting")
    if not rclpy.ok():
        rclpy.init()
        bag_file = f"{args.dir}/rosbag"
        output_dir = f"{args.dir}/rosbag"
        extractor = ROS2bagProcessor(bag_file, output_dir, ['/ouster/imu', '/ouster/points', '/arena_camera_node/image_raw'])
        extractor.extract()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
