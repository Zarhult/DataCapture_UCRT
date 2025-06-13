"""Capture syncronized rgb-lwir
"""

import os
import PySpin
import cv2
import numpy as np
from flirpy.camera.boson import Boson
from PIL import Image

from boson import configure_boson
from blackfly import configure_blackfly
from arguments import parser

import time
from datetime import datetime

import logging
NUM_BUFFERS=1

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

HOMO = np.array([[ 9.05196993e-01, -3.26275658e-03, -3.43366182e+01],
                [-2.74723623e-04,  9.01438758e-01,  3.03051600e+01],
                [ 8.44170313e-07, -9.40938689e-06,  1.00000000e+00]])

def calculate_fps(prev_time, curr_time, frame_count):
    time_elapsed = curr_time - prev_time
    fps = frame_count / time_elapsed
    return fps, curr_time

def capture_image_pair(blackfly_cam, boson_cam):
    try:

        # Capture from Boson
        boson_image = boson_cam.grab()

        # Capture from Blackfly
        blackfly_image = blackfly_cam.GetNextImage(100)
        if blackfly_image.IsIncomplete():
            print("Blackfly image incomplete. Skipping.")
            blackfly_image.Release()
            return None, None
        
        # Convert Blackfly image to numpy array
        blackfly_array = blackfly_image.GetNDArray()
        blackfly_image.Release()
        
        return blackfly_array, boson_image
    
    except PySpin.SpinnakerException as e:
        print(f"Error: {e}")
        return None, None

def save_recorded_frames(blackfly_frames, boson_frames, base_dir):
    # Determine the next available scene number
    scene_number = 1
    while os.path.exists(os.path.join(base_dir, f'scene_{scene_number}')):
        scene_number += 1
    
    # Create directories
    scene_path = os.path.join(base_dir, f'scene_{scene_number}')
    lwir_path = os.path.join(scene_path, 'lwir', 'raw', 'data')
    rgb_path = os.path.join(scene_path, 'rgb', 'raw', 'data')
    os.makedirs(lwir_path, exist_ok=True)
    os.makedirs(rgb_path, exist_ok=True)

    # Save frames
    for frame_number, (blackfly_frame, boson_frame) in enumerate(zip(blackfly_frames, boson_frames)):
        timestamp = datetime.now()
        
        # Save LWIR frame
        lwir_filename = f"LWIR_RAW_{frame_number:06d}_{timestamp.strftime('%H_%M_%S_%f')[:-3]}"
        frame = Image.fromarray(boson_frame)
        lwir_filename_path = os.path.join(lwir_path, lwir_filename)
        frame.save(f'{lwir_filename_path}.tiff')
        # cv2.imwrite(os.path.join(lwir_path, lwir_filename), boson_frame)
        
        # Save RGB frame
        rgb_filename = f"RGB_RAW_{frame_number:06d}_{timestamp.strftime('%H_%M_%S_%f')[:-3]}.png"
        cv2.imwrite(os.path.join(rgb_path, rgb_filename), blackfly_frame)
    
def recorder(blackfly_cam, boson_cam, num_frames):
    '''Lightweight method to record frames'''
    # Captured Frames
    captured_frames_blackfly = []
    captured_frames_boson = []
    ctr = 0
    # boson_cam.do_ffc()

    while ctr<=num_frames:
        blackfly_img, boson_img = capture_image_pair(blackfly_cam, boson_cam)
        
        if blackfly_img is None or boson_img is None:
            continue

        # Handle recording
        captured_frames_blackfly.append(blackfly_img)
        captured_frames_boson.append(boson_img)

        ctr+=1

    return captured_frames_blackfly, captured_frames_boson

def make_recording(args, recording, combined_img,blackfly_cam,boson_cam,num_frames,base_dir):
    time.sleep(args.delay)
    # boson_cam.do_ffc()
    # time.sleep(args.delay)

    if not recording:
        print("Starting recording")
        recording=True
        cv2.putText(combined_img, "Recording", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        captured_frames_blackfly, captured_frames_boson = recorder(blackfly_cam,boson_cam,num_frames)

        for i in range(len(captured_frames_blackfly)):
            captured_frames_blackfly[i] = cv2.cvtColor(captured_frames_blackfly[i], cv2.COLOR_BayerRG2RGB)
            print(captured_frames_blackfly[i].shape)
            H,W = captured_frames_blackfly[i].shape[:2]
            captured_frames_blackfly[i] = cv2.pyrDown(captured_frames_blackfly[i],(int(W//2),int(H//2)))

        print("Recording complete")
        recording = False
        save_recorded_frames(captured_frames_blackfly, captured_frames_boson, base_dir)  # You'll need to pass args to this function
        print(f"Frames saved successfully: {base_dir}")
        captured_frames_blackfly = []
        captured_frames_boson = []

def live_visualizer(blackfly_cam, boson_cam, args):
    cv2.namedWindow("Combined View", cv2.WINDOW_KEEPRATIO)
    base_dir = args.base_dir
    blackfly_cam.BeginAcquisition()

    # Initialize variables for recording
    recording = False
    captured_frames_blackfly = []
    captured_frames_boson = []
    num_frames = args.frames

    # Initialize FPS variables
    prev_time = time.time()
    frame_count = 0
    fps_update_interval = 1.0
    blackfly_fps = 0
    boson_fps = 0
    boson_cam.do_ffc()
    try:
        while True:
            blackfly_img, boson_img = capture_image_pair(blackfly_cam, boson_cam)

            if blackfly_img is None or boson_img is None:
                continue

            else:
                # FPS calculation
                frame_count += 1
                curr_time = time.time()
                if curr_time - prev_time >= fps_update_interval:
                    blackfly_fps, prev_time = calculate_fps(prev_time, curr_time, frame_count)
                    boson_fps = blackfly_fps
                    frame_count = 0
                
                # Prepare display image
                boson_height, boson_width = boson_img.shape[:2]
                blackfly_height, blackfly_width = blackfly_img.shape[:2]
                # If blackfly is unit16, convert to unit8
                if blackfly_img.dtype == np.uint16:
                    blackfly_img = (blackfly_img / 256).astype(np.uint8)
                scale_factor = boson_height / blackfly_height

                blackfly_img = cv2.pyrDown(blackfly_img,(int(blackfly_width//2),int(blackfly_height//2)))
                blackfly_resized = cv2.resize(blackfly_img, (boson_width, boson_height))
                blackfly_resized = cv2.cvtColor(blackfly_resized, cv2.COLOR_BayerRG2RGB)

                boson_normalized = cv2.normalize(boson_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                if args.flip_boson:
                    boson_normalized = cv2.flip(boson_normalized, 1)
    
                boson_colormap = cv2.applyColorMap(boson_normalized, cv2.COLORMAP_INFERNO)

                height, width = boson_colormap.shape[:2]
                warped = cv2.warpPerspective(blackfly_img, HOMO, (width,height))
                warped = np.repeat(warped[:, :, np.newaxis], 3, axis=2)
                overlay = cv2.addWeighted(warped, 0.5 , boson_colormap, 1-0.5, 0) 
                combined_img = np.hstack((blackfly_resized, boson_colormap, overlay))

                # Add FPS text
                cv2.putText(combined_img, f"FPS: {blackfly_fps:.2f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Combined View", combined_img)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                make_recording(args, recording, combined_img,blackfly_cam,boson_cam,num_frames,base_dir)
            elif key == ord('l'):
                make_recording(args, recording, combined_img,blackfly_cam,boson_cam,num_frames,base_dir)

    finally:
        blackfly_cam.EndAcquisition()
        cv2.destroyAllWindows()

def main():
    args = parser.parse_args()
    
    # Initialize cameras (existing code)
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    blackfly_cam = cam_list[0]
    blackfly_cam.Init()
    boson_cam = Boson()
    
    try:
        configure_blackfly(blackfly_cam,args.exposure_time,args.gain,args.width,args.height)
        configure_boson(boson_cam)
        
        os.makedirs(args.base_dir, exist_ok=True)
        live_visualizer(blackfly_cam, boson_cam, args)
    
    finally:
        blackfly_cam.DeInit()
        del blackfly_cam
        cam_list.Clear()
        system.ReleaseInstance()
        boson_cam.close()

if __name__ == "__main__":
    main()