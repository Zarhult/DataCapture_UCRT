import argparse

parser = argparse.ArgumentParser(description="Synchronized RGB-Thermal Camera System")

parser.add_argument("--base_dir", 
                    type=str, 
                    default=r"C:\Users\harshith\Documents\workspace\camera\DataCapture_UCRT\data\05_08_2025",
                    help="Base directory for saving synchronized image pairs")

parser.add_argument("--fps", 
                    type=float, 
                    default=59.0,
                    help="Target frame rate for acquisition (default: 59.0)")

parser.add_argument("--exposure_time", 
                    type=float, 
                    default=500.0,
                    help="Exposure time in microseconds for Blackfly camera (default: 5000.0)")

parser.add_argument("--gain", 
                    type=float, 
                    default=10.0,
                    help="Gain value for Blackfly camera (default: 10.0)")

parser.add_argument("--flip_boson", 
                    action="store_true",
                    help="Flip the Boson camera image horizontally")

parser.add_argument("--delay",
                    type=int,
                    default=3,
                    help="Adds a timer delay before recording.")

parser.add_argument("--frames",
                    type=int,
                    default=100,
                    help='Recording duration in frames.')

parser.add_argument("--width",
                    type=int,
                    default=1280,
                    help='Width of the image to capture.')

parser.add_argument("--height",
                    type=int,
                    default=960,
                    help='Height of the image to capture.')

parser.add_argument("--record",
                    action='store_true',
                    default=False,
                    help='Whether or not to record data.')

parser.add_argument("--indefinite",
                    action="store_true",
                    default=False,
                    help='Whether to record until user interrupts.')
