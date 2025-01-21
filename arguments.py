import argparse

parser = argparse.ArgumentParser(description="Synchronized RGB-Thermal Camera System")

parser.add_argument("--base_dir", 
                    type=str, 
                    default=r"C:\Users\EndUser\Documents\Programming\BosonChecks\UCR_RGBT\dataset\1_20_25_calib_2",
                    help="Base directory for saving synchronized image pairs")

parser.add_argument("--fps", 
                    type=float, 
                    default=59.0,
                    help="Target frame rate for acquisition (default: 59.0)")

parser.add_argument("--exposure_time", 
                    type=float, 
                    default=1000.0,
                    help="Exposure time in microseconds for Blackfly camera (default: 5000.0)")

parser.add_argument("--gain", 
                    type=float, 
                    default=15.0,
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
                    default=20,
                    help='Recording duration in frames.')