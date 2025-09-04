"""Configure the Boson camera
Harshith Mohan Kumar
1/19/25
"""
import sys
import cv2

def custom_setup_video(boson, device_id=None):
    """
    Setup the camera for video/frame capture.

    Attempts to automatically locate the camera, then opens an OpenCV VideoCapture object. The
    capture object is setup to capture raw video.
    """

    if device_id is None:
        boson.logger.debug("Locating cameras")
        device_id = boson.find_video_device()

    if device_id is None:
        raise ValueError("Boson not connected.")
    else:
        boson.logger.debug("Located camera at {}".format(device_id))

    if sys.platform.startswith("linux"):
        boson.cap = cv2.VideoCapture(device_id + cv2.CAP_V4L2)
    elif sys.platform.startswith("win32"):
        boson.cap = cv2.VideoCapture(device_id + cv2.CAP_DSHOW)
    else:
        # Catch anything else, e.g. Mac?
        boson.cap = cv2.VideoCapture(device_id)

    if not boson.cap.isOpened():
        raise IOError("Failed to open capture device {}".format(device_id))

    # Attempt to set 320x256 which only has an effect on the Boson 320
    boson.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    boson.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)

    # The order of these calls matters!
    # The Y16 command tells the Boson to send pre-AGC 16 bit pixels. This means the pixel intensity is linearly
    # proportional to the flux incident on the pixel. This also means all internal image processing done by the
    # boson is ignored. There are other options, see Boson datasheet.
    boson.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y16 "))
    boson.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

def configure_boson(boson):
    custom_setup_video(boson)

    # Set Boson to external sync slave mode
    boson.set_external_sync_mode(2)  # 2 is for slave mode
    # Reduce timing surprises
    boson.set_ffc_manual()
