"""Reads data from specified directory and returns a list of images. Options to view the images in a window.
"""

import cv2
import os
import numpy as np
from tqdm import tqdm

def read_images(directory):
    """Reads all images from the specified directory and returns a list of images.
    """
    lwir_images = []
    rgb_images = []

    # Read lwir images
    for file in tqdm(os.listdir(os.path.join(directory, "lwir/raw/data")), desc="Reading LWIR images"):
        if file.endswith(".tiff"):
            img = cv2.imread(os.path.join(directory, "lwir/raw/data", file), cv2.IMREAD_ANYDEPTH)
            lwir_images.append(img)

    # Read rgb images
    for file in tqdm(os.listdir(os.path.join(directory, "rgb/raw/data")), desc="Reading RGB images"):
        if file.endswith(".png"):
            rgb_images.append(cv2.imread(os.path.join(directory, "rgb/raw/data", file)))

    # Conver to np arrays
    lwir_images = np.array(lwir_images)
    rgb_images = np.array(rgb_images)

    # Normalize lwir images
    lwir_images = (lwir_images - np.min(lwir_images)) / (np.max(lwir_images) - np.min(lwir_images))

    return lwir_images, rgb_images

def display_images(lwir_images, rgb_images):
    """Displays the lwir and rgb images in a window.
    """
    for i in range(len(lwir_images)):
        # Create a copy of the LWIR image for display
        display_lwir = lwir_images[i].copy()
        
        # normalize lwir image if needed
        if np.max(display_lwir) <= 1:
            display_lwir = (display_lwir * 255.0).astype(np.uint8)
        
        # Flip LWIR image horizontally
        display_lwir = cv2.flip(display_lwir, 1)
        
        # Use colormap to display lwir image
        display_lwir = cv2.applyColorMap(display_lwir, cv2.COLORMAP_INFERNO)

        # Downsample rgb image
        display_rgb = cv2.pyrDown(rgb_images[i])
        display_rgb = cv2.resize(display_rgb, (display_lwir.shape[1], display_lwir.shape[0]))

        # Create display both images
        combined_image = np.hstack((display_lwir, display_rgb))
        # cv2.imshow("LWIR | RGB", combined_image)

        # Check for next image
        # key = cv2.waitKey(0) & 0xFF

        # if key == 27: # Esc key
        #     break
        # # Check for previous image (left arrow key)
        # elif key == 81:
        #     i -= 1
        # # Check for next image (right arrow key)
        # elif key == 83:
        #     i += 1
        
    # cv2.destroyAllWindows()

def save_images(lwir_images, rgb_images, output_directory):
    """Saves the lwir and rgb images to separate subdirectories in the specified output directory.
    """
    # Create separate directories for lwir and rgb images
    lwir_dir = os.path.join(output_directory, "lwir")
    rgb_dir = os.path.join(output_directory, "rgb")
    os.makedirs(lwir_dir, exist_ok=True)
    os.makedirs(rgb_dir, exist_ok=True)

    for i in range(len(lwir_images)):
        # Convert normalized LWIR image to uint8 and apply colormap
        display_lwir = (lwir_images[i] * 255.0).astype(np.uint8)
        display_lwir = cv2.applyColorMap(display_lwir, cv2.COLORMAP_INFERNO)
        
        # Save the images in their respective directories
        cv2.imwrite(os.path.join(lwir_dir, f"lwir_{i}.png"), display_lwir)
        cv2.imwrite(os.path.join(rgb_dir, f"rgb_{i}.png"), rgb_images[i])

if __name__ == "__main__":
    lwir_images, rgb_images = read_images("data/05_08_2025/scene_4")
    display_images(lwir_images, rgb_images)
    save_images(lwir_images, rgb_images, "data/05_08_2025_output/scene_4")
