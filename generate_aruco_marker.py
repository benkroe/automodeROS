#!/usr/bin/env python3
"""Generate ArUco marker PNG for the target detection system."""

import cv2
import numpy as np

# Configuration from ref_model
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ARUCO_MARKER_ID = 1
MARKER_SIZE_PX = 1000  # Output size in pixels

def main():
    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    
    # Generate marker image
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, ARUCO_MARKER_ID, MARKER_SIZE_PX)
    
    # Save to file
    output_file = f'aruco_marker_id{ARUCO_MARKER_ID}_{MARKER_SIZE_PX}px.png'
    cv2.imwrite(output_file, marker_img)
    
    print(f"Generated ArUco marker:")
    print(f"  Dictionary: DICT_4X4_50")
    print(f"  Marker ID: {ARUCO_MARKER_ID}")
    print(f"  Size: {MARKER_SIZE_PX}x{MARKER_SIZE_PX} pixels")
    print(f"  Saved to: {output_file}")
    print(f"\nUse this image as texture in your simulation.")
    print(f"Black = marker pattern, White = background")

if __name__ == '__main__':
    main()
