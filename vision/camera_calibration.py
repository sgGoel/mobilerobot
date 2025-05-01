# thanks https://github.com/mit212/lab8_2025.git

import cv2
import numpy as np

def main():
    # --- Adjustable Parameters ---
    # The number of *interior* corners in your chessboard pattern.
    # For a pattern of 9 squares by 6 squares, there are 8 x 5 interior corners,
    # but commonly you'll see "9,6" meaning a pattern with 9 corners along width 
    # and 6 along height. Adjust if needed.
    chessboard_size = (8, 6)

    # The real size of each chessboard square in meters (or any arbitrary unit).
    # Adjust if your printed chessboard squares have a known size, e.g. 24 mm -> 0.024 m
    square_size = 0.068
    
    # Define criteria for corner sub-pixel refinement
    # (Stop after max 30 iterations or when moving by less than 0.001)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # --- Data Structures for Calibration ---
    # 3D points in real-world space for each image
    objpoints = []
    # 2D points in image plane for each image
    imgpoints = []
    
    # Prepare a single "view" of object points:
    # e.g. (0,0,0), (1,0,0), (2,0,0) ... for the entire chessboard grid.
    # This will get appended multiple times—once per collected frame.
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Open the default camera (device index 0)
    cap = cv2.VideoCapture(0)#, cv2.CAP_DSHOW)
    """cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G') )"""

    if not cap.isOpened():
        print("Could not open webcam.")
        return

    print("Press 's' to save corners when a chessboard is detected.")
    print("Press 'c' to calibrate using all saved frames.")
    print("Press 'q' to quit without calibrating (or quit after calibration).")

    i = 0
    while True:
        ret, frame = cap.read()
        #print("reading frames, ", i) #debugging latency
        i += 1
        if not ret:
            print("Failed to read from camera.")
            break
        
        # Convert to grayscale for corner detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Attempt to find corners
        found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        # If found, refine and draw the corners
        if found:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, chessboard_size, corners, found)

        # Show the live feed
        cv2.imshow('Calibration - Live Feed', frame)
        key = cv2.waitKey(100) & 0xFF
        
        if key == ord('q'):
            # Quit the program
            print("Exiting without further calibration.")
            break
        
        elif key == ord('s'):
            # If corners were found, store them
            if found:
                print("Chessboard corners found! Saving this frame.")
                objpoints.append(objp)
                imgpoints.append(corners)
            else:
                print("Chessboard not found in this frame. No corners saved.")
        
        elif key == ord('c'):
            # Perform calibration if we have enough frames
            if len(objpoints) < 3:
                print("Not enough frames collected. Need at least 3. Keep collecting.")
                continue
            
            print("Calibrating... please wait.")
            # Perform camera calibration
            ret_calib, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints,
                imgpoints,
                gray.shape[::-1],  # (width, height)
                None,
                None
            )
            
            if ret_calib:
                print("\n--- Calibration Successful ---")
                print("Camera Matrix:\n", camera_matrix)
                print("Distortion Coeffs:\n", dist_coeffs.ravel())
                print("--------------------------------\n")

                # Optionally, save to a file for later use
                np.savez(
                    "camera_calibration_live.npz",
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    rvecs=rvecs,
                    tvecs=tvecs
                )
                print("Calibration saved as camera_calibration_live.npz.\n")
            else:
                print("Calibration was not successful. Try collecting more data.")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()