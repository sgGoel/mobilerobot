import cv2
import numpy as np
import pyapriltags as apriltag

def compute_pose_april(frame):
    pass

# TODO: integrate into compute_pose_april

def main():
    #----------------------------------------------------------------------
    # 1. Load camera calibration data
    #----------------------------------------------------------------------
    calibration_data = np.load('camera_calibration_live.npz')  # adjust filename
    camera_matrix = calibration_data['camera_matrix']  # shape (3, 3)
    dist_coeffs = calibration_data['dist_coeffs']      # shape (n,) typically (5,) or (8,)

    # Extract focal lengths and principal point from the camera matrix
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    #----------------------------------------------------------------------
    # 2. Set up the AprilTag detector
    #----------------------------------------------------------------------
    # For more details on parameters, see:
    # https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
    at_detector = apriltag.Detector(
        families='tag36h11',  # or 'tag25h9', etc.
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    # The length of one side of the tag in meters
    TAG_SIZE = 0.10  # 10 cm

    #----------------------------------------------------------------------
    # 3. Initialize Webcam
    #----------------------------------------------------------------------
    cap = cv2.VideoCapture(0)#, cv2.CAP_DSHOW)
    """cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G') )"""
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from the webcam.")
            break

        #------------------------------------------------------------------
        # 4. Undistort (optional but recommended)
        #    You can either:
        #    A) Undistort the entire image once, or
        #    B) Let the AprilTag detector handle radial distortion by
        #       passing camera_params directly (estimate_tag_pose=True).
        #
        # In practice, the built-in pose estimation in pupil_apriltags
        # uses the pinhole model with no distortion. So it's best to
        # either undistort the frame yourself or accept small distortion
        # errors if your lens is fairly undistorted or uses a cheap camera.
        #------------------------------------------------------------------
        # Option A: Undistort the entire frame
##        h, w = frame.shape[:2]
##        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
##            camera_matrix, dist_coeffs, (w, h), 1, (w, h))
##        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        undistorted = frame # This turns off the undistortion
        
        # Convert to grayscale
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        
        #------------------------------------------------------------------
        # 5. Detect AprilTags
        #------------------------------------------------------------------
        # With pupil_apriltags, you can directly get pose estimation by
        # providing 'estimate_tag_pose=True' and camera_params + tag_size.
        # This automatically computes the rotation and translation of the tag.
        #------------------------------------------------------------------
        results = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],  # from your calibration
            tag_size=TAG_SIZE
        )

        #------------------------------------------------------------------
        # 6. Process each detection
        #------------------------------------------------------------------
        for r in results:
            # r.tag_id: the ID of the detected tag
            # r.corners: the (4,2) array of corner coordinates in the image
            # r.center:  the (x,y) coordinates of the tag center
            # r.pose_R, r.pose_t: pose of the tag in the camera frame
            #                    (right-handed coordinate system):
            #    - R is a 3x3 rotation matrix
            #    - t is a 3x1 translation vector
            #
            # The coordinate system by default: +x to the right, +y down,
            # and +z forward from the camera's perspective.
            #

            #----------------------------------------
            # 6a. Extract Tag ID and corners
            #----------------------------------------
            tag_id = r.tag_id
            corners = r.corners.astype(int)

            #----------------------------------------
            # 6b. Draw the detection on the image
            #----------------------------------------
            # Draw the outline of the tag
            """for i in range(4): #NOTE: no need to display
                cv2.line(
                    undistorted,
                    tuple(corners[i]),
                    tuple(corners[(i+1) % 4]),
                    (0, 255, 0),
                    2
                )

            # Draw the tag ID near the center
            center_xy = (int(r.center[0]), int(r.center[1]))
            cv2.putText(undistorted, f"ID: {tag_id}", center_xy,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)"""

            #----------------------------------------
            # 6c. Get Pose (R, t)
            #----------------------------------------
            R = r.pose_R  # 3×3 rotation matrix
            t = r.pose_t  # 3×1 translation vector

            # Convert the rotation matrix to Euler angles, if needed:
            # (Though you can also use the rotation matrix directly.)
            # Example (using cv2.Rodrigues):
            rot_vec, _ = cv2.Rodrigues(R)     # from rotation matrix to rotation vector
            rot_deg = np.degrees(rot_vec)     # convert to degrees for display if desired

            # Print or display the pose
            # Note: The translation is in meters, given TAG_SIZE is in meters.
            # The orientation is with respect to the camera frame.
            print(f"Detected Tag ID {tag_id}:")
            print(f"  Translation (x, y, z) [m]: {t.ravel()}")

            #NOTE: no need to display
            #cv2.putText(undistorted, "X: " + str(round(float(t[0]),2)) + ", Y: " + str(round(float(t[1]),2)) + ", Z: " + str(round(float(t[2]),2)), corners[0], cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            print(f"  Rotation vector [deg]:     {rot_deg.ravel()}")

        #------------------------------------------------------------------
        # 7. Show the result
        #------------------------------------------------------------------
        #NOTE: no need to display
        #cv2.imshow('AprilTag Detection', undistorted)

        # Press 'q' to quit
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()