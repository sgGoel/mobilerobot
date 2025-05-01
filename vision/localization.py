from apriltags import *
from objectdetection import * #TODO: universal import!

import numpy as np
import cv2 as cv

def main():
    # thanks https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
    cap = cv.VideoCapture(0) # we'll only have one camera
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        #pe = compute_robot_pose(frame)
    
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv.imshow('frame', gray)
        if cv.waitKey(1) == ord('q'):
            break
    
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

def compute_robot_pose(frame):
    pos1, conf1 = compute_pose_cv(frame)
    pos2, conf2 = compute_pose_april(frame)
    return (pos1*conf1 + pos2*conf2) / (conf1 + conf2)

if __name__ == "__main__":
    main()
