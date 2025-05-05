import cv2
import numpy as np
import pyapriltags as apriltag

import time
import serial
import threading
import queue



port_name1 = '/dev/ttyACM0' #sudo chmod 777 portname
serial_port1 = serial.Serial(port=port_name1, baudrate=115200, timeout=1, write_timeout=1) #comment out for local debugging
#write_q1 = queue.Queue()
#read_q1 = queue.Queue()


port_name2 = '/dev/ttyACM1' #sudo chmod 777 portname
serial_port2 = serial.Serial(port=port_name2, baudrate=115200, timeout=1, write_timeout=1) #comment out for local debugging

# NOTE: assumes camera_calibration has alrady been done

def april_tag_detection(frame):
    ret_val = []

    # serial code
    data = [0, 0, 0, 0]

    calibration_data = np.load('camera_calibration_live.npz')  # adjust filename
    camera_matrix = calibration_data['camera_matrix']  # shape (3, 3)
    dist_coeffs = calibration_data['dist_coeffs']      # shape (n,) typically (5,) or (8,)

    # Extract focal lengths and principal point from the camera matrix
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

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

    undistorted = frame # This turns off the undistortion
    
    # Convert to grayscale
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    results = at_detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=[fx, fy, cx, cy],  # from your calibration
        tag_size=TAG_SIZE
    )


    for r in results: #NOTE: we're only considering the first result, thus should not be using april tag information in cases where we're turning

        tag_id = r.tag_id
        #print("detected ID: ", tag_id) #debugging
        corners = r.corners.astype(int)

        # Draw the outline of the tag #NOTE: no display needed
        for i in range(4):
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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)


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

        #NOTE: no display needed
        cv2.putText(undistorted, "X: " + str(round(float(t[0]),2)) + ", Y: " + str(round(float(t[1]),2)) + ", Z: " + str(round(float(t[2]),2)), corners[0], cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        print(f"  Rotation vector [deg]:     {rot_deg.ravel()}")

        try:
            #serial_port.write(bytes("testing", "utf-8"))
            ret_val = [tag_id, round(float(t[0]),2), round(float(t[1]),2), round(float(t[2]),2)]
            #f"@{tag_id}@{round(float(t[0]),2)}@{round(float(t[1]),2)}@{round(float(t[2]),2)}"
            print("success")
        except Exception as e:
            print(e)
            pass
        #time.sleep(1)

    return ret_val


def color_detection(frame):

    # HSV color ranges for red (two ranges), blue, and yellow
    COLOR_RANGES = {
        'red': [
            ((0, 100, 100), (10, 255, 255)),
            ((160, 100, 100), (179, 255, 255)),
        ],
        'blue': [((100, 150, 50), (140, 255, 255))],
        'yellow': [((20, 100, 100), (35, 255, 255))],
    }

    # Minimum pixel dimensions for the strip
    # minimum should be more aggressive or def on rectangle should be more aggressive because we don;t want red cagei n background to trigger
    # maximum should also be more aggressive (OR ratio should be more aggressive)
    MIN_WIDTH  = 20#30
    MIN_HEIGHT = 100#150
    MAX_WIDTH = 375
    MAX_HEIGHT = 375
    MIN_RATIO = 0.1
    MAX_RATIO = 0.5

    # 220 vs 50 is the rough height to width ratio

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []
    for color, ranges in COLOR_RANGES.items():
        mask = None
        for lower, upper in ranges:
            m = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x,y,w,h = cv2.boundingRect(c)
            if w >= MIN_WIDTH and h >= MIN_HEIGHT and w/h >= MIN_RATIO and w/h <= MAX_RATIO: #w >= MIN_WIDTH and h >= MIN_HEIGHT and w <= MAX_WIDTH and h <= MAX_HEIGHT and 
                #print(color, x, y, w, h) #DEBUG
                detections.append((color, (x,y,w,h)))
                #detections.append(color)

    for color, (x,y,w,h) in detections:
        # draw and label
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(frame, color, (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        #print(f"Detected {color} strip at x={x}, y={y}, w={w}, h={h}")

    return detections

def io_thread(ser, write_q, read_q):
    while True:
        data = write_q.get()
        if data:
            # TODO: CASES FOR QUEUES
            print(f"@{data[0]}@{data[1]}@{data[2]}@{data[3]}@{data[4]}")
            ser.write(bytes(f"@{data[0]}@{data[1]}@{data[2]}@{data[3]}@{data[4]}"), "utf-8")
            #ser.flush()
        time.sleep(0.1)

def read_micro(deli, in_port, out_port):
    data = [0, 0, 0, 0]
    while True:
        try: # Be careful: any code inside this try block that fails will not display an error. Instead, the block will simply exit.
             # We recommend moving code outside of the try block for testing.
            if in_port.in_waiting > 0:
                esp32_output = str(in_port.readline()) # The ESP32 output should be a series of values seperated by commas and terminated by "\n", e.g. "1,2,3,4,\n".
                                                            # This termination occurs automatically if you use Serial.println();
                    
                vals = esp32_output.split(",") # Split into a list of strings
                print(vals) # Useful for debugging
                
                c1 = esp32_output[2:].strip()[0]
                c2 = esp32_output[2:].strip()[1]
                #print(f"debugging {c1} {c2}")

                if (c1 == deli):
                    #print("writing to out port!")
                    if (deli == "#"):
                        out_port.write(bytes(f"@{c2}", "utf-8")) #TODO: test task variable being updates on ESPSender2.cpp
                    else:
                        out_port.write(bytes(f"&{c2}", "utf-8"))
                
                in_port.reset_input_buffer()
            else:
                time.sleep(0.00101) # Sleep for at least 1 ms to give the loop a chance to rest
        except:
            pass

def main():
    cap = cv2.VideoCapture(0)#, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to read from the webcam.")
            break

        at = april_tag_detection(frame) #str
        col = color_detection(frame) #lst #we'll assume for now that only one color bar is detected
        #cv2.imshow('AprilTag Detection', frame) #NOTE: no display needed

        
        if (len(at) > 0 and len(col) > 0): #TODO: send color = -1 if no color, and do clear bucket logic
            d = {"red":0, "blue":1, "yellow":2}
            c = d[col[0][0]] if col[0][0] else -1
            try:
                print(f"@{at[0]}@{at[1]}@{at[2]}@{at[3]}@{c}")
                #write_q1.put((at[0], at[1], at[2], at[3], c))
                serial_port1.write(bytes(f"@{at[0]}@{at[1]}@{at[2]}@{at[3]}@{c}", "utf-8"))
            except Exception as e:
                print(e)

        # Press 'q' to quit
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    read_main = threading.Thread(target=read_micro, daemon = True, args=('#', serial_port1, serial_port2))
    read_main.start()

    read_side = threading.Thread(target=read_micro, daemon = True, args=('&', serial_port2, serial_port1))
    read_side.start()

    main()