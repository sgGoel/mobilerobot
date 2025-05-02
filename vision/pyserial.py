

import time
import serial # Pyserial. Install as "pip install pyserial"
            
if __name__ == '__main__':
    
    # Connect to the ESP32. You can use PlatformIO to find its COM port. Remember that the COM port is different in BOOT mode than when code is running!
    port_name = '/dev/ttyACM2'

    serial_port = serial.Serial(port=port_name, baudrate=115200, timeout=1, write_timeout=1)
    data = [0, 0, 0, 0]

    while True:
        try:
            serial_port.write("testing")
        except:
            pass
        time.sleep(0.01)

    while True:
        try: # Be careful: any code inside this try block that fails will not display an error. Instead, the block will simply exit.
             # We recommend moving code outside of the try block for testing.
            if serial_port.in_waiting > 0:
                esp32_output = str(serial_port.readline()) # The ESP32 output should be a series of values seperated by commas and terminated by "\n", e.g. "1,2,3,4,\n".
                                                           # This termination occurs automatically if you use Serial.println();
                    
                vals = esp32_output.split(",") # Split into a list of strings
                print(vals) # Useful for debugging
                
                for i in range(len(data)):
                    data[i] = float(vals[i])
                
                serial_port.reset_input_buffer()
            else:
                time.sleep(0.00101) # Sleep for at least 1 ms to give the loop a chance to rest
        except:
            pass

        # How to write a String to the ESP32
##        try:
##            cmd_str = "Test"
##            serial_port.write(cmd_str.encode())
##        except:
##            print('Write failed')
                
    ##### END OF MAIN LOOP. Put some condition within the loop to break it when the program should exit. ######
    serial_port.close()