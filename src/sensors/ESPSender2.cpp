#include "esp_sender2.h"
#include <cstdio> 

void setupComm() {
    Serial.begin();

    if (Serial.available() > 0){ //TODO: test this setup mechanism
       int incomingByte = Serial.read();
    }
}

// read from Serial port version of loop() function
// (not inherently incompatible with writing to Serial port, but the writing capability is just not needed in this version)
SensorData loopComm() {

    static std::string buf;                    // keep it between calls

    while (Serial.available()) {               
        buf.push_back( static_cast<char>(Serial.read()) );
    }

    if (!buf.empty()) {                        
        // Serial.write() outputs raw bytes; no format conversion, no NULL terminator needed
        //Serial.write(reinterpret_cast<const uint8_t*>(buf.data()), buf.size()); //debug

        int task;
        if (sscanf(buf.c_str(), "@%d", &task) == 1) {
            Serial.print("task=");   Serial.println(task);
        }

        buf.clear();
        return {task};
    }

    return {-1};

    //delay(0.5); //some delay needed so as not to overwhelm microcontroller
}


