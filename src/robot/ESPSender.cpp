#include "esp_sender.h"
#include <cstdio> 

void setupComm() {
    Serial.begin();

    //if (Serial.available() > 0){
     //   int incomingByte = Serial.read();
    //}
}

// read from Serial port version of loop() function
// (not inherently incompatible with writing to Serial port, but the writing capability is just not needed in this version)
AprilTagData loopComm() {

    static std::string buf;                    // keep it between calls

    while (Serial.available()) {               
        buf.push_back( static_cast<char>(Serial.read()) );
    }

    if (!buf.empty()) {                        
        // Serial.write() outputs raw bytes; no format conversion, no NULL terminator needed
        Serial.write(reinterpret_cast<const uint8_t*>(buf.data()), buf.size()); //debug

        int id;
        float x, y, z;
        int col;
        if (sscanf(buf.c_str(), "@%d@%f@%f@%f@%f", &id, &x, &y, &z) == 4) {
            Serial.print("id=");   Serial.println(id);
            Serial.print("x=");    Serial.println(x);
            Serial.print("y=");    Serial.println(y);
            Serial.print("z=");    Serial.println(z);
            Serial.print("col=");  Serial.println(col);
        }

        buf.clear();
        return {id, x, y, z, col};
    }

    return {-1,-1,-1,-1,-1};

    //delay(0.5); //some delay needed so as not to overwhelm microcontroller
}


