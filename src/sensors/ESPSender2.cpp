#include "esp_sender2.h"
#include <cstdio> 
//#include "usb_id.h"

void setupComm() {
    //initUSB("Sensor‑ESP");
    Serial.begin(115200);
    Serial.println("$SENSOR");


    //if (Serial.available() > 0){ //TODO: test this setup mechanism
    //   int incomingByte = Serial.read();
    //}
}

// read from Serial port version of loop() function
// (not inherently incompatible with writing to Serial port, but the writing capability is just not needed in this version)
SensorData loopComm() {

    //sendToJetson();

    static std::string buf;                    // keep it between calls

    while (Serial.available()) {               
        buf.push_back( static_cast<char>(Serial.read()) );
    }

    //Serial.println("received on second thing!");

    if (!buf.empty()) {                        
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

void sendToJetson(){
    //Serial.println("&1"); //"I'm done!"
}


