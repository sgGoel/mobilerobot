#include "esp_sender.h"
#include <cstdio> 
#include <string>
//#include "usb_id.h"

std::atomic<bool> taskComp{false};
std::condition_variable cv;

void setupComm() {
    //initUSB("AprilTag‑ESP");

    Serial.begin(115200);
    Serial.println("$CAM");

    //if (Serial.available() > 0){ //TODO: test this setup mechanism
    //   int incomingByte = Serial.read();
    //}
    //delay(30);
}

// read from Serial port version of loop() function
// (not inherently incompatible with writing to Serial port, but the writing capability is just not needed in this version)
AprilTagData loopComm() {

    //TESTING
    //sendToJetson(1);

    static std::string buf;                    // keep it between calls

    while (Serial.available()) {               
        buf.push_back( static_cast<char>(Serial.read()) );
    }

    if (!buf.empty()) {                        
        //Serial.write() outputs raw bytes; no format conversion, no NULL terminator needed
        //Serial.write(reinterpret_cast<const uint8_t*>(buf.data()), buf.size()); //debug

        int id = -1;
        float x = -1;
        float y, z;
        int col;
        Serial.print("received:"); Serial.println(buf.c_str());
        if (sscanf(buf.c_str(), "@%d@%f@%f@%f@%d", &id, &x, &y, &z, &col) == 5) {
            Serial.print("id=");   Serial.println(id);
            Serial.print("x=");    Serial.println(x);
            Serial.print("y=");    Serial.println(y);
            Serial.print("z=");    Serial.println(z);
            Serial.print("col=");  Serial.println(col);
        }  else if (sscanf(buf.c_str(), "&%d", &id) == 1) {
            Serial.print("task_status=");   Serial.println(id);
            taskComp.store(true);
            cv.notify_one(); 
        }

        buf.clear();
        return {id, x, y, z, col};
    }

    return {-1,-1,-1,-1,-1};

    //delay(0.5); //some delay needed so as not to overwhelm microcontroller
}

//1 = pickup, 2 = dropoff
void sendToJetson(int task){
    if (task == 1){
        Serial.println("#1"); 
    } else {
        Serial.println("#2");
    }
}


