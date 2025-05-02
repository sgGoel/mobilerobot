#include <Arduino.h>
#include <string>


void setup() {
    Serial.begin();

    //if (Serial.available() > 0){
     //   int incomingByte = Serial.read();
    //}
}

// read from Serial port version of loop() function
// (not inherently incompatible with writing to Serial port, but the writing capability is just not needed in this version)
void loop() {

    static std::string buf;                    // keep it between calls

    while (Serial.available()) {               // ① drain the UART ring‑buffer
        buf.push_back( static_cast<char>(Serial.read()) );
    }

    if (!buf.empty()) {                        // ② send everything we got
        // Serial.write() outputs raw bytes; no format conversion, no NULL terminator needed
        Serial.write(reinterpret_cast<const uint8_t*>(buf.data()), buf.size());
        buf.clear();
    }

    delay(0.5);
}


