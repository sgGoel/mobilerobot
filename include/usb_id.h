#pragma once
#include <USB.h>                    // tinyUSB wrapper in Arduino‑ESP32
#include <Arduino.h>   // Serial, delay, ESP.*
//#include <USB.h>  
#include "esp32-hal-tinyusb.h"
/*
inline void initUSB(const char* product)
{
    // Build a per‑board serial number out of the chip's MAC address
    uint64_t mac = ESP.getEfuseMac();
    char sn[17];                    // 12 hex chars + NUL
    sprintf(sn, "%012llX", (unsigned long long)mac);

    TinyUSBDevice.setManufacturerDescriptor("Swati‑Labs");
    TinyUSBDevice.setProductDescriptor(product);   // e.g. "AprilTag‑ESP"
    TinyUSBDevice.setSerialDescriptor (sn);        // UNIQUE!

    Serial.begin(115200);           // starts the CDC‑ACM interface
    while (!Serial) { delay(10); }  // wait until the host enumerates
}
*/
/*
accompanying rule:
# AprilTag ESP32‑S3
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
       ATTRS{serial}=="ec:da:3b:5c:85:38", SYMLINK+="esp_cam"

# Sensor ESP32‑S3
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
       ATTRS{serial}=="ec:da:3b:41:a3:54", SYMLINK+="esp_sensors"
*/