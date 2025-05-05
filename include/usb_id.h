#pragma once
#include <USB.h>                    // tinyUSB wrapper in Arduino‑ESP32
#include <Arduino.h>   // Serial, delay, ESP.*
#include <USB.h>  

inline void initUSB(const char* product)
{
    // Build a per‑board serial number out of the chip's MAC address
    uint64_t mac = ESP.getEfuseMac();
    char sn[17];                    // 12 hex chars + NUL
    sprintf(sn, "%012llX", (unsigned long long)mac);

    USBDevice.setManufacturerDescriptor("Swati‑Labs");
    USBDevice.setProductDescriptor(product);   // e.g. "AprilTag‑ESP"
    USBDevice.setSerialDescriptor (sn);        // UNIQUE!

    Serial.begin(115200);           // starts the CDC‑ACM interface
    while (!Serial) { delay(10); }  // wait until the host enumerates
}

# AprilTag ESP32‑S3
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
       ATTRS{serial}=="A1B2C3D4E5F6", SYMLINK+="esp_cam"

# Sensor ESP32‑S3
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
       ATTRS{serial}=="0F1E2D3C4B5A", SYMLINK+="esp_sensors"