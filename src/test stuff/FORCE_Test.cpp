/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-force-sensor
 */

 #include <Arduino.h>

 #define FORCE_SENSOR_PIN A3 // ESP32 pin GPIO36 (ADC0): the FSR and 10K pulldown are connected to A0

 void setup() {
   Serial.begin();
 
   // set the ADC attenuation to 11 dB (up to ~3.3V input)
   analogSetAttenuation(ADC_11db);
 }
 
 void loop() {
   int analogReading = analogRead(FORCE_SENSOR_PIN);
 
   Serial.print("The force sensor value = ");
   Serial.print(analogReading); // print the raw analog reading
 
   if (analogReading < 10)       // from 0 to 9
     Serial.println(" -> no pressure");
   else if (analogReading < 200) // from 10 to 199
     Serial.println(" -> light touch");
   else if (analogReading < 500) // from 200 to 499
     Serial.println(" -> light squeeze");
   else if (analogReading < 800) // from 500 to 799
     Serial.println(" -> medium squeeze");
   else // from 800 to 1023
     Serial.println(" -> big squeeze");
 
   delay(100);
 }
 
 
 
 
 
 
 
 