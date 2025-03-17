#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <Adafruit_HX8357.h>
#include <Adafruit_FT5336.h>
#include <Arduino.h>

// TODO: Fix serial print

#define DISPLAY_WIDTH 480
#define DISPLAY_HEIGHT 320

struct TouchReading {
    int16_t x;
    int16_t y;
    int16_t pressure;

    void print(uint8_t nTabs = 0);
    bool operator==(const TouchReading& other);
} ;

#define FT5336_MAXTOUCHES 5
#define DEFAULT_TEXT_SIZE 3

class Display {
private:
    int _csPin;
    int _dcPin;
    int _ctpAddr;
    
    Adafruit_HX8357 _tft;
    Adafruit_FT5336 _ctp;
    TS_Point _pts[FT5336_MAXTOUCHES];

    bool _setupPrintCleared;
    TouchReading _currReading;

public:
    Display(int csPin, int dcPin, int ctpAddr = FT53XX_DEFAULT_ADDR);
    void setup();
    TouchReading read(bool debugPrint = false, int penRadius = 0, int penColor = HX8357_WHITE);

    void clear(int bgColor = HX8357_BLACK);
    void drawTouch(int penRadius = 0, int penColor = HX8357_WHITE);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t color);
    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    
    void setRotation(uint8_t rotation);
    void setCursor(int16_t x, int16_t y);
    void setTextColor(uint16_t color);
    void setTextSize(uint16_t size);

    void print(const char* text);
    void print(char text);
    void print(int num, int base = DEC) ;
    void print(unsigned int num, int base = DEC);
    void print(long num, int base = DEC) ;
    void print(unsigned long num, int base = DEC) ;
    void print(double num, int digits = 2);
    void print(const Printable& printable);
} ;

#endif // JOYSTICK_H