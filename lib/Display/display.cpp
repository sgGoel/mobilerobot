# include "display.h"
# include "util.h"
# include <Arduino.h>

void TouchReading::print(uint8_t nTabs) {
    printTabs(nTabs); Serial.printf("x: %d\n", x);
    printTabs(nTabs); Serial.printf("y: %d\n", y);
    printTabs(nTabs); Serial.printf("pressure: %d\n", pressure);
}

bool TouchReading::operator==(const TouchReading& other) {
    return x == other.x &&
           y == other.y;
}

Display::Display(int csPin, int dcPin, int ctpAddr) :
                _csPin(csPin), _dcPin(dcPin), _ctpAddr(ctpAddr), 
                _tft(csPin, dcPin), _setupPrintCleared(true) { }

void Display::setup() {
    _tft.begin();

    if (! _ctp.begin(_ctpAddr, &Wire)) { 
        if (Serial) Serial.println("Couldn't start FT5336 touchscreen controller.");
        while (1) delay(10);
    }

    clear();
    setRotation(1); 
    setTextColor(HX8357_WHITE);
    setTextSize(DEFAULT_TEXT_SIZE);
    setCursor(15, 15);  
    print("setup complete!"); 
    _setupPrintCleared = false;
}

TouchReading Display::read(bool debugPrint, int penRadius, int penColor) {
    uint8_t touches = _ctp.touched();
    if (! touches) {
        _currReading.pressure = -1;
        return _currReading;
    }
    
    _ctp.getPoints(_pts, 1);
    
    _currReading.x = _pts[0].y;
    _currReading.y = _pts[0].x;
    _currReading.pressure = _pts[0].z;

    if (debugPrint && Serial && _currReading.pressure > 0) {
        Serial.printf("Read display with touchscreen at I2C address 0x%x\n", _ctpAddr);
        _currReading.print();
    }

    drawTouch(penRadius, penColor);
    return _currReading;    
}

void Display::clear(int bgColor) {
    _tft.fillScreen(bgColor);
    _setupPrintCleared = true;
}

void Display::drawTouch(int penRadius, int penColor) {
    if (penRadius > 0 && _currReading.pressure > 0) {
        // If setup message is still printed on the screen
        if (!_setupPrintCleared) 
            clear();
        fillCircle(_currReading.x, _currReading.y, penRadius, penColor);
    }
}

void Display::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    _tft.fillRect(x, y, w, h, color);
}

void Display::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    _tft.drawRect(x, y, w, h, color);
}

void Display::fillCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
    _tft.fillCircle(x, y, r, color);
}

void Display::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    _tft.drawCircle(x0, y0, r, color);
}

void Display::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    _tft.drawLine(x0, y0, x1, y1, color);
}

void Display::setRotation(uint8_t rotation) {
    _tft.setRotation(rotation);
}

void Display::setCursor(int16_t x, int16_t y) {
    _tft.setCursor(x, y);
}

void Display::setTextColor(uint16_t color) {
    _tft.setTextColor(color);
}

void Display::setTextSize(uint16_t size) {
    _tft.setTextSize(size);
}

void Display::print(const char* text) {
    _tft.print(text);
}

void Display::print(char text) {
    _tft.print(text);
}

void Display::print(int num, int base) {
    _tft.print(num, base);
}

void Display::print(unsigned int num, int base) {
    _tft.print(num, base);
}

void Display::print(long num, int base) {
    _tft.print(num, base);
}

void Display::print(unsigned long num, int base) {
    _tft.print(num, base);
}

void Display::print(double num, int digits) {
    _tft.print(num, digits);
}

void Display::print(const Printable& printable) {
    _tft.print(printable);
}