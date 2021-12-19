#ifndef _TEENSY_h
#define _TEENSY_h
#include <FastLED.h>

#define eeprom_offset 0 //EEPROMの０～NUM_LINES番地を使う
#define NUM_LEDS 20
#define NUM_LINES 20
#define LED_DATA_PIN 29

class LineSensor {
  public:
    LineSensor(boolean calibration);
    void getLine(int *escape_x, int *escape_y);
    void printThreshold();
    void printLoopSp();
    void turnOnLed();
    void setThreshold();
    void calibration(byte mode);
    int lineRead(byte line_number);
    byte judgLine(byte line_number, int *x, int *y);
    double culc_LSM_13(int x[], int y[], int N);//1,3エリアの範囲のラインセンサの反応に対する最小二乗法の計算
    double culc_LSM_24(int y[], int x[], int N);//2,4エリアの範囲のラインセンサの反応に対する最小二乗法の計算

  private:
    const uint8_t leds_pin[NUM_LINES] = {A4, A5, A6, A7, A8, A9, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A1, A1, A2, A3};
    //↑ピン番号、↓ラインセンサの座標、ラインセンサの円を中心とし、半径255の円とする。
    const int coordinate[NUM_LINES][2] = {{0, -255}, {79, -243}, {150, -206}, {206, -150}, {243, -79}, {255, 0}, {243, 79}, {206, 150}, {150, 206}, {79, 243}, {0, 255}, { -79, 243}, { -150, 206}, { -206, 150}, { -243, 79}, { -255, 0}, { -243, -79}, { -206, -150}, { -150, -206}, { -79, -243}};
    const float threshold_ratio = 0.60;//閾値緑最大値6割
    int raw_value[NUM_LINES];
    int line_threshold[NUM_LINES] = {0};
    boolean cb;
    CRGB leds[NUM_LEDS];
};

#endif
