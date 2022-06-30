#include "Arduino.h"
#include "LineSensor.h"
#include <EEPROM.h>

LineSensor::LineSensor(boolean calibration) {
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  cb = calibration;
}

void LineSensor::setThreshold(void) {
  if (cb) {
    LineSensor::calibration(3);
    LineSensor::calibration(5);
    LineSensor::calibration(6);
  } else {
    for (int i = 0; i < NUM_LINES; i++) {
      line_threshold[i] = EEPROM.read(i + eeprom_offset) * 4;
    }
    Serial.println("Skipped calibration");
  }  
}

void LineSensor::getLine(int *escape_x, int *escape_y) {
  byte num_line_react = 0;//ラインが反応している個数
  int x, y;
  int xp[NUM_LINES] = {0};
  int yp[NUM_LINES] = {0};
  byte react_num;//ラインを最初に踏んだラインセンサの番号
  static byte last_value[2];
  static boolean lineflag;
  static byte linedir;

  for (int i = 0; i < NUM_LINES; i++) {
    raw_value[i] = LineSensor::lineRead(i);
    if (LineSensor::judgLine(i, &x, &y)) {
      if (num_line_react == 0) {
        react_num = i;
      }
      xp[num_line_react] = x;
      yp[num_line_react] = y;
      num_line_react++;
    }
  }

  if (last_value[1] == 0 && last_value[0] == 0 && num_line_react != 0) {
    lineflag = true;
    if ((react_num <= 2 && react_num >= 0) || (react_num == 18 || react_num == 19)) {
      linedir = 1;
    } else if (react_num >= 3 && react_num <= 7) {
      linedir = 2;
    } else if (react_num >= 8 && react_num <= 12) {
      linedir = 3;
    } else if (react_num >= 13 && react_num <= 17) {
      linedir = 4;
    }
  }

  if (last_value[1] == 0 && last_value[0] == 0 && num_line_react == 0) {
    lineflag = false;
    linedir = 0;
  }

  switch (linedir) {
    case 1:
      if (num_line_react == 1) {
        *escape_x = (-1) * xp[0];
        *escape_y = (-1) * yp[0];
        break;
      }
      *escape_y = 255;
      *escape_x = (-1) * LineSensor::culc_LSM_13(xp, yp, num_line_react) * (*escape_y);

      if (*escape_x > 255) {
        linedir = 4;
        *escape_x = 0;
      } else if (*escape_x < -255) {
        linedir = 2;
        *escape_x = 0;
      }
      break;

    case 2:
      if (num_line_react == 1) {
        *escape_x = (-1) * xp[0];
        *escape_y = (-1) * yp[0];
        break;
      }
      *escape_x = -255;
      *escape_y = LineSensor::culc_LSM_24(xp, yp, num_line_react) * (*escape_x);

      if (*escape_y > 255) {
        linedir = 1;
        *escape_y = 0;
      } else if (*escape_y < -255) {
        linedir = 3;
        *escape_y = 0;
      }
      break;

    case 3:
      if (num_line_react == 1) {
        *escape_x = (-1) * xp[0];
        *escape_y = (-1) * yp[0];
        break;
      }
      *escape_y = -255;
      *escape_x = (-1) * LineSensor::culc_LSM_13(xp, yp, num_line_react) * (*escape_y);

      if (*escape_x > 255) {
        linedir = 4;
        *escape_x = 0;
      } else if (*escape_x < -255) {
        linedir = 2;
        *escape_x = 0;
      }
      break;

    case 4:
      if (num_line_react == 1) {
        *escape_x = (-1) * xp[0];
        *escape_y = (-1) * yp[0];
        break;
      }
      *escape_x = 255;
      *escape_y = LineSensor::culc_LSM_24(xp, yp, num_line_react) * (*escape_x); //90°回転した座標を与える

      if (*escape_y > 255) {
        linedir = 1;
        *escape_y = 0;
      } else if (*escape_y < -255) {
        linedir = 3;
        *escape_y = 0;
      }
      break;

    default:
      *escape_x = 0;
      *escape_y = 0;
      break;
  }

  last_value[1] = last_value[0];
  last_value[0] = num_line_react;
  //Serial.println(num_line_react);
  //Serial.print("linedir: ");
  //Serial.println(linedir);
  //  Serial.print("lineflag: ");
  //  Serial.println(lineflag);
  //  Serial.print("escape_x: ");
  //  Serial.println(escape_x);
  //  Serial.print("escape_y: ");
  //  Serial.println(escape_y);
  //  Serial.print("\n");
}

byte LineSensor::judgLine(byte line_number, int *x, int *y) {
  if (raw_value[line_number] > line_threshold[line_number]) {
    *x = coordinate[line_number][0];
    *y = coordinate[line_number][1];
    return 1;
  } else {
    return 0;
  }
}

// 最小二乗法の計算//y = a0 + a1x//機体が逃げる方向の座標(x, y)//y = -(x / a1)
double LineSensor::culc_LSM_13(int x[], int y[], int N) {
  int i;
  double a;
  double A00 = 0 , A01 = 0, A02 = 0, A011 = 0, A012 = 0;

  for (i = 0; i < N; i++) {
    A00 += 1.0;
    A01 += x[i];
    A02 += y[i];
    A011 += x[i] * x[i];
    A012 += x[i] * y[i];
  }
  a = (A00 * A012 - A01 * A02) / (A00 * A011 - A01 * A01);
  return a;//ラインの傾き//-1/aが逃げる方向の傾き
}

//90°回転した座標を与える//回転行列90°で変換した
//理由は最小二乗法では傾きaは無限大となり、求められないため
//x座標は-yとなり、y座標はx座標になる。
double LineSensor::culc_LSM_24(int y[], int x[], int N) {
  int i;
  double a;
  double A00 = 0 , A01 = 0, A02 = 0, A011 = 0, A012 = 0;

  for (i = 0; i < N; i++) {
    A00 += 1.0;
    A01 += x[i];
    A02 += (-1 * y[i]);
    A011 += x[i] * x[i];
    A012 += x[i] * (-1 * y[i]);
  }
  a = (A00 * A012 - A01 * A02) / (A00 * A011 - A01 * A01);
  return a;//逃げる方向の傾き
}

int LineSensor::lineRead(byte line_number) {
  int raw_value = analogRead(leds_pin[line_number]);
  /*Serial.print("law_value");
    Serial.print(line_number + 1);
    Serial.print(" :");
    Serial.println(raw_value);*/
  return raw_value;
}

void LineSensor::turnOnLed(void) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 255, 255);
    FastLED.show();
  }
}

void LineSensor::calibration(byte mode) {
  int static green_max[NUM_LINES] = {0};
  int static white_max[NUM_LINES] = {0};
  char input;

  switch (mode) {
    case 1://緑最大値初期化
      for (int i = 0; i < NUM_LINES; i++) {
        green_max[i] = 0;
      }
      break;

    case 2://白最大値初期化
      for (int i = 0; i < NUM_LINES; i++) {
        white_max[i] = 0;
      }
      break;

    case 3://緑フィールドでの最大値測定
      LineSensor::turnOnLed();
      while (1) {
        for (int i = 0; i < NUM_LINES; i++) {
          raw_value[i] = LineSensor::lineRead(i);
          green_max[i] = max(raw_value[i], green_max[i]);
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print("Green_MAX:");
          Serial.print(green_max[i]);
          Serial.print("\traw_value");
          Serial.println(raw_value[i]);
          //Serial.print("\n");
        }
        input = Serial.read();
        if (input == 'f') {
          Serial.println("Calibration_Green_Completed");
          delay(2000);
          break;
        }
      }
    //break;

    case 4://ライン上での最大値測定
      LineSensor::turnOnLed();
      while (1) {
        for (int i = 0; i < NUM_LINES; i++) {
          raw_value[i] = LineSensor::lineRead(i);
          white_max[i] = max(raw_value[i], white_max[i]);
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print("White_MAX:");
          Serial.print(white_max[i]);
          Serial.print("\tGreen_MAX:");
          Serial.print(green_max[i]);
          Serial.print("\traw_value");
          Serial.println(raw_value[i]);
          //Serial.print("\n");
        }
        input = Serial.read();
        if (input == 'f') {
          Serial.println("Calibration_White_Completed");
          delay(2000);
          break;
        }
      }
      break;

    case 5://閾値表示
      for (int i = 0; i < NUM_LINES; i++) {
        line_threshold[i] = green_max[i] * threshold_ratio + white_max[i] * (1 - threshold_ratio);
        line_threshold[i] &= 0b1111111100;
        Serial.print("line_threshold");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(line_threshold[i]);
      }
      break;

    case 6://EEPROMに書き込み保存
      for (int i = 0; i < NUM_LINES; i++) {
        EEPROM.write(i + eeprom_offset, line_threshold[i] / 4);
      }
      break;

    default:
      break;
  }
}

void LineSensor::printThreshold(void) {
  for (int i = 0; i < NUM_LINES; i++) {
    Serial.print("line_threshold");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(line_threshold[i]);
  }
  delay(2000);
}

//ループ速度を計測する(ms),0or1(ms)程度ならok。
void LineSensor::printLoopSp(void) {
  int loopsp;
  static unsigned long pretime = 0;
  unsigned long nowtime;
  nowtime = millis();
  loopsp = nowtime - pretime;
  pretime = nowtime;

  Serial.print("\nloopsp(ms): ");
  Serial.println(loopsp);
}
