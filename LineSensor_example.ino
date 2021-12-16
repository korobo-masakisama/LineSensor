#include "LineSensor.h"
LineSensor LS = LineSensor(true); //引数はキャリブレーションするかどうか

void setup() {
  Serial.begin(115200);
  LS.setThreshold();//閾値設定
  LS.printThreshold();//閾値表示
  delay(5000);//ledが点灯するまでわざと時間がかかるという設定にしている
  LS.turnOnLed();//led点灯
}

void loop() {
  int lx, ly;
  LS.getLine(&lx, &ly);//機体が逃げる方向-255~255の大きさの(x y)のベクトル

  LS.printLoopSp();//ループ速度を計測する(ms),0or1(ms)程度ならok。
  
  Serial.print("\nescape_x: ");
  Serial.println(lx);
  Serial.print("escape_y: ");
  Serial.println(ly);
}
