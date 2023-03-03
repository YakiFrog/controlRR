#include <Arduino.h>
#include <PS4Controller.h>
#include <math.h>
#include "m2006.h" // 自作ライブラリ
#include "mdds30.h" // 自作ライブラリ
//#include "udp.h" // 自作ライブラリ
#include "as5600_tca9548a.h" // 自作ライブラリ
//#include "pid_m2006.h" // 自作ライブラリ
#include "esp_intr_alloc.h" // 割り込み処理

// LED
#define LED_PIN 2

// CAN
#define rx 4
#define tx 5
#define CAN_SPEED 1000E3

// PS4
#define PS4_ADDR "0C:B8:15:C5:1C:C4" // PS4のMACアドレス(青色)

// AS5600_TCA9548A
#define DIR_PIN 21 // AS5600のDIRピン

// -------------------------------------------------- //
// M2006
int16_t current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分の電流値(16bit)
uint8_t send_data1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // M2006に送信するデータ(8bit) (ID:1-4)
uint8_t send_data2[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // M2006に送信するデータ(8bit) (ID:5-8)

// MDDS30
signed int speed_left = 0; // 左モータの速度(-100 ~ 100)
signed int speed_right = 0; // 右モータの速度(-100 ~ 100)

// UDP(ESPNOW): 接続したいESP32のMACアドレス
uint8_t unity_mac_addr[6] = {0x0C, 0xB8, 0x15, 0xF7, 0x3B, 0xA8};

// UDP(ESPNOW): 自分のMACアドレス
uint8_t my_mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// AS5600_TCA9548A
float offset1[4] = {0.0, 0.0, 0.0, 0.0}; // 静止時のオフセット値(4個分)
float offset2[4] = {0.0, 0.0, 0.0, 0.0}; // 回転時のオフセット値(4個分)
float current_angle[4] = {0.0, 0.0, 0.0, 0.0}; // AS5600の現在の角度(4個分) (0 to 360)
float target_angle[4] = {0.0, 0.0, 0.0, 0.0}; // AS5600の目標角度(4個分) (0 to 360)
float diff_angle[4] = {0.0, 0.0, 0.0, 0.0}; // AS5600の目標角度と現在の角度の差(4個分) (0 to 360)
// -------------------------------------------------- //

double l_x = 0.0; // 左スティックのX軸
double l_y = 0.0; // 左スティックのY軸
double r_x = 0.0; // 右スティックのX軸
double r_y = 0.0; // 右スティックのY軸

double angle = 0.0; // 方向転換用の角度

int16_t mangle[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t mrpm[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t mtorque[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int id[8] = {0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207, 0x208};

// -------------------------------------------------- //
// PID
double Kp = 13; // 比例ゲイン
double Ki = 0.03; // 積分ゲイン
double Kd = 0.03; // 微分ゲイン
double dt = 0.01; // サンプリングタイム

double Kp_angle = 1; // 比例ゲイン
double Ki_angle = 0.03; // 積分ゲイン
double Kd_angle = 0.03; // 微分ゲイン

int16_t target_rpm[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int base_current = 1000;
int direction[8] = {1, 1, 1, 1, 1, 1, 1, 1};

int16_t error[8];
int16_t integral[8];
int16_t derivative[8];
int16_t pre_error[8];
float pre_time = 0.0;
uint16_t sotai_error;

int16_t error_angle[4];
int16_t integral_angle[4];
int16_t derivative_angle[4];
int16_t pre_error_angle[4];
float pre_time_angle = 0.0;

//PID_M2006 pid(Kp, Ki, Kd, dt); // M2006のPID制御

// 初期設定
void setup() {
  // Serial
  Serial.begin(115200);
  Serial.println("> Serial: Started.");
  while (!Serial);

  // LED
  pinMode(LED_PIN, OUTPUT);

  // MDDS30
  digitalWrite(LED_PIN, HIGH); // LEDを点灯
  delay(3000);
  digitalWrite(LED_PIN, LOW); // LEDを消灯: この瞬間にMDDS30のリセットボタンを押す
  Serial.write(0x80);
  for (int i = 0; i < 3000; i++) {
    Serial.write(0x80);
    delay(1);
  }
  Serial.println();
  Serial.println("> MDDS30: Started.");

  // CAN
  while (!can_init(rx, tx, CAN_SPEED));
  Serial.println("> CAN: Started.");
  Serial.println("> CAN: RX: " + String(rx) + ", TX: " + String(tx) + ", Speed: " + String(CAN_SPEED, 0) + " bps");

  // UDP(ESPNOW)
  // while (!udp_init());
  // Serial.println("> UDP: Started.");
  // Serial.println("> UDP: My MacAddress: " + WiFi.macAddress());

  // // UDP(ESPNOW): 接続したいESP32のMACアドレスを登録
  // if (registerPeerInfo(unity_mac_addr) == ESP_OK) {
  //   Serial.println("> UDP: Peer registered.");
  // } else {
  //   Serial.println("> UDP: Peer not registered.");
  // }

  // コールバック関数の登録
  //esp_now_register_send_cb(onSend);
  //esp_now_register_recv_cb(onReceive);

  // PS4
  PS4.begin(PS4_ADDR);
  Serial.println("> PS4: Started.");
  Serial.println("> PS4: Press PS button to connect.");
  while (!PS4.isConnected());
  Serial.println("> PS4: Connected.");

  // AS5600_TCA9548A
  as5600_tca9548a_init(DIR_PIN);
  Serial.println("> AS5600_TCA9548A: Started.");
  as5600_tca9548a_get_offset(offset1);
  Serial.println("> AS5600_TCA9548A: Offset1: " + String(offset1[0]) + ", " + String(offset1[1]) + ", " + String(offset1[2]) + ", " + String(offset1[3]));

  // LED
  digitalWrite(LED_PIN, HIGH);
  delay(1000); // 1秒待つ

}

int x = 0;
int num = 0;

// メインループ: LEDが点滅していなければ，動いていないということ．
void loop() {
  if (PS4.LatestPacket()){
    // 移動・旋回
    if (PS4.LStickX()) l_x = PS4.LStickX(); // -127 ~ 127
    if (PS4.LStickY()) l_y = PS4.LStickY(); // -127 ~ 127
    if (PS4.RStickX()) r_x = PS4.RStickX(); // -127 ~ 127
    if (PS4.RStickY()) r_y = PS4.RStickY(); // -127 ~ 127
    // // 投射機構：上下
    // if (PS4.Up()) mdds30_control_motor(0x01, 70, 70);
    // if (PS4.Down()) mdds30_control_motor(0x01, -70, -70);
    // if (!PS4.Up() && !PS4.Down()) mdds30_control_motor(0x01, 0, 0); 
    // // 投射機構：回転
    // if (PS4.Circle()) mdds30_control_motor(0x00, 100, 100); // 100%
    // if (PS4.Triangle()) mdds30_control_motor(0x00, 70, 70); // 70%
    // if (PS4.Square()) mdds30_control_motor(0x00, 50, 50); // 50%
    // // 移動・旋回・投射機構：停止
    // if (PS4.Cross()) { // 全てのモータを停止
    //   mdds30_control_motor(0x00, 0, 0); // 0%
    //   mdds30_control_motor(0x01, 0, 0); // 0%
    //   l_x = 0.0;
    //   l_y = 0.0;
    //   r_x = 0.0;
    //   r_y = 0.0;
    // }
  }

  if (l_x < 12.7 && l_x > -12.7) l_x = 0.0; // 12.7以下の値は0とする
  if (l_y < 12.7 && l_y > -12.7) l_y = 0.0; // 12.7以下の値は0とする
  if (r_x < 12.7 && r_x > -12.7) r_x = 0.0; // 12.7以下の値は0とする
  if (r_y < 12.7 && r_y > -12.7) r_y = 0.0; // 12.7以下の値は0とする

  // l_x, l_yから角度を求める(-180 to 180)
  angle = atan2(l_x, l_y) * 180 / PI; // 0 ~ 360 (90度ずらす)
  // -180 to 180 -> 0 to 360
  angle = map(angle, -180, 180, 0, 360); // 0 ~ 360
  angle -= 180;
  for (int i = 0; i < 4; i++){
    target_angle[i] = angle;
  }

  for (int i = 0; i < 4; i++){
    diff_angle[i] = target_angle[i] - current_angle[i]; // 例：180 - (-90) = 270
    if (diff_angle[i] > 180) diff_angle[i] -= 360; // 例：270 - 360 = -90
  } 

  // Serial.print(String(target_angle[0]) + ", ");
  // for (int i = 0; i < 4; i++){
  //   Serial.print(String(diff_angle[i]) + ", ");
  // }
  // Serial.println();

  // モータに流す電流値を計算

  // 基底電流(これを基準に，角度に応じて電流を変化させる)
  // l_y -> -450 ~ 450
  l_y = map(l_y, -127, 127, -300, 300); // -450 ~ 450
  for (int i = 0; i < 8; i++){
    target_rpm[i] = l_y;
  }

   // // PID制御(角度制御) -> 4ユニットの偏差角度を計算
  for (int i = 0; i < 4; i++){
    error_angle[i] = abs(0) - abs(current_angle[i]);
    if (error_angle[i] > 180) error_angle[i] -= 360; // 例：270 - 360 = -90 (CCW)
    if (error_angle[i] < -180) error_angle[i] += 360; // 例：-270 + 360 = 90 (CW)

    // CW
    if (error_angle[i] > 0){ // 偏差が＋の場合, 偶数番目のCurrent_dataを減らす(CW)
      if (target_rpm[(i * 2)] > 0) target_rpm[(i * 2)] = (l_y) - (l_y * 0.2);
      else target_rpm[(i * 2)] = (l_y) + (l_y * 0.2);
    }
    // CCW
    if (error_angle[i] < 0){ // 偏差が-の場合, 奇数番目のCurrent_dataを減らす(CCW)
      if (target_rpm[(i * 2) + 1] > 0) target_rpm[(i * 2) + 1] = (l_y) - (l_y * 0.2);
      else target_rpm[(i * 2) + 1] = (l_y) + (l_y * 0.2);
    }
    // integral_angle[i] += error_angle[i] * dt; // 積分
    // derivative_angle[i] = (error_angle[i] - pre_error_angle[i]) / dt; // 微分
    // pre_error_angle[i] = error_angle[i]; // 前回の誤差を保存
  }

  // PID制御(速度制御)
  float dt = millis() - pre_time;
  pre_time = millis();
  for (int i = 0; i < 8; i++){
    error[i] = abs(target_rpm[i]) - abs(mrpm[i]);
    if (target_rpm[i] > 0 && i % 2 == 1) error[i] += abs(mrpm[i - 1]) - abs(mrpm[i]); // 速度差を誤差に加算(CCW)
    if (target_rpm[i] < 0 && i % 2 == 0) error[i] += abs(mrpm[i + 1]) - abs(mrpm[i]); // 速度差を誤差に加算(CCW)
    integral[i] += error[i] * dt; // 積分
    derivative[i] = (error[i] - pre_error[i]) / dt; // 微分
    pre_error[i] = error[i]; // 前回の誤差を保存
    if (target_rpm[i] > 0){
      direction[i] = 1;
    }else if (target_rpm[i] < 0){
      direction[i] = -1;
    }
  }

  

  // 現在角度を計算
  // for (int i = 0; i < 4; i++){
  //   pid_angle[i] += (((mrpm[0 + (i * 2)] + mrpm[1 + (i * 2)]) / 2) * dt) * (180 / PI); 
  // }


  if (l_y != 0){
    // 速度制御
    for (int i = 0; i < 4; i++){
      current_data[0 + (i * 2)] = direction[0 + (i * 2)] * (base_current + (Kp * error[0 + (i * 2)] + Kd * derivative[0 + (i * 2)] + Ki * integral[0 + (i * 2)])); // TOP(CW)
      current_data[1 + (i * 2)] = -direction[1 + (i * 2)] * (base_current + (Kp * error[1 + (i * 2)] + Kd * derivative[1 + (i * 2)] + Ki * integral[1 + (i * 2)])); // BOTTOM(CCW)
      if (current_data[0 + (i * 2)] > 3000) current_data[0 + (i * 2)] = 3000;
      if (current_data[1 + (i * 2)] < -3000) current_data[1 + (i * 2)] = -3000;

    }
  } else if ((l_y == 0  && l_x == 0 )|| !PS4.LatestPacket()){
    for (int i = 0; i < 8; i++){
      current_data[i] = 0;
    }
  }

  // ------------------------------------------------------------ //
  // 100msごとに割り込み処理
  static unsigned long last_time = 0;
  if (millis() - last_time > 100){ 
    last_time = millis();
    // 現在のホイールの方位方向の角度を取得
    as5600_tca9548a_get_current_angle(current_angle, offset1, offset2);
  }
  // ------------------------------------------------------------ //
  // M2006に送信するデータを作成
  m2006_make_data(current_data, send_data1, send_data2);
  // M2006にデータを送信
  m2006_send_data(send_data1, send_data2);
  // M2006のデータを読むこむ
  m2006_read_data(id[num], mangle, mrpm, mtorque);
  num = (num + 1) % 8;
  //Serial.print(String(angle) + ": ");
  Serial.print(String(l_y) + ": ");
  int angle_num = 0;
  for (int i = 0; i < 8; i++){
    Serial.print(String(mrpm[i]) + ": "); // 速度
    //Serial.print("[" + String(current_data[i]) + "], "); // 電流
    if (i % 2 == 1){
      Serial.print(String(error_angle[angle_num]) + ": "); // 偏差角度
      angle_num++;
    }
  }
  Serial.println();
  // LEDを点滅
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // ------------------------------------------------------------ //
}