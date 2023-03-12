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
float diff_angle[4] = {0.0, 0.0, 0.0, 0.0}; // AS5600の目標角度と現在の角度の差(4個分) (0 to 360)
// -------------------------------------------------- //

double l_x = 0.0; // 左スティックのX軸
double l_y = 0.0; // 左スティックのY軸
double r_x = 0.0; // 右スティックのX軸
double r_y = 0.0; // 右スティックのY軸

double controller_angle = 0.0; // コントローラの角度

// M2006のCAN受信データ
int16_t mangle[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分の角度値(16bit)
int16_t mrpm[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分の回転数値(16bit)
int16_t mtorque[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分のトルク値(16bit)

int id[8] = {0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207, 0x208};

// -------------------------------------------------- //
// PID
double Kp = 13; // 比例ゲイン 13
double Ki = 0.03; // 積分ゲイン
double Kd = 0.05; // 微分ゲイン

double Kp_angle = 2; // 比例ゲイン
double Ki_angle = 0.0003; // 積分ゲイン
double Kd_angle = 0.000; // 微分ゲイン

int16_t target_rpm[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t target_angle[4] = {0, 0, 0, 0};
int base_current = 1000;
int direction[8] = {1, 1, 1, 1, 1, 1, 1, 1};
int direct = 1;
int length = 0;

// PID制御(速度制御)
int16_t error[8];
int16_t integral[8];
int16_t derivative[8];
int16_t pre_error[8];

// PID制御(角度制御)
int16_t fixed_rpm[4] = {0, 0, 0, 0};
int16_t error_angle[4];
int16_t integral_angle[4];
int16_t derivative_angle[4];
int16_t pre_error_angle[4];
float pre_time_angle = 0.0;

// PIDの計算用
float prev_time = 0.0;

float average_angle = 0;

// DifferentialDrive
int16_t w_w[4] = {0, 0, 0, 0}; // 移動方向の速度
int16_t w_a[4] = {0, 0, 0, 0}; // 方位方向の速度
int16_t d_a[4] = {0, 0, 0, 0}; // 方位方向の速度

//PID_M2006 pid(Kp, Ki, Kd, dt); // M2006のPID制御

// void pid_rpm(){
//   // PID制御(速度制御)
//   float dt = millis() - pre_time;
//   pre_time = millis();
//   for (int i = 0; i < 8; i++){
//     error[i] = abs(target_rpm[i]) - abs(mrpm[i]);
//     if (target_rpm[i] > 0 && i % 2 == 1) error[i] += abs(mrpm[i - 1]) - abs(mrpm[i]); // 速度差を誤差に加算(CCW)
//     if (target_rpm[i] < 0 && i % 2 == 0) error[i] += abs(mrpm[i + 1]) - abs(mrpm[i]); // 速度差を誤差に加算(CCW)
//     integral[i] += error[i] * dt; // 積分
//     derivative[i] = (error[i] - pre_error[i]) / dt; // 微分
//     pre_error[i] = error[i]; // 前回の誤差を保存
//   }
// }

void pid_angle(){
  for (int i = 0; i < 4; i++){
    error_angle[i] = target_angle[i] - current_angle[i];
    if (error_angle[i] > 180) error_angle[i] -= 360; // 180度以上の誤差を-180度に変換
    if (error_angle[i] < -180) error_angle[i] += 360; // -180度以下の誤差を180度に変換
    if (error_angle[i] > 0) direction[i] = 1; // 方向を保存
    if (error_angle[i] < 0) direction[i] = -1; // 方向を保存
    error_angle[i] = abs(error_angle[i]); // 誤差を絶対値に変換
  }
}

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
    if (PS4.Left()) l_x = -100;
    if (PS4.Right()) l_x = 100;
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

  // //l_x, l_yから角度を求める(-180 to 180)
  // controller_angle = atan2(l_x, l_y) * 180 / PI; // 0 ~ 360 (90度ずらす)
  // // -180 to 0 -> 180 to 360
  // if (controller_angle < 0) controller_angle += 360; 

  // for (int i = 0; i < 4; i++){
  //   target_angle[i] = controller_angle; // 目標角度(0 ~ 360)
  // }

  // l_y -127 ~ 127 to -300 ~ 300
  l_y = map(l_y, -127, 127, -300, 300);

  // r_x -127 ~ 127 to -300 ~ 300
  r_x = map(r_x, -127, 127, -100, 100);

  /******************************************
   * PID制御
   ******************************************/

  average_angle = current_angle[0]; // 1ユニット目の角度を目標角度とする

  // 経過時間の計算
  float dt = millis() - prev_time;
  prev_time = millis();

  // PID制御(角度制御)
  for (int i = 0; i < 4; i++){
    error_angle[i] = average_angle - current_angle[i];
    integral_angle[i] += error_angle[i] * dt; // 積分
    derivative_angle[i] = (error_angle[i] - pre_error_angle[i]) / dt; // 微分
    pre_error_angle[i] = error_angle[i]; // 前回の誤差を保存
  }

  // 電流値の計算(-3000 ~ 3000)
  for (int i = 0; i < 4; i++){
    fixed_rpm[i] = (Kp_angle * error_angle[i] + Ki_angle * integral_angle[i] + Kd_angle * derivative_angle[i]);
    fixed_rpm[i] = constrain(fixed_rpm[i], -60, 60);
  }

  // 目標のモータの回転数を計算
  for (int i = 0; i < 8; i++){ // 前後・左右移動のRPM値を代入して目標値を求めている
    if (i % 2 == 0) target_rpm[i] = l_y - r_x; // r_xを引くと，上から見た時にCWする
    else target_rpm[i] = -l_y - r_x; 
  }

  for (int i = 0; i < 4; i++){ 
    target_rpm[0 + (i * 2)] -= fixed_rpm[i];
    target_rpm[1 + (i * 2)] -= fixed_rpm[i];
  }

  // Diffencial Swerve Driveの式
  for (int i = 0; i < 4; i++){
    w_w[i] = (mrpm[0 + (i * 2)] - mrpm[1 + (i * 2)]) / 2; // 移動方向の速度
    w_a[i] = (mrpm[0 + (i * 2)] + mrpm[1 + (i * 2)]) / 2; // 方位方向の速度
  }

  // 角度の計算
  for (int i = 0; i < 4; i++){
    d_a[i] += (w_a[i] * (180 / PI)) * (dt / 1000) / 36; // 角度
    if (d_a[i] > 360) d_a[i] -= 360; // 360度以上になったら0度に戻す
    if (d_a[i] < 0) d_a[i] += 360; // 0度未満になったら360度にする
  }

  // PID制御(速度制御)
  for (int i = 0; i < 8; i++){
    error[i] = target_rpm[i] - mrpm[i];
    integral[i] += error[i] * dt; // 積分
    derivative[i] = (error[i] - pre_error[i]) / dt; // 微分
    pre_error[i] = error[i]; // 前回の誤差を保存
  }

  // 電流値の計算(-3000 ~ 3000)
  for (int i = 0; i < 4; i++){
    current_data[0 + (i * 2)] = (Kp * error[0 + (i * 2)] + Ki * integral[0 + (i * 2)] + Kd * derivative[0 + (i * 2)]);
    current_data[1 + (i * 2)] = (Kp * error[1 + (i * 2)] + Ki * integral[1 + (i * 2)] + Kd * derivative[1 + (i * 2)]);

    current_data[0 + (i * 2)] = constrain(current_data[0 + (i * 2)], -3000, 3000); // -3000 ~ 3000
    current_data[1 + (i * 2)] = constrain(current_data[1 + (i * 2)], -3000, 3000); // -3000 ~ 3000
  }

  // average_angle = 0;
  // for (int i = 0; i < 4; i++){
  //   average_angle += current_angle[i];
  // }
  // average_angle /= 4;

  // もし，PS4の左スティックが中央に戻ったら，モータの電流を0にする（もしくは，PS4の接続が切れたら）
  // if ((l_y == 0 && l_x == 0 && r_x == 0 && r_y == 0)|| !PS4.LatestPacket()){
  //   for (int i = 0; i < 8; i++){
  //     current_data[i] = 0;
  //     target_rpm[i] = 0;
  //   }
  //   for (int i = 0; i < 4; i++){
  //     target_angle[i] = 0;
  //   }
  // }

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

  //Serial.print(String(controller_angle, 0) + ": ");
  // Serial.print(String(l_y, 0) + ": ");
  // Serial.print(String(r_x, 0) + ": ");
  Serial.print(String(l_y - r_x, 0) + ": ");
  int angle_num = 0;
  for (int i = 0; i < 8; i++){
    //Serial.print(String(mrpm[i]) + "[rpm], "); // 速度
    //Serial.print(String(abs(abs(mrpm[i]) - abs(l_y)) / abs(l_y)) + "[%], "); // 相対誤差
    //Serial.print("[" + String(current_data[i]) + "mA], "); // 電流
    if (i % 2 == 1){ // i = 1, 3, 5, 7 のとき
      // Serial.print(String(w_w[angle_num]) + "[rpm], "); // 移動方向の速度
      // Serial.print(String(w_a[angle_num]) + "[rpm], "); // 回転方向の速度
      Serial.print(String(d_a[angle_num]) + "[deg(en)], "); // 累積角度
      Serial.print(String(current_angle[angle_num], 0) + "[deg(rl)], "); // 現在の角度
      //Serial.print(String(error_angle[angle_num]) + "[deg], "); // 偏差角度
      //Serial.print(": "); // 目標速度
      //Serial.print(String(abs(abs(mrpm[i - 1]) - abs(l_y - r_x)) / abs(l_y - r_x) * abs(abs(mrpm[i]) - abs(l_y - r_x)) / abs(l_y - r_x)) + "[%], "); // 相対誤差
      //Serial.print(String(fixed_rpm[angle_num]) + "[mA], "); 
      Serial.print(String(abs(abs(current_angle[angle_num] - abs(average_angle))) / abs(average_angle)) + "[%], "); // 相対誤差
      angle_num++;
    }
  }
  // 角度の平均値を計算
  Serial.print(": ");
  Serial.print(String(average_angle, 0) + "[av_deg], "); // 平均角度
  Serial.println();
  // LEDを点滅
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // ------------------------------------------------------------ //
}