#include "mbed.h"
#include "PinNames.h"
#include "ThisThread.h"
#include <cmath>

// モーターの制御周期（秒単位）
const double motor_period = 0.001;

// PID制御のパラメータ（比例、積分、微分）
const double Kp = 0.015;  // 比例ゲイン
const double Ki = 0.0;    // 積分ゲイン
const double Kd = 0.0;    // 微分ゲイン

// 各モーターのピン設定
// 左輪
PwmOut motorL_1(PC_8);  // PWM3/3N
PwmOut motorL_2(PC_6);  // PWM3/1N
// 右輪
PwmOut motorR_1(PA_9);  // PWM1/2N
PwmOut motorR_2(PA_8);  // PWM1/1N
// 後輪
PwmOut motorB_1(PB_3);  // PWM2/2N
PwmOut motorB_2(PA_10); // PWM1/3N
// 補助モーター
PwmOut motor1(PB_10);   // PWM2/3N
PwmOut motor2(PB_5);    // PWM3/2N

// SPI通信設定（MOSI, MISO, SCLK, SS）
SPISlave device(PA_7, PA_6, PA_5, PA_4);

// モーター制御用の関数の宣言
void straight(double speed);  // 前進
void back(double speed);      // 後退
void left(double speed);      // 左旋回
void right(double speed);     // 右旋回
void stop(void);              // 停止

// PID制御の変数
double previous_error_x = 0.0;  // x方向の前回の誤差
double previous_error_y = 0.0;  // y方向の前回の誤差
double integral_x = 0.0;        // x方向の誤差の積分値
double integral_y = 0.0;        // y方向の誤差の積分値
const double dt = 0.001;        // 時間刻み幅（秒）

// メイン関数
int main() {
    // SPI通信の周波数を設定
    device.frequency(115200);

    // 各モーターのPWM周期を設定
    motorL_1.period(motor_period);
    motorL_2.period(motor_period);
    motorR_1.period(motor_period);
    motorR_2.period(motor_period);
    motorB_1.period(motor_period);
    motorB_2.period(motor_period);

    while (1) {
        // SPI通信でデータが受信された場合
        if (device.receive()) {
            int8_t distance_x = device.read();  // 最初のバイトを受信（x方向の距離）
            device.reply(0xAA);  // 確認応答

            // 2つ目のバイトが受信されるまで待機
            while (!device.receive());
            int8_t distance_y = device.read();  // 2つ目のバイトを受信（y方向の距離）
            device.reply(0xBB);  // 確認応答

            // 受信したデータを表示
            printf("%d, %d\n", distance_x, distance_y);

            // 誤差の計算（目標との差）
            double error_x = distance_x;
            double error_y = distance_y;

            // 誤差の積分値を更新
            integral_x += error_x * dt;
            integral_y += error_y * dt;

            // 誤差の微分値を計算
            double derivative_x = (error_x - previous_error_x) / dt;
            double derivative_y = (error_y - previous_error_y) / dt;

            // PID制御による出力を計算
            double output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
            double output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;

            // 出力を-1.0から1.0の範囲に制限
            output_x = fmax(-1.0, fmin(1.0, output_x));
            output_y = fmax(-1.0, fmin(1.0, output_y));

            // 前回の誤差を更新
            previous_error_x = error_x;
            previous_error_y = error_y;

            // x方向の誤差が30を超えた場合の制御
            if (abs(error_x) > 30) {
                if (output_x < 0) {
                    left(-output_x);  // 左回転
                } else {
                    right(output_x);  // 右回転
                }
            } else {
                stop();  // 停止
            }

            // y方向の誤差が30を超えた場合の制御
            if (abs(error_y) > 30) {
                if (output_y > 0) {
                    back(output_y);  // 後退
                } else {
                    straight(-output_y);  // 前進
                }
            } else {
                stop();  // 停止
            }
        }
    }
    return 0;
}

// 前進
void straight(double speed) {
    motorL_1.write(0);
    motorL_2.write(speed);
    motorR_1.write(speed);
    motorR_2.write(0);
}

// 後退
void back(double speed) {
    motorL_1.write(speed);
    motorL_2.write(0);
    motorR_1.write(0);
    motorR_2.write(speed);
}

// 左回転
void left(double speed) {
    motorL_1.write(speed);
    motorL_2.write(0);
    motorR_1.write(speed);
    motorR_2.write(0);
    motorB_1.write(speed);
    motorB_2.write(0);
}

// 右回転
void right(double speed) {
    motorL_1.write(0);
    motorL_2.write(speed);
    motorR_1.write(0);
    motorR_2.write(speed);
    motorB_1.write(0);
    motorB_2.write(speed);
}

// 停止
void stop() {
    motorL_1.write(1);
    motorL_2.write(1);
    motorR_1.write(1);
    motorR_2.write(1);
    motorB_1.write(1);
    motorB_2.write(1);
}

