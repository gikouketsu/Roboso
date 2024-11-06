// 未テスト -- change in 2024/11/6


#include "mbed.h"
#include "PinNames.h"
#include "ThisThread.h"
#include <cmath>

// 定義: モーター制御の周期 (秒)
const double motor_period = 0.001; 

// PID制御のパラメータ
const double Kp = 0.015;  // 比例ゲイン
const double Ki = 0.001;  // 積分ゲイン
const double Kd = 0.005;  // 微分ゲイン

// モーターのピン定義
PwmOut motorL_1(PC_8);  
PwmOut motorL_2(PC_6);  
PwmOut motorR_1(PA_9);  
PwmOut motorR_2(PA_8);  
PwmOut motorB_1(PB_3);  
PwmOut motorB_2(PA_10); 
PwmOut motor1(PB_10);   
PwmOut motor2(PB_5);    

// SPI通信の設定
SPISlave device(PA_7, PA_6, PA_5, PA_4);

// PID制御変数
double previous_error_x = 0.0;
double previous_error_y = 0.0;
double integral_x = 0.0;
double integral_y = 0.0;
const double dt = 0.001;  // motor_periodと一致させる

// 速度のスムーズ化処理（速度フィルタリング）
const double max_speed_change = 0.05;  // 各サイクルでの速度変化の最大許容値
double previous_output_x = 0.0;
double previous_output_y = 0.0;

// モーター制御関数のプロトタイプ宣言
void straight(double speed);
void back(double speed);
void left(double speed);
void right(double speed);
void stop();

// エラー検出の閾値
const int ERROR_THRESHOLD = 30;

// SPIからのデータが有効かどうかを確認
bool is_valid_data(int8_t data) {
    // センサーデータが-127から127の範囲にあるかチェック
    return data >= -127 && data <= 127;  
}

// 速度スムーズ化関数
double smooth_output(double current_output, double previous_output) {
    // 現在の出力と前回の出力との差を計算
    double delta = current_output - previous_output;
    // deltaがmax_speed_changeより大きい場合、変化幅を制限
    if (fabs(delta) > max_speed_change) {
        return previous_output + (delta > 0 ? max_speed_change : -max_speed_change);
    }
    return current_output;
}

// メインプログラム
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
        // SPIデータの受信処理
        if (device.receive()) {
            int8_t distance_x = device.read();
            device.reply(0xAA);  // 確認の返信

            // 2バイト目のデータを待機
            while (!device.receive());
            int8_t distance_y = device.read();
            device.reply(0xBB);  // 確認の返信

            // 受信データの有効性を確認
            if (!is_valid_data(distance_x) || !is_valid_data(distance_y)) {
                printf("Invalid data received\n");
                stop();
                continue;
            }

            // エラー（偏差）を計算
            double error_x = static_cast<double>(distance_x);
            double error_y = static_cast<double>(distance_y);

            // 積分値の更新（積分過大の防止）
            integral_x = fmax(-10, fmin(10, integral_x + error_x * dt));
            integral_y = fmax(-10, fmin(10, integral_y + error_y * dt));

            // 微分項の計算
            double derivative_x = (error_x - previous_error_x) / dt;
            double derivative_y = (error_y - previous_error_y) / dt;

            // PID出力の計算
            double output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
            double output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;

            // 出力範囲の制限
            output_x = fmax(-1.0, fmin(1.0, output_x));
            output_y = fmax(-1.0, fmin(1.0, output_y));

            // 速度スムーズ化処理の適用
            output_x = smooth_output(output_x, previous_output_x);
            output_y = smooth_output(output_y, previous_output_y);

            // 前回の出力値を更新
            previous_output_x = output_x;
            previous_output_y = output_y;

            // 前回のエラーを更新
            previous_error_x = error_x;
            previous_error_y = error_y;

            // 優先度制御ロジック
            if (abs(error_x) > ERROR_THRESHOLD) {
                if (output_x < 0) {
                    left(-output_x);
                } else {
                    right(output_x);
                }
            } else if (abs(error_y) > ERROR_THRESHOLD) {
                if (output_y > 0) {
                    back(output_y);
                } else {
                    straight(-output_y);
                }
            } else {
                stop();  // 顕著なエラーがない場合は停止
            }
        }
    }
    return 0;
}

// 前進関数
void straight(double speed) {
    motorL_1.write(0);
    motorL_2.write(speed);
    motorR_1.write(speed);
    motorR_2.write(0);
}

// 後退関数
void back(double speed) {
    motorL_1.write(speed);
    motorL_2.write(0);
    motorR_1.write(0);
    motorR_2.write(speed);
}

// 左回転関数
void left(double speed) {
    motorL_1.write(speed);
    motorL_2.write(0);
    motorR_1.write(speed);
    motorR_2.write(0);
    motorB_1.write(speed);
    motorB_2.write(0);
}

// 右回転関数
void right(double speed) {
    motorL_1.write(0);
    motorL_2.write(speed);
    motorR_1.write(0);
    motorR_2.write(speed);
    motorB_1.write(0);
    motorB_2.write(speed);
}

// 停止関数
void stop() {
    motorL_1.write(0);
    motorL_2.write(0);
    motorR_1.write(0);
    motorR_2.write(0);
    motorB_1.write(0);
    motorB_2.write(0);
}

