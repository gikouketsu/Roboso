#include "mbed.h"
#include "PinNames.h"
#include "ThisThread.h"
#include <cmath>

// 定义马达的周期
const double motor_period = 0.001;

// 定义每个马达的pin
// 左轮
PwmOut motorL_1(PC_8);  // PWM3/3N
PwmOut motorL_2(PC_6);  // PWM3/1N
// 右轮
PwmOut motorR_1(PA_9);  // PWM1/2N
PwmOut motorR_2(PA_8);  // PWM1/1N
// 后轮
PwmOut motorB_1(PB_3); //PWM2/2N
PwmOut motorB_2(PA_10); // PWM1/3N 
// 辅助
PwmOut motor1(PB_10); // PWM2/3N
PwmOut motor2(PB_5); // PWM3/2N

// 定义SPI通讯
SPISlave device(PA_7, PA_6, PA_5, PA_4);

// 驱动
// 直走
void straight(void);
// 后退
void back(void);
// 左转
void left(void);
// 右转
void right(void);
// 停止
void stop(void);


// 主函数
int main(){

    // 设置一个较高的SPI频率
    device.frequency(115200);

    //
    motorL_1.period(motor_period);
    motorL_2.period(motor_period);
    motorR_1.period(motor_period);
    motorR_2.period(motor_period);
    motorB_1.period(motor_period);
    motorB_2.period(motor_period);

    while (1) {
        // 如果接收到数据
        if (device.receive()) {
            // 一次性读取两个字节的数据
            int8_t distance_x = device.read();  // 读取第一个字节
            device.reply(0xAA);  // 回复第一个字节的确认

            // 等待第二个字节到达
            while (!device.receive());

            int8_t distance_y = device.read();  // 读取第二个字节
            device.reply(0xBB);  // 回复第二个字节的确认

            // 显示收到的数据
            printf("%d, %d\n", distance_x, distance_y);

            if(distance_x < -10){
                left();

            }else if(distance_y > 10){
                back();

            }else if(distance_y < -10){
                straight();

            }else{
                stop();
            }

            if(distance_x > 10){
                right();

            }else if(distance_y > 10){
                back();

            }else if(distance_y < -10){
                straight();

            }else{
                stop();
            }
        }
        


    }
    return 0;
}

void straight(){
    motorL_1.write(0);
    motorL_2.write(0.7);
    motorR_1.write(0.7);
    motorR_2.write(0);
}
void back(){
    motorL_1.write(0.7);
    motorL_2.write(0);
    motorR_1.write(0);
    motorR_2.write(0.7);
}
void left(){
    motorL_1.write(0.5);
    motorL_2.write(0);
    motorR_1.write(0.5);
    motorR_2.write(0);
    motorB_1.write(0.5);
    motorB_2.write(0);
}
void right(){
    motorL_1.write(0);
    motorL_2.write(0.5);
    motorR_1.write(0);
    motorR_2.write(0.5);
    motorB_1.write(0);
    motorB_2.write(0.5);
}
void stop(){
    motorL_1.write(1);
    motorL_2.write(1);
    motorR_1.write(1);
    motorR_2.write(1);
    motorB_1.write(1);
    motorB_2.write(1);
}

