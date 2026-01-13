#include "stm32f10x.h"
#include "motor.h"
#include "servo.h"
#include "usart.h"

#include "ros_lib/ros.h"
#include "ros_lib/geometry_msgs/Twist.h"
#include "ros_lib/std_msgs/Float32.h"

#define CMD_TIMEOUT_MS 500

static STM32Hardware hw;
static ros::NodeHandle_<STM32Hardware> nh;
static geometry_msgs::Twist cmd_msg;

static uint32_t last_cmd_ms = 0;

/* 回调：接收速度指令 */
void cmdVelCb(const geometry_msgs::Twist& msg)
{
    last_cmd_ms = millis();
    cmd_msg = msg;
    motor_set_cmd(msg.linear.x, msg.angular.z);
}

/* 回调：接收舵机角度（0~180） */
void servoCb(const std_msgs::Float32& msg)
{
    float ang = msg.data;
    if (ang < 0.0f) ang = 0.0f;
    if (ang > 180.0f) ang = 180.0f;
    servo_set_angle(ang);
}

static ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/yolo_car/cmd_vel", &cmdVelCb);
static ros::Subscriber<std_msgs::Float32> sub_servo("/yolo_car/servo_angle", &servoCb);

static void clock_init(void)
{
    /* 使用外部 8MHz 晶振，系统时钟 72MHz。
       若你的工程已有 SystemInit()，这里无需重复配置。 */
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000); /* 1ms 节拍用于 millis() */
}

int main(void)
{
    clock_init();
    motor_init();
    servo_init();
    usart1_init(115200);

    nh.initNode();
    nh.subscribe(sub_cmd_vel);
    nh.subscribe(sub_servo);

    /* 开机先停转，舵机回中 */
    motor_stop();
    servo_set_angle(90.0f);
    last_cmd_ms = millis();

    while (1)
    {
        nh.spinOnce();

        /* 超时刹车 */
        uint32_t now = millis();
        if ((now - last_cmd_ms) > CMD_TIMEOUT_MS) {
            motor_stop();
        }

        /* 约 100 Hz 循环 */
        delay_ms(10);
    }
}
