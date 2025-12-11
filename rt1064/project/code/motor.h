#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_typedef.h"


#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define MOTOR1_DIR               (C9 )
#define MOTOR1_PWM               (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR               (C7 )
#define MOTOR2_PWM               (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR               (D2 )
#define MOTOR3_PWM               (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR               (C10 )
#define MOTOR4_PWM               (PWM2_MODULE2_CHB_C11)

#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

//参数宏定义
#define ENCODER_RESOLUTION      4096.0   //编码器分辨率, 轮子转一圈，编码器产生的脉冲数
#define WHEEL_DIAMETER          0.058    //轮子直径,单位：米
#define D_X                     0.18     //底盘Y轴上两轮中心的间距
#define D_Y                     0.25     //底盘X轴上两轮中心的间距
#define PID_RATE                100       //PID调节PWM值的频率

#define LIMIT_PWM_MIN              -6000
#define LIMIT_PWM_MAX               6000

#define LIMIT_ENCODER_MIN          -400
#define LIMIT_ENCODER_MAX           400
#define ENCODER_FILTER_ALPHA       0.35f

extern int up_left_speed;
extern int up_right_speed;
extern int down_left_speed;
extern int down_right_speed;

extern int speed_three_array[3];
extern int speed_encoder[4];

void motor_init(void);
void encoder_init(void);
int Limit_int(int left_limit, int target_num, int right_limit);
void motor_pwm(int up_left_speed,int up_right_speed,int down_left_speed,int down_right_speed);
void motor_control(int* input_speed_encoder);

void encoder_read_filtered(int *enc1, int *enc2, int *enc3, int *enc4);

void Kinematics_Init(void);
void Kinematics_Inverse(int* input, int* output);

#endif
