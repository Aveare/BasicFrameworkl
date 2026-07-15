#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "bsp_pwm.h"
#include "tim.h"

/* ========== 摩擦轮PWM占空比查找表 ==========
 * 索引对应 Friction_Duty_Level_e 枚举值，封装油门数值便于阅读与标定
 * c615 PWM协议（50Hz/20ms基准）：0.05(1ms)=停止  0.075(1.5ms)=中速  0.10(2ms)=全速
 * TODO: 以下值需实车标定，当前为键盘项目实测起点值
 */
static const float friction_duty_table[] = {
    [FRICTION_DUTY_STOP]    = 0.050f,  // → 1.0ms   停止
    [FRICTION_DUTY_15MPS]   = 0.065f,  // → 1.3ms   15m/s弹速油门（约65%推力，需标定）
    [FRICTION_DUTY_18MPS]   = 0.070f,  // → 1.4ms   18m/s弹速油门（约70%推力，需标定）
    [FRICTION_DUTY_DEFAULT] = 0.074f,  // → 1.48ms  默认油门（对应键盘项目CCR≈1480）
    [FRICTION_DUTY_30MPS]   = 0.085f,  // → 1.7ms   30m/s弹速油门（约85%推力，需标定）
    [FRICTION_DUTY_FULL]    = 0.100f,  // → 2.0ms   全速（100%推力）
};

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *loader; // 拨盘电机(M2006). 摩擦轮已改为PWM控制(c615电调),不再走CAN
static PWMInstance *lid_servo = NULL;                      // 弹舱盖舵机PWM实例（TIM1 CH1 / PE9）
static PWMInstance *friction_l_pwm = NULL;                 // 左摩擦轮PWM（TIM1 CH2 / PE11 / c615电调）
static PWMInstance *friction_r_pwm = NULL;                 // 右摩擦轮PWM（TIM1 CH3 / PE13 / c615电调）

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 摩擦轮已改为PWM控制(c615电调, TIM1 CH2/CH3), 下面的CAN版M3508摩擦轮初始化已注释停用
    // Motor_Init_Config_s friction_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //     },
    //     .controller_param_init_config = {
    //         .speed_PID = {
    //             .Kp = 0, // 20
    //             .Ki = 0, // 1
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit,
    //             .IntegralLimit = 10000,
    //             .MaxOut = 15000,
    //         },
    //         .current_PID = {
    //             .Kp = 0, // 0.7
    //             .Ki = 0, // 0.1
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit,
    //             .IntegralLimit = 10000,
    //             .MaxOut = 15000,
    //         },
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //
    //         .outer_loop_type = SPEED_LOOP,
    //         .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //     },
    //     .motor_type = M3508};
    // friction_config.can_init_config.tx_id = 1;
    // friction_l = DJIMotorInit(&friction_config);
    //
    // friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    // friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    // friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 0, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 0, // 10
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 2006
    };
    loader = DJIMotorInit(&loader_config);

    // 初始化弹舱盖舵机（TIM1 CH1 / PE9，50Hz 标准舵机PWM）
    PWM_Init_Config_s lid_config = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_1,
        .period = 0.02f,       // 20ms（50Hz）
        .dutyratio = 0.075f,    // 1.5ms，中间位置（初始）
    };
    lid_servo = PWMRegister(&lid_config);

    // 初始化左右摩擦轮PWM（TIM1 CH2 / PE11, TIM1 CH3 / PE13，50Hz 适配c615电调）
    // c615电调PWM协议：1ms(0.05)=停止, 1.5ms(0.075)=中速, 2ms(0.10)=全速
    PWM_Init_Config_s friction_l_pwm_config = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_2,
        .period = 0.02f,
        .dutyratio = 0.05f,  // 初始停止
    };
    friction_l_pwm = PWMRegister(&friction_l_pwm_config);

    PWM_Init_Config_s friction_r_pwm_config = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_3,
        .period = 0.02f,
        .dutyratio = 0.05f,  // 初始停止
    };
    friction_r_pwm = PWMRegister(&friction_r_pwm_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(loader);
        PWMSetDutyRatio(friction_l_pwm, friction_duty_table[FRICTION_DUTY_STOP]); // 1ms → c615停止
        PWMSetDutyRatio(friction_r_pwm, friction_duty_table[FRICTION_DUTY_STOP]);
    }
    else // 恢复运行
    {
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮（PWM控制，TIM1 CH2/CH3 → c615电调 → Snail 2305）
    // duty值由 friction_duty_table[] 统一管理，索引见 Friction_Duty_Level_e
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            PWMSetDutyRatio(friction_l_pwm, friction_duty_table[FRICTION_DUTY_15MPS]);
            PWMSetDutyRatio(friction_r_pwm, friction_duty_table[FRICTION_DUTY_15MPS]);
            break;
        case SMALL_AMU_18:
            PWMSetDutyRatio(friction_l_pwm, friction_duty_table[FRICTION_DUTY_18MPS]);
            PWMSetDutyRatio(friction_r_pwm, friction_duty_table[FRICTION_DUTY_18MPS]);
            break;
        case SMALL_AMU_30:
            PWMSetDutyRatio(friction_l_pwm, friction_duty_table[FRICTION_DUTY_30MPS]);
            PWMSetDutyRatio(friction_r_pwm, friction_duty_table[FRICTION_DUTY_30MPS]);
            break;
        default: // 默认摩擦轮速度（对应键盘项目的1480）
            PWMSetDutyRatio(friction_l_pwm, friction_duty_table[FRICTION_DUTY_DEFAULT]);
            PWMSetDutyRatio(friction_r_pwm, friction_duty_table[FRICTION_DUTY_DEFAULT]);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        PWMSetDutyRatio(friction_l_pwm, friction_duty_table[FRICTION_DUTY_STOP]);
        PWMSetDutyRatio(friction_r_pwm, friction_duty_table[FRICTION_DUTY_STOP]);
    }

    // 开关弹舱盖（TIM1 CH1 / PE9 舵机PWM控制）
    // PWM对应关系（50Hz/20ms基准）：
    //   duty=0.105 → 2.1ms脉冲 → 弹舱打开（与键盘项目TIM5 CH4/2100一致）
    //   duty=0.045 → 0.9ms脉冲 → 弹舱关闭（与键盘项目TIM5 CH4/900一致）
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        PWMSetDutyRatio(lid_servo, 0.045f);  // 0.9ms → 舵机关闭位置
        // TODO: judge_send_mesg.client_custom_data.masks &= ~0x08;  // 裁判系统第3位清零(关闭)
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        PWMSetDutyRatio(lid_servo, 0.105f);  // 2.1ms → 舵机打开位置
        // TODO: judge_send_mesg.client_custom_data.masks |= 0x08;   // 裁判系统第3位置1(打开)
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}