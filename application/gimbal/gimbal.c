#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "bsp_log.h"
#include "user_lib.h"
#include <math.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

// pitch轴整体使能开关:置1则pitch电机失力(GYRO/FREE模式下走Stop,不回水平),置0恢复编码器闭环回水平.
// 用途:调试/机械维护时临时停用pitch轴,后期随时置0启用.当前=0=启用pitch(编码器闭环回水平)
static uint8_t pitch_disabled = 0;

// pitch回水平(编码器闭环,目标=PITCH_HORIZON_ECD对应的机械单圈角+摇杆偏置)
// grav_ff:重力补偿电流前馈,由GimbalTask根据当前编码器相对水平的角度算出,供pitch电机电流环前馈使用.
//         进入云台模式才更新;ZERO_FORCE时前馈也会被清零(见DJIMotorControl对stop_flag的处理),安全.
// pitch_target_now:速率限幅后的pitch目标角(度,编码器单圈角量纲),每tick向目标逼近,防上电甩头.
// pitch_mode_active:当前是否处于下发pitch控制的云台模式(GYRO/FREE),用于模式切换时重置起点.
// pitch_stick_offset:摇杆累加的相对水平位偏置(度),由cmd通过gimbal_cmd_recv.pitch下发,0=回水平位
static float pitch_grav_ff = 0.0f;
static float pitch_target_now = 0.0f;
static uint8_t pitch_mode_active = 0;
static float pitch_stick_offset = 0.0f;
// pitch位置环反馈:相对水平角的连续量(度,过零处理).由PitchReturnToHorizon每tick更新.
// 用OTHER_FEED指向此变量,避免MOTOR_FEED走total_angle(上电有假圈数,基准与目标不一致导致发散)
static float pitch_rel_angle = 0.0f;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088 __attribute__((unused)); // 云台IMU
void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针,云台反馈仍需要IMU数据回传给cmd

    // 恢复云台电机注册: yaw(GM6020)挂CAN1, pitch(GM6020)挂CAN2.
    // 反馈来源使用IMU(OTHER_FEED),通过other_angle/speed_feedback_ptr指向gimba_IMU_data的成员.
    // 注意: IMU的Gyro[]单位为rad/s,而电机速度环measure->speed_aps为度/s,标定speed环PID时需考虑约57.3倍的单位差异.
    // TODO: 三环PID系数需实车标定;方向相反时翻motor_reverse_flag(GYRO2GIMBAL_DIR_*宏当前未在代码中引用,改它无效).
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1, // yaw在CAN1
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle, // yaw多圈总角度(度)
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[Z],       // yaw角速度(rad/s,注意单位)
            // GM6020 yaw参数(位置环IMU多圈度数 / 速度环IMU rad/s / 电流环满量程16384)
            // 响应慢=速度环MaxOut=5000被限幅卡死(spd_out顶满5000,电流才几百).抬高限幅放开上限
            // Kd+微分滤波保留压振荡,限幅大胆抬(不引振荡),Kp略加
            .angle_PID = {
                .Kp = 10.0f,                  // 加回(6→10)提响应,有Kd/滤波压振荡
                .Ki = 0.0f,
                .Kd = 0.5f,                   // 保留微分阻尼
                .MaxOut = 16000.0f,            // 位置环输出限幅抬高(10000→16000),放开目标角速度上限
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 5000.0f,
                .Derivative_LPF_RC = 0.02f,    // 微分低通滤波,抑制IMU高频噪声
            },
            .speed_PID = {
                .Kp = 1200.0f,                 // 加回(800→1200)提响应,rad/s反馈需大Kp
                .Ki = 0.0f,
                .Kd = 2.0f,                    // 保留微分阻尼(rad/s噪声大,配滤波)
                .MaxOut = 12000.0f,            // ⭐速度环限幅抬高(5000→12000),解决限幅卡死响应慢
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 6000.0f,
                .Derivative_LPF_RC = 0.02f,    // 微分低通滤波,抑制IMU高频噪声
            },
            .current_PID = {
                .Kp = 0.8f,                    // 电流环Kp略加(0.5→0.8)提升跟踪力
                .Ki = 0.0f,
                .Kd = 0.0f,                   // 电流环不加微分(内环要快)
                .MaxOut = 16384.0f,            // GM6020电流满量程
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 实测: 板子左转时电机右转, 反转输出力矩方向使闭环收敛
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    yaw_motor = DJIMotorInit(&yaw_config); // DJIMotorInit内部已调用DJIMotorEnable()

    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2, // pitch在CAN2
            .tx_id = 2,           // 与电调拨码ID一致: 绿灯1s2闪=ID2, 反馈帧0x204+2=0x206
        },
        .controller_param_init_config = {
            // 位置环用OTHER_FEED指向pitch_rel_angle(相对水平角,过零处理),避免MOTOR_FEED的total_angle上电假圈数
            .other_angle_feedback_ptr = &pitch_rel_angle,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[Y], // pitch已改编码器速度反馈,此指针不再用于pitch
            // 重力补偿电流前馈:指针指向pitch_grav_ff(由GimbalTask算出),标志位启用CURRENT_FEEDFORWARD
            .current_feedforward_ptr = &pitch_grav_ff,
            // GM6020 pitch参数(位置环相对水平度数 / 速度环编码器度/s / 电流环满量程16384)
            // 上版Kd减小+滤波后频率降幅度小=方向对,继续减Kd+加强滤波+降Kp
            .angle_PID = {
                .Kp = 20.0f,
                .Ki = 0.0f,
                .Kd = 0.3f,                // 再减(0.5→0.3)
                .MaxOut = 12000.0f,           // 位置环输出限幅=最大目标角速度(度/s)
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 6000.0f,
                .Derivative_LPF_RC = 0.02f,   // 微分滤波加强(0.01→0.02)
            },
            .speed_PID = {
                .Kp = 28.0f,                  // 再降Kp(35→28)
                .Ki = 0.0f,
                .Kd = 0.5f,                   // 再减(1→0.5)
                .MaxOut = 12000.0f,           // 速度环输出限幅=最大目标电流,保持不丢力气
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 6000.0f,
                .Derivative_LPF_RC = 0.02f,   // 微分滤波加强(0.01→0.02)
            },
            .current_PID = {
                .Kp = 0.8f,                    // 电流环Kp
                .Ki = 0.0f,
                .Kd = 0.0f,                   // 电流环不加微分(内环要快,微分易振)
                .MaxOut = 16384.0f,            // GM6020电流满量程
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
            },
        },
        .controller_setting_init_config = {
            // pitch位置环用OTHER_FEED(pitch_rel_angle相对水平角,避免total_angle上电假圈数)
            // 速度环用MOTOR_FEED(编码器speed_aps度/s,无过零问题)
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP, // 三环闭环:位置→速度→电流
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 实测:cur_out负(期望往下)时电机往上转,反转输出使力矩方向与PID一致
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD, // 启用电流前馈,叠加pitch_grav_ff重力补偿
        },
        .motor_type = GM6020,
    };
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 计算pitch编码器当前角相对水平位(PITCH_HORIZON_ECD)的短边角度差(度,过零处理)
 *        返回值:正=当前角大于水平角(按编码器增大方向),负=小于.范围[-180,180].
 *        pitch不过零时,该值就是当前pitch相对水平的真实机械偏移,可用于限位/前馈.
 *        pitch_motor未注册或无反馈时返回0.
 * @return 相对水平位的角度差(度)
 */
static float PitchEcdRelativeToHorizon()
{
    if (!pitch_motor)
        return 0.0f;
    float cur = pitch_motor->measure.angle_single_round;       // 当前单圈角(0-360)
    float hor = PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI;        // 水平位单圈角(0-360)
    float d = cur - hor;
    if (d > 180.0f)  d -= 360.0f; // 过零取短边
    if (d < -180.0f) d += 360.0f;
    return d;
}

/**
 * @brief pitch目标控制(编码器闭环,使能后回到标定的机械水平位PITCH_HORIZON_ECD)
 *        仅在进入云台模式(GYRO/FREE)后生效.上电时处于ZERO_FORCE,pitch不发力,不会甩头.
 *        位置环反馈=pitch_rel_angle(相对水平角,过零处理,OTHER_FEED),目标=相对水平量纲(0=水平)
 *        避免MOTOR_FEED的total_angle:其上电有假圈数(last_ecd=0导致total_round误判),基准与目标不一致会发散
 *        1. 每tick更新pitch_rel_angle=当前编码器相对水平角的短边差(过零处理)
 *        2. 模式切入瞬间,以当前pitch_rel_angle为起点,避免目标突变导致甩头
 *        3. 速率限幅(PITCH_RETURN_RATE度/秒)使目标向摇杆偏置平滑逼近,而非瞬变
 *        4. 重力补偿电流前馈 K·cos(rel) 由电流环叠加,rel=当前相对水平角,系数见robot_def.h
 *        5. 软件限位:目标钳位到[PITCH_MIN_ANGLE, PITCH_MAX_ANGLE](相对水平度数)
 *        摇杆偏置:cmd把左摇杆竖直(rocker_l1)累加进gimbal_cmd_recv.pitch,此处作为相对水平的目标偏置
 *        注:返回的pitch目标值(相对水平量纲,0=水平)会作为DJIMotorSetRef的输入(经angle/speed/current串级闭环)
 * @return 速率限幅后的pitch目标角(度,相对水平量纲,0=水平位)
 */
static float PitchReturnToHorizon()
{
    static const float rate_per_tick = PITCH_RETURN_RATE / 200.0f; // 200Hz GimbalTask每tick最大变化量(度)
    // 位置环反馈:当前编码器相对水平角(过零处理),0=在标定水平位.每tick更新供OTHER_FEED读取
    pitch_rel_angle = PitchEcdRelativeToHorizon();
    float rel = pitch_rel_angle;

    // 模式切入(从非激活进入激活)时,把当前相对水平角作为起点,避免目标突变
    if (!pitch_mode_active)
    {
        pitch_target_now = rel;
        pitch_mode_active = 1;
    }

    // 摇杆偏置(相对水平,度),由cmd下发.保存一份供RTT观察
    pitch_stick_offset = gimbal_cmd_recv.pitch;

    // 软件限位区间[lo,hi](相对水平度数):PITCH_MIN/MAX_ANGLE为相对水平的机械角偏移(度)
    // 0表示该方向不限制,回退用PITCH_SOFT_LIMIT兜底(对称±度)
    float lo = (PITCH_MIN_ANGLE == 0.0f) ? -PITCH_SOFT_LIMIT : (float)PITCH_MIN_ANGLE; // 下极限(俯角),0=用SOFT_LIMIT兜底
    float hi = (PITCH_MAX_ANGLE == 0.0f) ?  PITCH_SOFT_LIMIT : (float)PITCH_MAX_ANGLE; // 上极限(仰角),0=用SOFT_LIMIT兜底
    // 目标=摇杆偏置(相对水平),先被限位区间钳位(防止cmd值超界)
    float target = float_constrain(pitch_stick_offset, lo, hi);

    // 速率限幅逼近目标:每tick最多移动rate_per_tick度
    float err = target - pitch_target_now;
    if (err > rate_per_tick)
        pitch_target_now += rate_per_tick;
    else if (err < -rate_per_tick)
        pitch_target_now -= rate_per_tick;
    else
        pitch_target_now = target;

    // 目标角钳位到限位区间,防止起点/速率计算把目标推过界(软限位:外力越界靠PID拉回)
    pitch_target_now = float_constrain(pitch_target_now, lo, hi);

    // 重力补偿前馈:K·cos(rel).rel=当前相对水平角,越接近水平重力臂越长所需补偿越大(cos近似)
    // 系数PITCH_GRAVITY_FF_COEF为GM6020力矩原始量纲,符号:正=抬升pitch方向,实车标定
    pitch_grav_ff = PITCH_GRAVITY_FF_COEF * cosf(rel * PI / 180.0f);

    return pitch_target_now;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        if (yaw_motor)
            DJIMotorStop(yaw_motor);
        if (pitch_motor)
            DJIMotorStop(pitch_motor);
        pitch_mode_active = 0;    // 退出云台模式,下次切入时重新以当前pitch为起点
        pitch_grav_ff = 0.0f;     // 失能时清零重力前馈(stop_flag已会清零输出,这里同步状态)
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        if (yaw_motor)
        {
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw用IMU反馈,cmd处理多圈
        }
        // pitch编码器闭环:使能后回到标定水平位(PITCH_HORIZON_ECD)+摇杆偏置,速率限幅+重力前馈,防甩头
        // pitch_disabled=1时跳过pitch控制(Stop失力),方便后期调试/机械维护时随时启用或停用pitch轴
        if (pitch_motor)
        {
            if (pitch_disabled)
            {
                DJIMotorStop(pitch_motor);
                pitch_mode_active = 0;
                pitch_grav_ff = 0.0f;
            }
            else
            {
                DJIMotorEnable(pitch_motor);
                // pitch位置环OTHER_FEED(pitch_rel_angle相对水平角),速度环MOTOR_FEED(编码器speed_aps).初始化已设,显式确认.
                DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
                DJIMotorSetRef(pitch_motor, PitchReturnToHorizon()); // 目标=相对水平量纲(0=水平+摇杆偏置)
            }
        }
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        if (yaw_motor)
        {
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw用IMU反馈,cmd处理多圈
        }
        // pitch编码器闭环回水平(与GYRO_MODE同逻辑),pitch_disabled=1时停用
        if (pitch_motor)
        {
            if (pitch_disabled)
            {
                DJIMotorStop(pitch_motor);
                pitch_mode_active = 0;
                pitch_grav_ff = 0.0f;
            }
            else
            {
                DJIMotorEnable(pitch_motor);
                DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
                DJIMotorSetRef(pitch_motor, PitchReturnToHorizon());
            }
        }
        break;
    default:
        break;
    }



    // 回传yaw电机单圈角(0-360度,截断为uint16_t)供cmd计算offset_angle(云台-底盘夹角).
    // 未填则cmd侧angle恒为0 → offset_angle恒为 -YAW_ALIGN_ANGLE(-119.13),坐标投影(chassis.c:274-277)用错误常量扭曲运动方向.
    gimbal_feedback_data.yaw_motor_single_round_angle =
        yaw_motor ? (uint16_t)yaw_motor->measure.angle_single_round : 0;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}