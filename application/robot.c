#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit(); // 摩擦轮PWM(TIM1_CH2/CH3)+弹舱舵机(TIM1_CH1)+拨盘m2006, 取消注释以注册PWM并启动发射任务
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask(); // 处理shoot_cmd话题, 根据friction_mode/lid_mode设PWM占空比
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif

}