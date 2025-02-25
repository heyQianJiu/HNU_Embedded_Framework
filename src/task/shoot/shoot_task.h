/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
*/

#ifndef RTTHREAD_SHOOT_TASK_H
#define RTTHREAD_SHOOT_TASK_H

/**
 * @brief shoot线程入口
 */
void shoot_task_entry(void* argument);

/**
 * @brief 发射器模式
 */
typedef enum
{
    /*发射模式*/
  SHOOT_STOP=0        ,     //射击关闭
  SHOOT_ONE=1         ,     //单发模式（比赛用）
  SHOOT_CONTINUE=2,         //连续发射（日常用）
  SHOOT_REVERSE=3     ,     //堵弹反转
  SHOOT_INIT=4          ,     //初始化,用于堵转到极限以计算位置

} shoot_mode_e;
/**
 * @brief 扳机模式
 */
typedef enum
{
    /*扳机状态*/
    TRIGGER_ON=1      ,     //扳机开火状态
    TRIGGER_OFF=0     ,     //扳机闭火状态
    TRIGGER_ING=2     ,     //扳机持续状态
} trigger_mode_e;
/**
 * @brief debug mode
 */
typedef  enum
{
    DEBUG_OFF =1,
    DEBUG_ON =2,
}debug_mode_e;
/**
 * @brief frequency
 */
typedef enum
{
    LOW_FREQUENCY=1,
    MIDDLE_FREQUENCY=2,
    HIGH_FREQUENCY=3,
} shoot_frequency_e;
/**
  * @brief   发射器状态回馈
  */
//TODO:具体回馈设置待讨论
typedef enum
{
  SHOOT_OK=1,   //发射正常
  SHOOT_ERR=0,  //发射异常
  SHOOT_WAITING=2, //发射异常
} shoot_back_e;
/**
  * @brief 单发和连发角度继承
  */
typedef enum
{
    SHOOT_ANGLE_CONTINUE=0,   //角度为连发状态
    SHOOT_ANGLE_SINGLE=1,  //角度为单发状态
} shoot_angle_inherit_e;

/*供弹电机状态回馈*/
typedef  enum
{
    LOADING = 0,
    LOAD_BACK_ON = 1,
    LOAD_BACK_OK = 2,
}load_back_e;
#endif //RTTHREAD_SHOOT_TASK_H


