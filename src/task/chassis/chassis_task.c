/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#define HWTIMER_DEV_NAME   "timer4"     /* 定时器名称 */
#include <rtdbg.h>

#define CHASSIS_MOTOR_NUM 2
#define SQUARE(x) ((x)*(x))
#define YAW_MOTOR 0
#define PITCH_MOTOR 1



/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;
static struct ins_msg ins_data;

static int chassis_cnt;
static publisher_t *pub_chassis;
static subscriber_t *sub_cmd,*sub_ins;

static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */

static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
    pid_obj_t *angle_pid;
}chassis_controller[CHASSIS_MOTOR_NUM];

static dji_motor_object_t *chassis_motor[CHASSIS_MOTOR_NUM];  // 底盘电机实例
static int16_t motor_ref_rpm[CHASSIS_MOTOR_NUM]; // 电机控制期望rpm
// static float motor_ref_angle[CHASSIS_MOTOR_NUM]; //电机控制期望angle

static void chassis_motor_init();
/*定时器初始化*/
static int TIM_Init(void);
/*角度数据*/
static float pitch_angle,yaw_angle;
/*里程计所需数据*/
static float x_sin_w,x_cos_w,y_sin_w,y_cos_w;

/* --------------------------------- 底盘线程入口 --------------------------------- */
static float cmd_dt;

void chassis_thread_entry(void *argument)
{
    static float cmd_start;

    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();
    TIM_Init();

    LOG_I("Chassis Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 计算实际速度 */

        /* 更新该线程所有的订阅者 */
        chassis_sub_pull();

        chassis_cnt++;
        chassis_cnt%=1000;

        for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            dji_motor_enable(chassis_motor[i]);
        }

        switch (chassis_cmd.ctrl_mode)
        {
            case CHASSIS_RELAX:
                for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
                {
                    dji_motor_relax(chassis_motor[i]);
                }
                break;
            case CHASSIS_OPEN_LOOP:
                motor_ref_rpm[YAW_MOTOR] =chassis_cmd.vw_yaw;
                motor_ref_rpm[PITCH_MOTOR] = chassis_cmd.vw_pitch;

            break;
            case CHASSIS_STOP:
                motor_ref_rpm[YAW_MOTOR] = 0;
                motor_ref_rpm[PITCH_MOTOR] = 0;
            break;
            // case CHASSIS_INIT://角度环控制的，先不启用

            default:
                for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
                {
                    dji_motor_relax(chassis_motor[i]);
                }
                break;
            }

        /* 更新发布该线程的msg */
        chassis_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
            LOG_E("Chassis Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}

/**
 * @brief chassis 线程中所有发布者初始化
 */
static void chassis_pub_init(void)
{
    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    pub_push_msg(pub_chassis,&chassis_fdb);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    sub_get_msg(sub_cmd, &chassis_cmd);
    sub_get_msg(sub_ins, &ins_data);
}

/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static rt_int16_t motor_control_yaw(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(rt_int16_t) pid_calculate(chassis_controller[YAW_MOTOR].speed_pid, measure.speed_rpm, motor_ref_rpm[YAW_MOTOR]);
    return set;
}
// static rt_int16_t motor_control_yaw(dji_motor_measure_t measure)
// {
//     /* PID局部指针，切换不同模式下PID控制器 */
//     static pid_obj_t *pid_angle;
//     static pid_obj_t *pid_speed;
//     static float get_speed, get_angle;  // 闭环反馈量
//     static float pid_out_angle;         // 角度环输出
//     static rt_int16_t send_data;        // 最终发送给电调的数据
//
//     /*串级pid，一个角度环和一个速度环*/
//     pid_speed = chassis_controller[YAW_MOTOR].speed_pid;
//     pid_angle = chassis_controller[YAW_MOTOR].angle_pid;
//     get_angle=measure.total_angle;
//     get_speed=measure.speed_rpm;
//
//     /* 切换模式需要清空控制器历史状态 */
//     if(chassis_cmd.ctrl_mode != chassis_cmd.last_mode)
//     {
//         pid_clear(pid_angle);
//         pid_clear(pid_speed);
//     }
//
//     /*pid计算输出*/
//     //双环pid,CHASSIS_STOP
//     if(chassis_cmd.ctrl_mode == CHASSIS_STOP) {
//         pid_out_angle = (int16_t) pid_calculate(pid_angle, get_angle, motor_ref_rpm[YAW_MOTOR]);  // 编码器增长方向与imu相反
//         send_data = (int16_t) pid_calculate(pid_speed, get_speed, pid_out_angle);     // 电机转动正方向与imu相反
//
//     }
//     //单环pid,CHASSIS OPEN LOOP
//     else if(chassis_cmd.ctrl_mode==CHASSIS_OPEN_LOOP)//，只用速度环
//     {
//         send_data = (int16_t) pid_calculate(pid_speed, get_speed, motor_ref_rpm[YAW_MOTOR] );
//     }
//     // VAL_LIMIT(gim_motor_ref[PITCH], PIT_ANGLE_MIN, PIT_ANGLE_MAX);
//     return send_data;
// }
static rt_int16_t motor_control_pitch(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(rt_int16_t) pid_calculate(chassis_controller[PITCH_MOTOR].speed_pid, measure.speed_rpm, motor_ref_rpm[PITCH_MOTOR]);
    return set;
}

/* 底盘每个电机对应的控制函数 */
static void *motor_control[2] =
{
    motor_control_yaw,
    motor_control_pitch,
};

motor_config_t chassis_motor_config[2] =
    {
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = YAW_MOTOR_ID,
        .controller = &chassis_controller[YAW_MOTOR],
    },
{
    .motor_type = M2006,
    .can_name = CAN_CHASSIS,
    .rx_id = PITCH_MOTOR_ID,
    .controller = &chassis_controller[PITCH_MOTOR],
    }
    };
/**
 * @brief 注册底盘电机及其控制器初始化
 */
static void chassis_motor_init()
{
    /*yaw*/
    pid_config_t yaw_speed_config = INIT_PID_CONFIG(YAW_KP_V, YAW_KI_V, YAW_KD_V, YAW_INTEGRAL_V, YAW_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    // pid_config_t yaw_angle_config = INIT_PID_CONFIG(YAW_KP_V, YAW_KI_V, YAW_KD_V, YAW_INTEGRAL_V, YAW_MAX_V,
    //                                                     (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    chassis_controller[YAW_MOTOR].speed_pid = pid_register(&yaw_speed_config);
    // chassis_controller[YAW_MOTOR].angle_pid = pid_register(&yaw_angle_config);
    chassis_motor[YAW_MOTOR] = dji_motor_register(&chassis_motor_config[YAW_MOTOR], motor_control[YAW_MOTOR]);
    /*pitch*/
    pid_config_t pitch_speed_config = INIT_PID_CONFIG(PITCH_KP_V, PITCH_KI_V, PITCH_KD_V, PITCH_INTEGRAL_V, PITCH_MAX_V,
                                                            (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    chassis_controller[PITCH_MOTOR].speed_pid = pid_register(&pitch_speed_config);
    chassis_motor[PITCH_MOTOR] = dji_motor_register(&chassis_motor_config[PITCH_MOTOR], motor_control[PITCH_MOTOR]);

}



/**
 * @brief 全向轮底盘运动逆解算求解实际速度(x,y,w是相对于底盘的x，y，w的速度)
 *
 * @param TODO:具体数值正负待定，由测试得正确结果
 * @param
 */

static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{

    // /*暂时写在这里*/
    // yaw_angle = chassis_motor[YAW_MOTOR]->measure.total_angle * YAW_GEAR_RATIO;// 总角度 * 减速比 = 实际角度,正为顺时针,负为逆时针
    // // pitch_angle = pitch_angle_cal(chassis_motor[PITCH_MOTOR]->measure.total_angle);
    chassis_fdb.yaw_degree = yaw_angle;//直接读取ins数据
    chassis_fdb.pitch_degree = pitch_angle;
    /*这两行不能注释，否则进死循环*/
    chassis_fdb.x_pos_gim=x_cos_w + x_sin_w;
    chassis_fdb.y_pos_gim=y_cos_w - y_sin_w;
    /*这两行不能注释，否则进死循环*/
    return 0;
}

int TIM_Init(void)
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    rt_device_t hw_dev = RT_NULL;   /* 定时器设备句柄 */
    rt_hwtimer_mode_t mode;         /* 定时器模式 */
    rt_uint32_t freq = 10000;               /* 计数频率 */

    /* 查找定时器设备 */
    hw_dev = rt_device_find("timer4" );
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    timeout_s.sec = 0;      /* 秒 */
    timeout_s.usec = 1000;     /* 微秒 */
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }
}

