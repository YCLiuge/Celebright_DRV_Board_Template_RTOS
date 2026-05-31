#include "app.h"
#include "car_attitude.h"
#include "car_control.h"
#include "config.h"
#include <math.h>

_car_control car_control;

static void get_current_distance(void);
static void get_current_spin_angle(void);
static float add_bias(float target,_Bool if_spin);
static void clear_car_control(void);

extern void Upload_Car_IntEnableAck(void);
extern void Upload_Car_OperateDoneAck(void);

/* Stub: override in application layer to handle interrupt enable ACK */
__weak void Upload_Car_IntEnableAck(void) {}
/* Stub: override in application layer to handle operation done ACK */
__weak void Upload_Car_OperateDoneAck(void) {}

/*!
 * @brief 初始化小车控制
 * @param  
 */
void init_Car_Contorl(void){
    Set_PID_Limit(&car_control.pid_line_pos,LIMIT_INC_POS,LIMIT_POS_POS,LIMIT_ITGR_POS);
    Set_PID_Limit(&car_control.pid_spin,LIMIT_INC_SPIN,LIMIT_POS_SPIN,LIMIT_ITGR_SPIN);
    Set_PID(&car_control.pid_line_pos,P_POS,I_POS,D_POS);
    Set_PID(&car_control.pid_spin,P_SPIN,I_SPIN,D_SPIN);
}

void clear_car_control(void){
    car_control.oprate_done=0;
    car_control.if_enable_interrupt=0;
    //car_control.updated=0;
    car_control.spin_parameter.start_yaw=car_state.yaw;
	  //printf("%f\n",car_control.spin_parameter.start_yaw);
    car_state.yaw_circles=0;
    Wheel_Clear_Distance();
    PID_Clear(&car_control.pid_line_pos);
    PID_Clear(&car_control.pid_spin);      /* 否则上一次 spin 的积分会残留, 新 spin 起步反向 */
    PID_Clear(&car_attitude.pid_v_angle);
}

/*!
 * @brief 控制小车
 *  自动选择模式,
 *      全为0停止
 *      只有x不为0走直线
 *      只有x,y不为0去指定点
 *      只有y,angle原地旋转,y表示旋转半径,不分正负,可为0
 *      全不为0非法,停止
 *  以前轮中点为原点,中轴向前为x正向,左侧为y正向
 * @param x 目标点x值 mm
 * @param y 目标点y值 mm
 * @param angle degree
 */
/* 浮点零值比较容差 (mm 或 degree) */
#define FLOAT_EPSILON 0.001F

static inline _Bool is_near_zero(float v) { return fabsf(v) < FLOAT_EPSILON; }

void Set_Car_Control(float x, float y, float angle){
    if(is_near_zero(y) && !is_near_zero(x) && is_near_zero(angle)){
        car_control.mode=GO_LINE;
        car_control.target_line_distance=x;

        clear_car_control();
        
        car_control.to_point_parameter.interrupt_tolerance=fabsf(car_control.target_line_distance)*LINE_INT_RATE;

        car_control.target_line_distance=add_bias(car_control.target_line_distance,0);
        car_control.to_point_parameter.interrupt_tolerance=add_bias(car_control.to_point_parameter.interrupt_tolerance,0);
    }
    else if(!is_near_zero(y) && !is_near_zero(x) && is_near_zero(angle)){
        car_control.mode=TO_POINT;
        car_control.to_point_parameter.dir=y>0?1.0F:-1.0F;
        y=fabsf(y);
        car_control.to_point_parameter.R=(x*x+y*y)/(2*y);
        float ratio = x / car_control.to_point_parameter.R;
        if (ratio > 1.0F) ratio = 1.0F;
        if (ratio < -1.0F) ratio = -1.0F;
        car_control.target_line_distance=car_control.to_point_parameter.R * asinf(ratio);
        
        clear_car_control();

        car_control.to_point_parameter.interrupt_tolerance=fabsf(car_control.target_line_distance)*LINE_INT_RATE;

        car_control.target_line_distance=add_bias(car_control.target_line_distance,0);
        car_control.to_point_parameter.interrupt_tolerance=add_bias(car_control.to_point_parameter.interrupt_tolerance,0);
    }
    else if(is_near_zero(x) && !is_near_zero(angle)){
        car_control.mode=SPIN;
        car_control.target_spin_angle=angle;
        car_control.spin_parameter.r=fabsf(y/2);
        
        clear_car_control();

        car_control.spin_parameter.interrupt_tolerance=fabsf(angle)*SPIN_INT_RATE;

        car_control.target_spin_angle=add_bias(car_control.target_spin_angle,1);
        car_control.spin_parameter.interrupt_tolerance=add_bias(car_control.spin_parameter.interrupt_tolerance,1);
    }
    else{
        car_control.mode=STOP;
    }
}

void Set_Car_Control_Absolute(float x, float y, float angle){
        car_control.mode=SPIN;
				car_control.target_spin_angle= (int)(angle + 360 - ((car_state.yaw_all>0) ? ((int)car_state.yaw_all%360) : car_state.yaw_all))%360;
        car_control.spin_parameter.r=fabsf(y/2);
        clear_car_control();

        car_control.spin_parameter.interrupt_tolerance=fabsf(angle)*SPIN_INT_RATE;

        car_control.target_spin_angle=add_bias(car_control.target_spin_angle,1);
        car_control.spin_parameter.interrupt_tolerance=add_bias(car_control.spin_parameter.interrupt_tolerance,1);	
}
/*!
 * @brief 更新小车控制当前状态
 * @param  
 */
void Car_Control_Update_Input(void){
    switch (car_control.mode)
    {
        case GO_LINE:{
            get_current_distance();
            if(fabsf(car_control.current_line_distance - car_control.target_line_distance) <=  BIAS_LINE) {
                car_control.oprate_done=1;
                Set_Car_Control(0,0,0);
            }
            if(fabsf(car_control.current_line_distance - car_control.target_line_distance) <  car_control.to_point_parameter.interrupt_tolerance){
                car_control.if_enable_interrupt=1;
            }
            break;
        }
            
        case TO_POINT:{
            get_current_distance();
            if(fabsf(car_control.current_line_distance - car_control.target_line_distance) <= BIAS_LINE) {
                car_control.oprate_done=1;
                Set_Car_Control(0,0,0);
            }
            if(fabsf(car_control.current_line_distance - car_control.target_line_distance) <  car_control.to_point_parameter.interrupt_tolerance){
                car_control.if_enable_interrupt=1;
            }
            break;
        }
            
        case SPIN:{
            get_current_spin_angle();
					//printf("%.2f\n",car_control.current_spin_angle);
            if(fabsf(car_control.current_spin_angle - car_control.target_spin_angle) < BIAS_ANGLE) {
                car_control.oprate_done=1;
                Set_Car_Control(0,0,0);
            }
            if(fabsf(car_control.current_spin_angle - car_control.target_spin_angle) <  car_control.spin_parameter.interrupt_tolerance){
                car_control.if_enable_interrupt=1;
            }
            break;
        }
            
        default:{
            break;
        }
    }
}


/*!
 * @brief 更新小车控制输出
 * @param  
 */
void Car_Control_Update_Output(void){
#if     USE_CAR_CONTROL
    float target_v_line,target_v_angle;
    switch (car_control.mode)
    {
        case GO_LINE:{
            target_v_line=PID_Cal_Pos(&car_control.pid_line_pos,car_control.current_line_distance, car_control.target_line_distance)+ car_control.to_point_parameter.v_bias;
            Set_Car_Attitude(target_v_line,0);
            break;
        }
            
        case TO_POINT:{
            target_v_line=PID_Cal_Pos(&car_control.pid_line_pos,car_control.current_line_distance,car_control.target_line_distance)+ car_control.to_point_parameter.v_bias;
            target_v_angle=car_control.to_point_parameter.dir * target_v_line / car_control.to_point_parameter.R * RAD_TO_DEGREE;
            Set_Car_Attitude(target_v_line,target_v_angle);
            break;
        }
            
        case SPIN:{
					//printf("%.2f,%.2f\n",car_control.current_spin_angle,car_control.target_spin_angle);
            target_v_angle=PID_Cal_Pos(&car_control.pid_spin,car_control.current_spin_angle,car_control.target_spin_angle);
            target_v_line=fabsf(target_v_angle)*DEGREE_TO_RAD*car_control.spin_parameter.r;
            Set_Car_Attitude(target_v_line,target_v_angle);
            break;
        }
            
        default:{
            if (g_cruise_speed != 0.0F)
                Set_Car_Attitude(g_cruise_speed, 0);
            else
                Set_Car_Attitude(0, 0);
            break;
        }
    }
#endif
}

void get_current_distance(void){
    car_control.current_line_distance = 0.5F*(Wheel_Get_Distance(LEFT)+Wheel_Get_Distance(RIGHT));
}

void get_current_spin_angle(void){
    car_control.current_spin_angle = car_state.yaw_circles*360.0F + car_state.yaw - car_control.spin_parameter.start_yaw;
}

float add_bias(float target,bool if_spin){
    bool signal = target < 0;
    if(if_spin){
        if(signal){
            return target - BIAS_ANGLE;
        }
        else{
            return target + BIAS_ANGLE;
        }
    }
    else{
        if(signal){
            return target - BIAS_LINE;
        }
        else{
            return target + BIAS_LINE;
        }
    }
    
}



