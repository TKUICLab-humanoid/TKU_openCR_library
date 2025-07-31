#ifndef WALKINGGAIT_H_
#define WALKINGGAIT_H_

#include <math.h>
#include <BasicLinearAlgebra.h>

#define STARTSTEPCOUNTER 2
#define COM_HEIGHT       40
#define STAND_HEIGHT     48
#define LENGTH_PELVIS    19.8
#define G                981
#define SAMPLE_TIME      30

using namespace BLA;
class WalkingGait
{
public:
    WalkingGait();

    // 成員變數
    float length_pelvis = LENGTH_PELVIS;
    float com_height = COM_HEIGHT;
    float g = G;
    float Tc_ = sqrt(com_height / g);       // 機器人的自然週期
    int sample_time = SAMPLE_TIME;
    float t_;
    int time_point;

    long sample_point = 0;

    int mode = 1;                    // 控制模式
    float step_length = 25;          // 前進量
    int shift_length =  0;          // 平移量
    float theta_ = 0;                 // 旋轉量
    float var_theta = 0;

    float width_size = 9.9;  // 開腳寬度 (cm)
    int period_t_ = 1200;          // 步週期 (秒)
    float lift_height = 6;       // 擺盪腳高度 (cm)
    float com_y_swing = 0;        // 起步補償
    float compensation_swing_hip = 0; // 擺盪補償髖
    float compensation_swing_ankle = 0; // 擺盪補償踝
    float Tdsp = 0;
    float board_height = 2;
    bool ready_to_stop = false;
    bool continuous_flag = true;
    bool LC_flag = true;

    int now_step = 0;
    int pre_step = -1;
    int walking_state = 0;  // 步態狀態
    int step_ = 99999;

    float compensation_y_hip;
    float compensation_y_ankle;
    float footstep_x = 0;
    float footstep_y = -width_size;  // 預設的腳步Y位置
    float now_right_x, now_right_y;
    float now_left_x, now_left_y;
    float displacement_x = 0;
    float displacement_y = 0;
    float last_displacement_x, last_displacement_y;
    float base_x = 0;
    float base_y = 0;
    float last_base_x, last_base_y;
    float zmp_x = 0;
    float zmp_y = 0;
    float width_x = 0;
    float width_y = 0;
    float now_width;
    float hand = 0;
    float foot_lift_height, com_lift_height;
    float Cvx, Cvy;
    float Cpx, Cpy, Cpz;
    float Lx, Ly, Lz, Lt;
    float Rx, Ry, Rz, Rt;
    float step_lxw, step_lyw;
    float step_rxw, step_ryw;
    float step_lx ,step_ly ,step_lz;
    float step_rx ,step_ry, step_rz;
    float step_ltheta, step_rtheta;
    float end_point_lx, end_point_ly, end_point_lz;
    float end_point_rx, end_point_ry, end_point_rz;
    float end_point_ltheta, end_point_rtheta;

    float last_theta;
    float com_x = 0;
    float com_y = 0;

    void continuous();
    void LC();
    void initial();
    float Swingfoot_pos_XY(float start, float length, float t, float T, float T_DSP);
    float Swingfoot_pos_z(float lift_height, float t_now, float T, float Tdsp);
    float Com_vel(float x0, float xt, float px, float t, float Tc);
    float Com_pos(float x0, float xt, float px, float t, float Tc);
    float wFootTheta(float theta, float reverse, float t, float T, float T_DSP);
    void IMU_calculate(float roll, float pitch);
};

extern WalkingGait walking;

#endif /* WALKINGGAIT_H_ */
