#include "Walkinggait.h"
#include <math.h>
#include <Arduino.h>
WalkingGait::WalkingGait(){
}
void WalkingGait::initial(){
  zmp_x = 0;
  zmp_y = 0;
  width_x = 0;
  width_y = 0;
  hand = 0;
  var_theta = 0;
  footstep_x = 0;
  footstep_y = -width_size;
  displacement_x = 0;
  displacement_y = 0;
  base_x = 0;
  base_y = 0;
  sample_point = 0;
  now_step = 0;
  pre_step = -1;
  walking_state = 0;  // 步態狀態
  step_ = 99999;
  plus_lift_height = 0;
  ready_to_stop = false;
  continuous_flag = true;
}

void WalkingGait::continuous(){
  float TT_ = period_t_ * 0.001;
  time_point = sample_time * sample_point;
  t_ = ((float)(time_point % period_t_) + (float)sample_time) / 1000; //步週期內時刻(s)
  now_step = (sample_point)/(period_t_/ sample_time); 
  var_theta = theta_ / 180 * M_PI;
  if (now_step < 4){
    walking_state = 2; //start
  }
  else if (now_step == 4){
    walking_state = 1; // first
  }
  else if (now_step == step_){
    walking_state = 0; // stop
  }
  else if (now_step > step_) {
    continuous_flag = false;
    continuous_flag = false;
    continuous_flag = false;
    return;
  }
  else {
    walking_state = 3; //continuous
  }

  if (pre_step != now_step){
    if (pre_step == -1){
      footstep_x = 0;
      footstep_y = -width_size;
      now_right_x = footstep_x;
      now_right_y = -width_size;
      now_left_x = 0;
      now_left_y = width_size;
    }
    else if (pre_step % 2 == 1){
      now_right_x = footstep_x;
      now_right_y = footstep_y;
    }
    else if (pre_step % 2 == 0){
      now_left_x = footstep_x;
      now_left_y = footstep_y;
    }
  
  zmp_x = footstep_x;
  zmp_y = footstep_y;
  last_displacement_x = displacement_x;   //上次的跨幅
  last_base_x = base_x;                   //上次到達的位置
  last_displacement_y = displacement_y;   //上次的Y軸位移量
  last_base_y = base_y;                   //上次的Y軸位移位置
  last_theta = var_theta;                 //前一次的Theta變化量

  if (walking_state == 2) {//start
    var_theta = 0;
    now_width = 2 * width_size * (-pow(-1, now_step + 1));
    width_x = -sin(var_theta) * now_width;
    width_y = cos(var_theta) * now_width;
    displacement_x = width_x;
    displacement_y = width_y;
    footstep_x = footstep_x + width_x;
    footstep_y = footstep_y + width_y;   
  }
  else if (walking_state == 0) {// stop
    now_width = 2 * width_size * (-pow(-1, now_step + 1));
    width_x = -sin(var_theta) * now_width;
    width_y = cos(var_theta) * now_width;
    displacement_x = width_x;
    displacement_y = width_y;
    footstep_x = footstep_x + width_x;
    footstep_y = footstep_y + width_y;
  }
  else {
    now_width = 2 * width_size * (-pow(-1, now_step + 1));
    width_x = -sin(var_theta) * now_width;
    width_y = cos(var_theta) * now_width;
    displacement_x = step_length * cos(var_theta) - shift_length * sin(var_theta) + width_x;
    displacement_y = step_length * sin(var_theta) + shift_length * cos(var_theta) + width_y;
    footstep_x = footstep_x + displacement_x;
    footstep_y = footstep_y + displacement_y;
  }
  base_x = (footstep_x + zmp_x) / 2;
  base_y = (footstep_y + zmp_y) / 2;
  }

  hand = 10 * (1 + abs(0.1*step_length));
  if (hand > 20) {
    hand = 20;
  }
  if (step_length < 0){
    hand = -hand;
  }
  pre_step = now_step;
  if (step_length > 10 or step_length < -10) {
    plus_lift_height = 1; 
  }
  // else if (step_length > 5 or step_length < -5) {
  //   plus_lift_height = 0.5; 
  // }
  else {
    plus_lift_height = 0;
  }
  switch (walking_state) {
    case 0://stop
      Cvx = Com_vel(last_base_x, base_x, zmp_x, TT_, Tc_);
      Cpx = Com_pos(last_base_x, Cvx, zmp_x, t_, Tc_);
      Cvy = Com_vel(last_base_y, base_y, zmp_y, TT_, Tc_);
      Cpy = Com_pos(last_base_y, Cvy, zmp_y, t_, Tc_);
      Cpz = COM_HEIGHT;
      if (now_step % 2 == 0) {
        Lx = Swingfoot_pos_XY(now_left_x, (last_displacement_x + displacement_x) / 2, t_, TT_, Tdsp);
        Ly = Swingfoot_pos_XY(now_left_y, (last_displacement_y + displacement_y) / 2, t_, TT_, Tdsp);
        Lz = Swingfoot_pos_z(lift_height + plus_lift_height, t_, TT_, Tdsp);

        Rx = zmp_x;
        Ry = zmp_y;
        Rz = 0;

        Lt = 0;
        Rt = wFootTheta(-last_theta, 1, t_, TT_, Tc_);
      }
      else if (now_step % 2 == 1) {
        Lx = zmp_x;
        Ly = zmp_y;
        Lz = 0;

        Rx = Swingfoot_pos_XY(now_right_x, (last_displacement_x + displacement_x) / 2, t_, TT_, Tdsp);
        Ry = Swingfoot_pos_XY(now_right_y, (last_displacement_y + displacement_y) / 2, t_, TT_, Tdsp);
        Rz = Swingfoot_pos_z(lift_height + plus_lift_height, t_, TT_, Tdsp);

        Lt =  wFootTheta(-last_theta, 1, t_, TT_, Tc_);
        Rt = 0;
      }
      break;
    case 1://first
      Cvx = Com_vel(0, base_x, zmp_x, TT_, Tc_);
      Cpx = Com_pos(0, Cvx, zmp_x, t_, Tc_);
      Cvy = Com_vel(0, base_y, zmp_y, TT_, Tc_);
      Cpy = Com_pos(0, Cvy, zmp_y, t_, Tc_);
      Cpz = COM_HEIGHT;
      
      Lx = Swingfoot_pos_XY(now_left_x, displacement_x / 2, t_, TT_, Tdsp);
      Ly = Swingfoot_pos_XY(now_left_y, (displacement_y - now_width) / 2, t_, TT_, Tdsp);
      Lz = Swingfoot_pos_z(lift_height, t_, TT_, Tdsp);

      Rx = zmp_x;
      Ry = zmp_y;
      Rz = 0;
          
      Lt = wFootTheta(last_theta, 1, t_, TT_, Tc_);
      // Lt = 0;
      Rt = wFootTheta(-var_theta, 0, t_, TT_, Tc_);
      break;
    case 2://start
      int temp;
      Cvx = Com_vel(0, 0, zmp_x, TT_, Tc_);
      Cpx = Com_pos(0, Cvx, zmp_x, t_, Tc_);
      Cvy = Com_vel(0, 0, zmp_y, TT_, Tc_); 
      Cpz = COM_HEIGHT;
      if (now_step % 2 == 0) {
        if (now_step >= 2) {
          temp = 2;
        }
        else {
          temp = 1;
        }
        if(now_step == 0){Cpy = Com_pos(0, Cvy, zmp_y, t_, Tc_) + com_y_swing * sin(M_PI * t_ / TT_);}
        else{Cpy = Com_pos(0, Cvy, zmp_y, t_, Tc_);}
        

        Lx = Swingfoot_pos_XY(now_left_x, 0, t_, TT_, Tdsp);
        Ly = Swingfoot_pos_XY(now_left_y, 0, t_, TT_, Tdsp);
        Lz = Swingfoot_pos_z(lift_height / 2 * temp, t_, TT_, Tdsp);

        Rx = zmp_x;
        Ry = zmp_y;
        Rz = 0;

        Lt = 0;
        Rt = 0;
      }
      else if (now_step % 2 == 1) {
        if (now_step >= 2) {
          temp = 3 / 2;
        }
        else {
          temp = 1;
        }
        // Cpy = Com_pos(0, Cvy, zmp_y, t_, Tc_) - com_y_swing * sin(M_PI * t_ / TT_);
        Cpy = Com_pos(0, Cvy, zmp_y, t_, Tc_);
        Lx = zmp_x;
        Ly = zmp_y;
        Lz = 0;

        Rx = Swingfoot_pos_XY(now_right_x, 0, t_, TT_, Tdsp);
        Ry = Swingfoot_pos_XY(now_right_y, 0, t_, TT_, Tdsp);
        Rz = Swingfoot_pos_z(lift_height / 3 * 2 * temp, t_, TT_, Tdsp);

        Lt = 0;
        Rt = 0;
      }
      break;
    case 3://contiuous
      Cvx = Com_vel(last_base_x, base_x, zmp_x, TT_, Tc_);
      Cpx = Com_pos(last_base_x, Cvx, zmp_x, t_, Tc_);
      Cvy = Com_vel(last_base_y, base_y, zmp_y, TT_, Tc_);
      Cpy = Com_pos(last_base_y, Cvy, zmp_y, t_, Tc_);
      Cpz = COM_HEIGHT;
      if (now_step % 2 == 0) {
        // Cpy = Com_pos(last_base_y, Cvy, zmp_y, t_, Tc_)+ com_y_swing * sin(M_PI * t_ / TT_); 
        Lx = Swingfoot_pos_XY(now_left_x, (last_displacement_x + displacement_x) / 2, t_, TT_, Tdsp);
        Ly = Swingfoot_pos_XY(now_left_y, (last_displacement_y + displacement_y) / 2, t_, TT_, Tdsp);
        Lz = Swingfoot_pos_z(lift_height + plus_lift_height, t_, TT_, Tdsp);

        Rx = zmp_x;
        Ry = zmp_y;
        Rz = 0;

        if (var_theta * last_theta >= 0) {
          Lt = wFootTheta(-var_theta, 0, t_, TT_, Tc_);
          // Rt = 0;
          Rt = wFootTheta(last_theta, 1, t_, TT_, Tc_);
        }
        else {
          Lt = 0;
          Rt = 0;
        }
      }
      else if (now_step % 2 == 1) {
        // Cpy = Com_pos(last_base_y, Cvy, zmp_y, t_, Tc_)- com_y_swing * sin(M_PI * t_ / TT_); 
        Lx = zmp_x;
        Ly = zmp_y;
        Lz = 0;

        Rx = Swingfoot_pos_XY(now_right_x, (last_displacement_x + displacement_x) / 2, t_, TT_, Tdsp);
        Ry = Swingfoot_pos_XY(now_right_y, (last_displacement_y + displacement_y) / 2, t_, TT_, Tdsp);
        Rz = Swingfoot_pos_z(lift_height, t_, TT_, Tdsp);
        if (var_theta * last_theta >= 0) {
          // Lt = 0;
          Lt = wFootTheta(var_theta, 0, t_, TT_, Tc_);
          Rt = wFootTheta(-last_theta, 1, t_, TT_, Tc_);
        }
        else {
          Lt = 0;
          Rt = 0;
        }
      }
      break;
  }
  // if(now_step <= 3)
  // {
  //   compensation_y_hip = (compensation_swing_hip + 0) * sin(M_PI * t_ / TT_) * M_PI / 180;
  //   compensation_y_ankle = (compensation_swing_ankle + 0) * sin(M_PI * t_ / TT_) * M_PI / 180;
  // }
  // else
  // {
  if (now_step % 2 == 0)
  {
    compensation_y_hip = (compensation_swing_hip + 0) * sin(M_PI * t_ / TT_) * M_PI / 180;
    compensation_y_ankle = (compensation_swing_ankle + 0) * sin(M_PI * t_ / TT_) * M_PI / 180;
  }
  else if (now_step % 2 == 1)
  {
    compensation_y_hip = (compensation_swing_hip - 0) * sin(M_PI * t_ / TT_) * M_PI / 180;
    compensation_y_ankle = (compensation_swing_ankle + 0) * sin(M_PI * t_ / TT_) * M_PI / 180;
  }
  // }
  // Cpy = Cpy + 0.1 * (Cpy - com_y);
  // Cpx = Cpx - 0.1 * (Cpx - com_x);

  
  step_lxw = Lx - Cpx;
  step_rxw = Rx - Cpx;

  step_lyw = Ly - Cpy;
  step_ryw = Ry - Cpy;

  step_lx = step_lxw * cos(-var_theta) - step_lyw * sin(-var_theta);
  step_ly = step_lxw * sin(-var_theta) + step_lyw * cos(-var_theta);
  step_lz = Cpz - Lz;

  step_rx = step_rxw * cos(-var_theta) - step_ryw * sin(-var_theta);
  step_ry = step_rxw * sin(-var_theta) + step_ryw * cos(-var_theta);
  step_rz = Cpz - Rz;

  step_ltheta = -Lt;
  step_rtheta = -Rt;

  end_point_lx = step_lx;
  end_point_ly = step_ly - width_size;
  end_point_lz = step_lz - (COM_HEIGHT - STAND_HEIGHT);
  
  end_point_rx = step_rx;
  end_point_ry = step_ry + width_size;
  end_point_rz = step_rz - (COM_HEIGHT - STAND_HEIGHT);

  end_point_ltheta = step_ltheta;
  end_point_rtheta = step_rtheta;   

  if (ready_to_stop) {
    step_ = now_step + 1;
    ready_to_stop = false;
  }
  sample_point++;
}


float WalkingGait::Swingfoot_pos_XY(float start, float length, float t, float T, float T_DSP){
  float point = 0;
  float new_t = t-T*T_DSP/2;
  float omega = 2 * M_PI / (T * (1 - T_DSP));
  float t1 = T * T_DSP / 2;
  float t2 = T * (1 - T_DSP / 2);
  
  if (t > 0 && t <= t1) {
      point = start;
  }
  else if (t > t1 && t <= t2) {
      point = 2 * length * (omega*new_t-sin(omega*new_t)) / (2*M_PI) + start;
  }
  else {
      point = 2 * length + start;
  }
  return point;
}

float WalkingGait::Swingfoot_pos_z(float lift_height, float t_now, float T, float Tdsp){
  float pos;
  float t1 = T * Tdsp / 2;
  float t2 = T * (1 - Tdsp / 2);
  float nt = t_now - t1;
  float w = 2 * M_PI / (T * (1 - Tdsp));
            
  if (t1 < t_now && t_now <= t2) {
      pos = 0.5 * lift_height * (1 - cos(w * nt));
  }
  else {
      pos = 0;
  }
  return pos;
}

float WalkingGait::Com_vel(float x0, float xt, float px, float t, float Tc){
  return (xt - x0 * cosh(t / Tc) + px * (cosh(t / Tc) - 1)) / (Tc * sinh(t / Tc));
}

float WalkingGait::Com_pos(float x0, float vx0, float px, float t, float Tc){
  return px + x0 * cosh(t / Tc) + Tc * vx0 * sinh(t / Tc) - px * cosh(t / Tc);
}

float WalkingGait::wFootTheta(float theta, float reverse, float t, float T, float T_DSP){
  float output;
  float new_T = T*(1-T_DSP);
  float new_t = t-T*T_DSP/2;
  float omega = 2 * M_PI / (T * (1 - T_DSP));
  float t1 = T * T_DSP / 2;
  float t2 = T * (1 - T_DSP / 2);

  if (t > 0 && t <= t1) {
      if (reverse) {
        output = theta;
      }
      else {
        output = 0;
      }
  }
  else if (t > t1 && t <= t2) {
      if (reverse) {
        output = 0.5*theta*(1-cos(0.5*omega*(new_t-new_T)));
      }
      else {
        output = 0.5*theta*(1-cos(0.5*omega*new_t));
      }
  }
  else if (t > t2) {
      if (reverse) {
        output = 0;
      }
      else {
        output = theta;
      }
  }
  else {
    output = 0;
  }
  return output;
}

void WalkingGait::IMU_calculate(float roll, float pitch){
  BLA::Matrix<3> WpA;   // World to Ankle
  BLA::Matrix<3> BpA;   // Body to Ankle
  BLA::Matrix<3> Theta; // [roll, pitch, yaw]
  BLA::Matrix<3> WpB;   // World to Body result vector

  // 設定 WpA、BpA、Theta
  if (now_step % 2 == 1) {
    WpA = {Lx, Ly, 0};
    BpA = {end_point_lx, end_point_ly, -COM_HEIGHT};
    Theta = {roll, pitch, theta_};
  } else {
    WpA = {Rx, Ry, 0};
    BpA = {end_point_rx, end_point_ry, -COM_HEIGHT};
    Theta = {roll, pitch, theta_};
  }

  // 計算 ZYX 旋轉矩陣
  float cx = cos(Theta(0));
  float sx = sin(Theta(0));
  float cy = cos(Theta(1));
  float sy = sin(Theta(1));
  float cz = cos(Theta(2));
  float sz = sin(Theta(2));

  BLA::Matrix<3,3> R = {
    cz * cy,             cz * sy * sx - sz * cx,   cz * sy * cx + sz * sx,
    sz * cy,             sz * sy * sx + cz * cx,   sz * sy * cx - cz * sx,
    -sy,                 cy * sx,                  cy * cx
  };

  // 執行向量減法：WpB = WpA - R * BpA
  WpB = WpA - R * BpA;
  com_x = WpB(0);
  com_y = WpB(1);
  // 若你要回傳 WpB，請加輸出參數或成員變數儲存
}




WalkingGait walking = WalkingGait();
