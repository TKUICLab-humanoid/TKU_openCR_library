#include "dynamixel_setup.h"
#include <math.h>
#include <Arduino.h>

control::control(){
}
void control::init(dynamixel::PortHandler *ph, dynamixel::PacketHandler *pkh) {
  portHandler = ph;
  packetHandler = pkh;
}

void control::motor_address(){
  // define porthandler 
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  // for X-series use differen id to separate two kind motor
  for (int id : {DXL5_ID, DXL6_ID, DXL11_ID, DXL12_ID}) {
    // setup indrectadress
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_X_PROFILE_VELOCITY + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_X_PROFILE_VELOCITY + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_X_PROFILE_VELOCITY + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_X_PROFILE_VELOCITY + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_X_PROFILE_VELOCITY + 4, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_X_PROFILE_VELOCITY + 5, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_X_PROFILE_VELOCITY + 6, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_X_PROFILE_VELOCITY + 7, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_READ + 0, ADDR_PRO_X_PRESENT_POSITION + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRO_X_PRESENT_POSITION + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRO_X_PRESENT_POSITION + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRO_X_PRESENT_POSITION + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_X_INDIRECTADDRESS_FOR_TORQUE, ADDR_PRO_X_TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
  }
  // for pro_series
  for (int id : {DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL7_ID, DXL8_ID, DXL9_ID, DXL10_ID}) {
    // setup indrectadress
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_H_PROFILE_VELOCITY + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_H_PROFILE_VELOCITY + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_H_PROFILE_VELOCITY + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_H_PROFILE_VELOCITY + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_H_GOAL_POSITION + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_H_GOAL_POSITION + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_H_GOAL_POSITION + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_H_GOAL_POSITION + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    //read
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_READ + 0, ADDR_PRO_H_PRESENT_POSITION + 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_READ + 2, ADDR_PRO_H_PRESENT_POSITION + 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_READ + 4, ADDR_PRO_H_PRESENT_POSITION + 2, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_READ + 6, ADDR_PRO_H_PRESENT_POSITION + 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_H_INDIRECTADDRESS_FOR_TORQUE, ADDR_PRO_H_TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      Serial.print(dxl_error);
    }
  }
}

void control::motor_torque(){
  // Initialize GroupSyncWrite instance to initial data adress
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_TORQUE, LEN_PRO_GOAL_AND_VELOCITY);
  uint8_t dxl_error = 0;
  uint8_t param[1] = {TORQUE_ENABLE};;
  // for X-series, the adress is velocity-position
  for (int id = 1;id < 13;id++) {
    if (!groupSyncWrite.addParam(id, param)){
      Serial.print("Failed to add parameter for ID: ");
      Serial.println(id);
    } 
  }
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    Serial.print("Failed to send Sync write packet: ");
    Serial.println(packetHandler->getTxRxResult(dxl_comm_result));
  }
  groupSyncWrite.clearParam();
}

void control::motor_groupSyncWrite(int profile_velocity[13], int goal_position[13]){
  // Initialize GroupSyncWrite instance to initial data adress
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_WRITE, LEN_PRO_GOAL_AND_VELOCITY);
  uint8_t dxl_error = 0;
  // for X-series, the adress is velocity-position
  for (int id = 1;id < 13;id++) {
    // add information for every motor
    uint8_t param_goal_position_velocity[LEN_PRO_GOAL_AND_VELOCITY];
    param_goal_position_velocity[0] = DXL_LOBYTE(DXL_LOWORD(profile_velocity[id]));
    param_goal_position_velocity[1] = DXL_HIBYTE(DXL_LOWORD(profile_velocity[id]));
    param_goal_position_velocity[2] = DXL_LOBYTE(DXL_HIWORD(profile_velocity[id]));
    param_goal_position_velocity[3] = DXL_HIBYTE(DXL_HIWORD(profile_velocity[id]));
    param_goal_position_velocity[4] = DXL_LOBYTE(DXL_LOWORD(goal_position[id]));
    param_goal_position_velocity[5] = DXL_HIBYTE(DXL_LOWORD(goal_position[id]));
    param_goal_position_velocity[6] = DXL_LOBYTE(DXL_HIWORD(goal_position[id]));
    param_goal_position_velocity[7] = DXL_HIBYTE(DXL_HIWORD(goal_position[id]));
    if (!groupSyncWrite.addParam(id, param_goal_position_velocity)){
      Serial.print("Failed to add parameter for ID: ");
      Serial.println(id);
    } 
  }
  // Send the packet to all Dynamixels
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    Serial.print("Failed to send Sync write packet: ");
    Serial.println(packetHandler->getTxRxResult(dxl_comm_result));
    return;
  }
  groupSyncWrite.clearParam();
}

void control::motor_groupSyncRead(){
  bool dxl_addparam_result;
  int id;
  int dxl_comm_result = COMM_TX_FAIL;
  int32_t dxl_present_position;
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
  for(id = 2;id < 3;id++) {
    dxl_addparam_result = groupSyncRead.addParam(id);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(id); Serial.println("groupSyncRead addparam failed");
    }
  }
  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
  for(id = 2;id < 3;id++) {
    groupSyncRead.isAvailable(id, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(id); Serial.println("groupSyncRead getdata failed");
    }
  }
  for(id = 2;id < 3;id++) {
    dxl_present_position = groupSyncRead.getData(id, ADDR_PRO_INDIRECTDATA_FOR_READ, LEN_PRO_PRESENT_POSITION);
    Serial.print("[ID:"); Serial.print(id);
    Serial.print("  PresPos:"); Serial.print(dxl_present_position);
    Serial.println(" ");
  }
  groupSyncRead.clearParam();
}

void control::Inverse_kinematic(float end_point_x, float end_point_y, float end_point_z, float end_point_theta, int RL){
  float L6 = 5.8;
  float l = 25.0;
  makeTransformMatrix(0, 0, end_point_theta, end_point_x, end_point_y, end_point_z);
  // 拆解 T 變數
  float nx = T[0][0];
  float ny = T[1][0];
  float nz = T[2][0];

  float ox = T[0][1];
  float oy = T[1][1];
  float oz = T[2][1];

  float ax = T[0][2];
  float ay = T[1][2];
  float az = T[2][2];

  float px = T[0][3];
  float py = T[1][3];
  float pz = T[2][3];

  float px2 = px + L6 * ax;
  float py2 = py + L6 * ay;
  float pz2 = pz + L6 * az;

  float L = sqrt(px2 * px2 + py2 * py2 + pz2 * pz2);
  theta[RL+4] = acos((L * L) / (2 * l * l) - 1);
  float a = acos(L / (2 * l));

  theta[RL+6] = atan2(py + l * ay, pz + l * az);
  theta[RL+5] = -atan2(px + l * ax, sqrt((py + l * ay) * (py + l * ay) + (pz + l * az) * (pz + l * az))) - a;

  float s6 = sin(theta[RL+6]);
  float c6 = cos(theta[RL+6]);
  float c45 = cos(theta[RL+4] + theta[RL+5]);
  float s45 = sin(theta[RL+4] + theta[RL+5]);
  float R_21 = ny * c45 + oy * s6 * s45 + ay * c6 * s45;
  float R_22 = oy * c6 - ay * s6;
  float R_13 = -nx * s45 + ox * s6 * c45 + ax * c6 * c45;
  float R_23 = -ny * s45 + oy * s6 * c45 + ay * c6 * c45;
  float R_33 = -nz * s45 + oz * s6 * c45 + az * c6 * c45;

  theta[RL+1] = atan2(R_13, R_33);
  float s1 = sin(theta[RL+1]);
  float c1 = cos(theta[RL+1]);

  theta[RL+2] = atan2(-R_23, R_13*s1+R_33*c1);
  theta[RL+3] = atan2(R_21, R_22);
}


void control::makeTransformMatrix(float roll, float pitch, float yaw, float px, float py, float pz) {
  // 三角函數
  float cx = cos(roll);
  float sx = sin(roll);
  float cy = cos(pitch);
  float sy = sin(pitch);
  float cz = cos(yaw);
  float sz = sin(yaw);

  // ZYX rotation matrix
  float R[3][3] = {
    {cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx},
    {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx},
    {-sy,     cy * sx,                cy * cx}
  };

  // 填入旋轉矩陣
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      T[i][j] = R[i][j];
    }
  }

  // 位置向量 px, py, pz
  T[0][3] = px;
  T[1][3] = py;
  T[2][3] = pz;

  // 最下面那一列
  T[3][0] = 0;
  T[3][1] = 0;
  T[3][2] = 0;
  T[3][3] = 1;
}

void control::rad2motor(int RL){
  float X_pi2output = 4096/(2*M_PI);
  float H_pi2output = 303750/(2*M_PI);
  float l = 5.7;
  int d = 4;
  float theta5_origin = theta[RL+5];
  float theta6_origin = theta[RL+6];
  float x = (d*sin(theta6_origin));
  float theta56_new = asin(x/l);
  // theta[RL+2] = -theta[RL+2];
  // theta[RL+6] = -theta[RL+6];
  if (RL == 0) {
    theta[5] = -theta56_new-theta5_origin;
    theta[6] = -theta56_new+theta5_origin;
  }
  else {
    theta[RL+5] = theta56_new-theta5_origin;
    theta[RL+6] = theta56_new+theta5_origin;
  }
  for (int i = 1;i < 5;i++){
    motor_position[RL+i] = round(theta[RL+i]*H_pi2output);
  }
  for (int i = 5;i < 7;i++){
    motor_position[RL+i] = round(theta[RL+i]*X_pi2output);
  }
  if (RL == 0) {
    motor_position[1] = motor_position[1];
    motor_position[4] = motor_position[4];
    motor_position[5] = 2048-motor_position[5];
    motor_position[6] = 2048-motor_position[6];
  }
  else {
    motor_position[RL+1] = -motor_position[RL+1];
    motor_position[RL+4] = -motor_position[RL+4];
    motor_position[RL+5] = 2048+motor_position[RL+5];
    motor_position[RL+6] = 2048+motor_position[RL+6];
  }
  motor_position[RL+3] = -motor_position[RL+3];
}

void control::motor_speed(int motion_delay, int RL){
  double different_theta[13];
  for(int i = 1;i < 7;i++){
    different_theta[RL+i] = abs(past_theta[RL+i] - theta[RL+i]);
    past_theta[RL+i] = theta[RL+i];
  }
  for(int i = 1;i < 7;i++){
    profile_velocity[RL+i] = round((abs(different_theta[RL+i]) / (2*M_PI)) * (1000/(float)motion_delay) * 60 / 0.00329218);
    if (i < 4 & profile_velocity[RL+i] < 3000){
      profile_velocity[RL+i] = 3000;
    }
    if (i == 4 & profile_velocity[RL+4] < 5000){
      profile_velocity[RL+i] = 5000;
    }
  }
  for(int i = 5;i < 7;i++){
    profile_velocity[RL+i] = profile_velocity[RL+i] / 80.5;
  }
}


control setupControl = control();