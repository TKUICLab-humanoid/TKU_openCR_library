#include <Arduino.h>
#include <RTOS.h>
#include "dynamixel_setup.h"
#include "Walkinggait.h"

// 執行緒 ID
osThreadId thread_id_control;
osThreadId thread_id_walking;

int state;
bool head_flag = false;
uint8_t position_velocity[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const uint32_t interval_ms = 30;

// === 前向宣告 ===
static void Thread_Control(void const *argument);
static void Thread_Walking(void const *argument);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  setupControl.init(portHandler, packetHandler);
  Serial.println("Start..");
  if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Failed to initialize port.");
    return;
  }
  Serial.println("Port initialized!");

// 設定給 control class
  setupControl.motor_address();
  setupControl.motor_torque();
  setupControl.Inverse_kinematic(0, 0, 48, 0, 0);
  setupControl.Inverse_kinematic(0, 0, 48, 0, 6);
  setupControl.rad2motor(0);
  setupControl.rad2motor(6);
  for (int i = 0; i < 12; ++i) setupControl.origin_position[i] = setupControl.motor_position[i];
  // 建立執行緒
  osThreadDef(THREAD_CONTROL, Thread_Control, osPriorityNormal, 0, 2048);
  osThreadDef(THREAD_WALKING, Thread_Walking, osPriorityNormal, 0, 2048);
  thread_id_control = osThreadCreate(osThread(THREAD_CONTROL), NULL);
  thread_id_walking = osThreadCreate(osThread(THREAD_WALKING), NULL);

  // 啟動 RTOS 核心
  osKernelStart();
}

void loop() {
  // 不需要做任何事，RTOS 已經接管
}

// ================ 執行緒：處理輸入與狀態切換 ================
static void Thread_Control(void const *argument) {
  (void)argument;

  while (1) {
    if (Serial.available() > 0) {
      int newState = Serial.parseInt();
      if (newState >= 1 && newState <= 5) {
        state = newState;
        Serial.print("切換到狀態: ");
        Serial.println(state);
      } 
      else {
        Serial.println("無效輸入，請輸入 1~5");
      }
    }

    if (state == 1) {
      Serial.println("狀態: 伸直");
      for (int i = 0; i < 4; i++) setupControl.motor_position[i] = 0;
      for (int i = 6; i < 10; i++) setupControl.motor_position[i] = 0;
      setupControl.motor_position[4] = 2048;
      setupControl.motor_position[5] = 2048;
      setupControl.motor_position[10] = 2048;
      setupControl.motor_position[11] = 2048;
      setupControl.walking_groupSyncWrite(setupControl.profile_velocity, setupControl.motor_position);
      osDelay(1000);
      state = 0;
    } else if (state == 2) {
      Serial.println("狀態: 站立");
      setupControl.Inverse_kinematic(0, 0, 48, 0, 0);
      setupControl.Inverse_kinematic(0, 0, 48, 0, 6);
      for (int i = 0; i < 12; i++) Serial.println(setupControl.theta[i]);
      setupControl.rad2motor(0);
      setupControl.rad2motor(6);
      setupControl.walking_groupSyncWrite(setupControl.profile_velocity, setupControl.motor_position);
      for (int i = 0; i < 12; i++) setupControl.past_theta[i] = setupControl.theta[i];
      for (int i = 0; i < 12; i++) Serial.println(setupControl.motor_position[i]);
      osDelay(1000);
      state = 0;
    } else if (state == 3) {
      walking.ready_to_stop = true;
      state = 4;
    osDelay(50);  // 短暫休息避免佔滿 CPU
   }
 }
}

// ================ 執行緒：每 30ms 更新一次步態控制 ================
static void Thread_Walking(void const *argument) {
  (void)argument;
  uint32_t timestamp = 0;
  uint32_t now_timestamp = 0;
  while (1) {
    if (state == 4 && walking.continuous_flag) {
      timestamp = osKernelSysTick();
      walking.continuous();
      setupControl.Inverse_kinematic(-walking.end_point_rx, -walking.end_point_ry, walking.end_point_rz, walking.end_point_rtheta, 0);
      if (walking.now_step % 2 == 1){
        setupControl.theta[1] -= walking.compensation_y_hip;
        setupControl.theta[5] += walking.compensation_y_ankle;
      }
      setupControl.rad2motor(0);
      setupControl.motor_speed(walking.sample_time, walking.step_length, 0);
      setupControl.Inverse_kinematic(-walking.end_point_lx, -walking.end_point_ly, walking.end_point_lz, walking.end_point_ltheta, 6);
      if (walking.now_step % 2 == 0){
        setupControl.theta[7] += walking.compensation_y_hip;
        setupControl.theta[11] -= walking.compensation_y_ankle;
      }
      setupControl.rad2motor(6);
      setupControl.motor_speed(walking.sample_time, walking.step_length, 6);
      setupControl.walking_groupSyncWrite(setupControl.profile_velocity, setupControl.motor_position);
      now_timestamp = osKernelSysTick();
    } 
    else if (state == 5 && walking.LC_flag){
      timestamp = osKernelSysTick();
      walking.LC();
      setupControl.Inverse_kinematic(-walking.end_point_lx, -walking.end_point_ly, walking.end_point_lz, walking.end_point_ltheta, 0);
      setupControl.rad2motor(0);
      setupControl.motor_speed(walking.sample_time, walking.step_length, 0);
      setupControl.Inverse_kinematic(-walking.end_point_rx, -walking.end_point_ry, walking.end_point_rz, walking.end_point_rtheta, 6);
      setupControl.rad2motor(6);
      setupControl.motor_speed(walking.sample_time, walking.step_length, 6);
      setupControl.walking_groupSyncWrite(setupControl.profile_velocity, setupControl.motor_position);
      now_timestamp = osKernelSysTick();
      Serial.print("left:");
      Serial.println(walking.end_point_lz);
      Serial.print("right:");
      Serial.println(walking.end_point_rz);
    } 
    else {
      state = 0;
      walking.initial();  // 停止時初始化
      osDelay(timestamp - now_timestamp + 30);
    } 
    osDelay(timestamp - now_timestamp + 30);
  }
}
