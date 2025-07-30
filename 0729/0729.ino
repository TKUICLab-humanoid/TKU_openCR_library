#include <Arduino.h>
#include <RTOS.h>
#include "dynamixel_setup.h"
#include "Walkinggait.h"
// 加在全局区，Thread_Control 之外
// debug 用
#define DBG SerialUsb
#define PAYLOAD_BYTES 96
// ACK + Dynamixel 用
// #define COM Serial
constexpr uint8_t SW_PIN = 53;
static bool last_sw_status = HIGH;


// === 狀態與 IMU ===
osThreadId thread_id_control;
osThreadId thread_id_walking;
osThreadId thread_id_imu;
// osThreadId thread_id_cmd;
osThreadId thread_id_switch;
int state;

String imu_data = "";
bool receiving = false;

float imu_yaw = 0.0, imu_pitch = 0.0, imu_roll = 0.0;
float imu_yaw_offset = 0.0, imu_pitch_offset = 0.0, imu_roll_offset = 0.0, r = 0, p = 0 ,y = 0;
uint8_t position_velocity[PAYLOAD_BYTES];

unsigned long last_receive_time = 0;
const uint32_t interval_ms = 30;

// --- 1) Thread 原型宣告 ---
static void Thread_Control(void const *argument);
static void Thread_Walking(void const *argument);
static void Thread_IMU(void const *argument);
static void Thread_Switch(void const *argument);


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial4.begin(115200);
  while (!Serial);

  pinMode(SW_PIN, INPUT_PULLUP);

  // Dynamixel init
  dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  setupControl.init(portHandler, packetHandler);

  Serial.println("Start..");
  if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Failed to initialize port.");
    return;
  }
  Serial.println("Port initialized!");


  setupControl.Inverse_kinematic(0, 0, 50, 0, 0);
  setupControl.Inverse_kinematic(0, 0, 50, 0, 6);
  setupControl.rad2motor(0);
  setupControl.rad2motor(6);
  for (int i = 0; i < 12; ++i) setupControl.origin_position[i] = setupControl.motor_position[i];

  // 建立執行緒
  osThreadDef(THREAD_CONTROL, Thread_Control, osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_WALKING, Thread_Walking, osPriorityHigh, 0, 2048);
  osThreadDef(THREAD_IMU,     Thread_IMU,     osPriorityLow, 0, 512);
  osThreadDef(THREAD_SWITCH, Thread_Switch,   osPriorityLow, 0, 256);

  thread_id_imu     = osThreadCreate(osThread(THREAD_IMU),     nullptr);
  thread_id_control = osThreadCreate(osThread(THREAD_CONTROL), nullptr);
  thread_id_walking = osThreadCreate(osThread(THREAD_WALKING), nullptr);
  thread_id_switch  = osThreadCreate(osThread(THREAD_SWITCH) , nullptr);

  // 啟動 RTOS
  osKernelStart();
}

void loop() {
  // RTOS 已接管，不用在這裡做事
}

static void Thread_Switch(void const *argument){
  (void)argument;
  while(1){
    bool cur = digitalRead(SW_PIN);
    if (cur == LOW && last_sw_status == HIGH){
      setupControl.motor_address();
      delay(10);
      setupControl.motor_torque();
      delay(10);
      setupControl.motor_groupSyncWrite(position_velocity);
      Serial.println("#ack=swtich_is_true");

      last_sw_status = LOW;
    }
    else if (cur == HIGH && last_sw_status == LOW){
      // setupControl.motor_torque_disable();
      Serial.println("#ack=swtich_is_false");
      last_sw_status = HIGH;
    }
    osDelay(30);
  }
}

static void Thread_IMU(void const *argument) {
  (void)argument;
  while (1) {
    while (Serial2.available()) {
      char inChar = (char)Serial2.read();
      if (inChar == '#') {
        imu_data = "";
        receiving = true;
      } else if (inChar == '\n') {
        receiving = false;
        parseIMUData(imu_data);
        last_receive_time = millis();
      } else if (receiving) {
        imu_data += inChar;
      }
    }
    if (Serial4.available()) {
      char cmd = Serial4.read();
      if (cmd == 0) {
        imu_yaw_offset   = imu_yaw;
        imu_pitch_offset = imu_pitch;
        imu_roll_offset  = imu_roll;
      }
    }
    osDelay(30);
  }
}

// static void Thread_Command(void const *argument) {
//   (void)argument;
//   while (1) {
//     if (Serial4.available()) {
//       char cmd = Serial4.read();
//       if (cmd == 0) {
//         imu_yaw_offset   = imu_yaw;
//         imu_pitch_offset = imu_pitch;
//         imu_roll_offset  = imu_roll;
//       }
//     }
//     osDelay(10);
//   }
// }

float normalizeAngle180(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void parseIMUData(String data) {
  if (data.startsWith("YPR=")) {
    data = data.substring(4);
    int c1 = data.indexOf(','), c2 = data.indexOf(',', c1+1);
    if (c1 > 0 && c2 > c1) {
      imu_yaw   = data.substring(0,   c1).toFloat();
      imu_pitch = data.substring(c1+1, c2).toFloat();
      imu_roll  = data.substring(c2+1).toFloat();

      y = -normalizeAngle180(imu_yaw   - imu_yaw_offset);
      p = -normalizeAngle180(imu_pitch - imu_pitch_offset);
      r = -normalizeAngle180(imu_roll  - imu_roll_offset);

      String out = "#YPR=" + String(y,2) + "," + String(p,2) + "," + String(r,2);
      Serial4.println(out);
    }
  }
}

// ================ 執行緒：處理輸入與狀態切換 ================
static void Thread_Control(void const *argument) {
  // 把 void* argument 转回 control*
  control* self = (control*)argument;
  (void)argument;
  float com_y_swing, y_swing_range, period_t_;
  float osc_lockrange, base_default_z, now_stand_height, now_com_height;
  bool stand_balance;
  uint8_t footer;

  while (1) {
    while (Serial.available() >= 1) {
      uint8_t header = Serial.read();

    if (header == 0x48 && Serial.available() >= sizeof(float)*7 + sizeof(bool) + 1) {
        // 原 0x48 Loading
        Serial.readBytes((char*)&com_y_swing,    sizeof(float));
        Serial.readBytes((char*)&y_swing_range,  sizeof(float));
        Serial.readBytes((char*)&period_t_,       sizeof(float));
        Serial.readBytes((char*)&osc_lockrange,  sizeof(float));
        Serial.readBytes((char*)&base_default_z, sizeof(float));
        Serial.readBytes((char*)&now_stand_height,sizeof(float));
        Serial.readBytes((char*)&now_com_height, sizeof(float));
        Serial.readBytes((char*)&stand_balance,  sizeof(bool));

        footer = Serial.read();
        if (footer == 0x45) {
          setupControl.motor_address();
          setupControl.motor_torque();
          walking.com_y_swing  = com_y_swing;
          walking.width_size   = y_swing_range;
          walking.period_t_    = period_t_;
          walking.Tdsp          = osc_lockrange;
          walking.lift_height  = base_default_z;
          // state = 2;
          Serial.println("#ack=loading&send");
        } else {
          Serial.println("#ack=error");
        }

      } else if (header == 0x49 && Serial.available() >= 2) {
        // 原 0x49 Walking/Stop
        uint8_t generate = Serial.read();
        footer = Serial.read();
        if (footer == 0x45) {
          if (generate == 1) {
            state = 1;
            Serial.println("#ack=walking");
          } else {
            walking.ready_to_stop = true;
            Serial.println("#ack=stop");
          }
        } else {
          Serial.println("#ack=generate error");
        }

      } else if (header == 0x47 && Serial.available() >= sizeof(float)*3 + 1) {
        // 原 0x47 Change Walk
        float x, y, theta;
        Serial.readBytes((char*)&x,     sizeof(float));
        Serial.readBytes((char*)&y,     sizeof(float));
        Serial.readBytes((char*)&theta, sizeof(float));
        footer = Serial.read();
        if (footer == 0x45) {
          walking.step_length  = x / 1000.0;
          walking.shift_length = y / 1000.0;
          walking.theta_       = int(theta);
          // Serial.println("#ack=change walk");
          Serial.println(String(walking.step_length));
        } else {
          Serial.println("#ack=change walk error");
        }

      }if (header == 0xF2) {
        // 这会一直等到读够 PAYLOAD_BYTES 或超时（默认 1 s）
        size_t got = Serial.readBytes(position_velocity, PAYLOAD_BYTES);
        if (got == PAYLOAD_BYTES) {
          setupControl.motor_groupSyncWrite(position_velocity);
          setupControl.tranfer_bit(position_velocity);
          Serial.println("#motion_written");
        } else {
          Serial.println("#ack=payload_timeout");
        }
      }


    } 

    if (state == 3) {
      walking.ready_to_stop = true;
      state = 1;
    }

    osDelay(30);
  } // end outer while
}

// ================ 執行緒：每 30ms 更新一次步態控制 ================
static void Thread_Walking(void const *argument) {
  (void)argument;
  uint32_t timestamp = 0;
  uint32_t now_timestamp = 0;
  while (1) {
    if (state == 1 && walking.continuous_flag) {
      timestamp = osKernelSysTick();
      walking.continuous();
      setupControl.Inverse_kinematic(-walking.end_point_lx, -walking.end_point_ly, walking.end_point_lz, walking.end_point_ltheta, 0);
      if (walking.now_step % 2 == 1){
        setupControl.theta[1] -= walking.compensation_y_hip;
        setupControl.theta[5] += walking.compensation_y_ankle;
      }
      setupControl.rad2motor(0);
      setupControl.motor_speed(walking.sample_time, 0);
      setupControl.Inverse_kinematic(-walking.end_point_rx, -walking.end_point_ry, walking.end_point_rz, walking.end_point_rtheta, 6);
      if (walking.now_step % 2 == 0){
        setupControl.theta[7] += walking.compensation_y_hip;
        setupControl.theta[11] -= walking.compensation_y_ankle;
      }
      setupControl.rad2motor(6);
      setupControl.motor_speed(walking.sample_time, 6);
      setupControl.walking_groupSyncWrite(setupControl.profile_velocity, setupControl.motor_position);
      now_timestamp = osKernelSysTick();
    } else {
      state = 0;
      walking.initial();  // 停止時初始化
      osDelay(timestamp - now_timestamp + 30);
    }
    osDelay(timestamp - now_timestamp + 30);
  }
}
