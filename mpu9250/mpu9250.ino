#include "SparkFunMPU9250-DMP.h"        // 包含与MPU9250传感器通信的库的头文件
#include "I2Cdev.h"                    // 包含与I2C设备通信的库的头文件
#include "Kalman.h"                    // 包含实现卡尔曼滤波器的库的头文件

/*======================Global variable======================*/

// Debug mode
int PID_debug_mode = 1;                // 调试模式：1为开启，0为关闭
int RF_debug_mode = 0;                 // RF调试模式：1为开启，0为关闭

// MPU9250
MPU9250_DMP imu_9250;                  // MPU9250对象，用于与MPU9250陀螺仪传感器通信
float accelX, accelY, accelZ;          // 加速度计的X、Y、Z轴数据变量
float gyroX, gyroY, gyroZ;             // 陀螺仪的X、Y、Z轴数据变量
float roll, pitch;                     // 姿态角的Roll和Pitch值变量
float gyroXrate, gyroYrate;            // 陀螺仪的X和Y轴角速度值变量
float rad_to_reg = 180 / 3.141592654;  // 弧度转角度的比例因子

// Kalman Filter
Kalman kalmanX;                        // X轴数据的Kalman滤波器对象
Kalman kalmanY;                        // Y轴数据的Kalman滤波器对象
double gyroXangle, gyroYangle;          // 根据陀螺仪角速度计算得到的陀螺仪角度变量
double compAngleX, compAngleY;          // 根据陀螺仪和加速度计数据计算得到的补偿角度变量
double kalAngleX, kalAngleY;            // 经过Kalman滤波器滤波后的角度变量
double corrected_x, corrected_y;        // 校正偏移后的角度变量

// Motor
int dir1_L_PIN = 2;                    // 连接到左边电机的引脚1的引脚号
int dir2_L_PIN = 3;                    // 连接到左边电机的引脚2的引脚号
int speed_L_PIN = 9;                   // 连接到左边电机的速度控制引脚的引脚号
int dir1_R_PIN = 4;                    // 连接到右边电机的引脚1的引脚号
int dir2_R_PIN = 5;                    // 连接到右边电机的引脚2的引脚号
int speed_R_PIN = 10;                  // 连接到右边电机的速度控制引脚的引脚号
float MIN_SPEED = 25;                  // 电机的最小速度
float MAX_SPEED = 50;                  // 电机的最大速度

// PID
float kp = 22;                         // PID控制器的比例常数
float ki = 0.4;                        // PID控制器的积分常数
float kd = 20;                         // PID控制器的微分常数
float kp_error = 0.0;                  // 存储比例误差的变量
float ki_error = 0.0;                  // 存储积分误差的变量
float kd_error = 0.0;                  // 存储微分误差的变量
float kp_pass_error = 0.0;             // 存储通过误差项后的比例误差的变量
float kp_result = 0;                   // 存储比例项的结果的变量
float ki_result = 0;                   // 存储积分项的结果的变量
float kd_result = 0;                   // 存储微分项的结果的变量
float final_result = 0;                // 存储PID控制器的最终输出结果的变量

// Special angle
float overshoot_angle = 30;            // 在目标角度附近产生超调的角度
float PID_angle = 8;                   // 目标角度的PID控制器响应的阈值
float reference_angle = 0.0;           // 目标角度

// Joystick
int joy_x = A0;                        // 连接到摇杆的X轴的引脚
enum re_command {forward = 1, backward = 2, stay = 0}; // 运动命令枚举类型
float throttle = 50;                   // 电机的速度控制参数

// Timer
float now_time;                        // 当前时间
float pas_time;                        // 过去时间
float dif_time;                        // 时间差

/*======================support function======================*/
void UpdateIMUData(void)
{
  // 更新加速度计和陀螺仪数据
  accelX = imu_9250.calcAccel(imu_9250.ax);
  accelY = imu_9250.calcAccel(imu_9250.ay);
  accelZ = imu_9250.calcAccel(imu_9250.az);
  gyroX = imu_9250.calcGyro(imu_9250.gx);
  gyroY = imu_9250.calcGyro(imu_9250.gy);
  gyroZ = imu_9250.calcGyro(imu_9250.gz);

  // 将加速度计数据转换为角度
  roll = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * rad_to_reg;
  pitch = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * rad_to_reg;
  gyroXrate = gyroX / 131.0;
  gyroYrate = gyroY / 131.0;
}

void printIMUData(float control)
{
  // 打印姿态角、Kalman滤波器输出、控制信号和误差项
  Serial.println("Angle: " + String(roll) + " kalAngleY: " + String(kalAngleY) + " Control signal: " + String(control) + " kp_error: " + String(kp_error));
}

re_command check_receiver()
{
  // 检查接收机输入（摇杆）的值，运动命令
  int joy_x_value = analogRead(joy_x);
  re_command command = stay;

  if (joy_x_value >= 1000) {command = forward;}
  else if (joy_x_value <= 23) {command = backward;}

  if (RF_debug_mode) {
    Serial.println("PIN X: " + String(joy_x_value));
  }

  return command;
}

float pid_control() {
  // 计算当前误差
  kp_error = kalAngleY - reference_angle;

  // 快速调整车辆
  if (kp_error >= overshoot_angle && kp_error <= -overshoot_angle) {
    kp = 40;
  } else {
    kp = 22;
  }

  // 计算积分项和微分项
  ki_error += kp_error * dif_time;
  kd_error = (kp_error - kp_pass_error) / dif_time;

  // 计算结果
  kp_result = kp_error * kp;
  ki_result = ki_error * ki;
  kd_result = kd_error * kd;
  kp_pass_error = kp_error;
  final_result = kp_result + kd_result;

  // 当角度较小时，才使用PID控制
  if (kp_error <= PID_angle && kp_error >= -PID_angle) {
    final_result = kp_result + kd_result + ki_result;
  }

  return final_result;
}

void kalman() {
  // 校正姿态角度，避免旋转角度突变
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dif_time); // 使用卡尔曼滤波器计算角度
  }

  // 当角度超过90度时，反转陀螺仪数据
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate;
  }

  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dif_time); // 使用卡尔曼滤波器计算角度
  gyroXangle += gyroXrate * dif_time; // 计算陀螺仪角度（未经过滤）
  gyroYangle += gyroYrate * dif_time;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dif_time) + 0.07 * roll; // 使用互补滤波器计算角度
  compAngleY = 0.93 * (compAngleY + gyroYrate * dif_time) + 0.07 * pitch;

  // 当陀螺仪角度漂移过大时，重新校正
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalAngleY;
  }
}
/*======================setup======================*/
void setup() {
  Serial.begin(9600);

  // 初始化MPU-9250
  if (imu_9250.begin() != INV_SUCCESS)
  {
    imu_9250.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);// Enable all sensors:
    imu_9250.setGyroFSR(2000); // Set gyro to 2000 dps
    imu_9250.setAccelFSR(2); // Set accel to +/-2g
    imu_9250.setLPF(5); // Set LPF corner frequency to 5Hz
    imu_9250.setSampleRate(10); // Set sample rate to 10Hz
  }
/*设置L298N引脚*/
  // 左侧电机引脚
  pinMode(dir1_L_PIN, OUTPUT);
  pinMode(dir2_L_PIN, OUTPUT);
  pinMode(speed_L_PIN, OUTPUT);
  // 右侧电机引脚
  pinMode(dir1_R_PIN, OUTPUT);
  pinMode(dir2_R_PIN, OUTPUT);
  pinMode(speed_R_PIN, OUTPUT);

  // 设置JoyStick引脚
  pinMode(joy_x, INPUT);

  // 初始化计时器
  pas_time = millis();
}
/**************************左右转指定角度****************/
void leftTurn(float target_angle) {
  float current_angle = 0;

  while (current_angle > -target_angle) {
    kalman(); // 更新角度数据
    float control_signal = pid_control(); // 计算PID控制信号

    // 设置电机转向和速度
    digitalWrite(dir1_L_PIN, HIGH);
    digitalWrite(dir2_L_PIN, LOW);
    digitalWrite(dir1_R_PIN, LOW);
    digitalWrite(dir2_R_PIN, HIGH);
    analogWrite(speed_L_PIN, control_signal);
    analogWrite(speed_R_PIN, control_signal);

    delay(10); // 控制循环周期
    current_angle = kalAngleY; // 更新当前角度

    // 打印角度信息
    Serial.println("Current Angle: " + String(current_angle));
  }

  // 停止电机
  digitalWrite(speed_L_PIN, LOW);
  digitalWrite(speed_R_PIN, LOW);
}

void rightTurn(float target_angle) {
  float current_angle = 0;

  while (current_angle < target_angle) {
    kalman(); // 更新角度数据
    float control_signal = pid_control(); // 计算PID控制信号

    // 设置电机转向和速度
    digitalWrite(dir1_L_PIN, LOW);
    digitalWrite(dir2_L_PIN, HIGH);
    digitalWrite(dir1_R_PIN, HIGH);
    digitalWrite(dir2_R_PIN, LOW);
    analogWrite(speed_L_PIN, control_signal);
    analogWrite(speed_R_PIN, control_signal);

    delay(10); // 控制循环周期
    current_angle = kalAngleY; // 更新当前角度

    // 打印角度信息
    Serial.println("Current Angle: " + String(current_angle));
  }

  // 停止电机
  digitalWrite(speed_L_PIN, LOW);
  digitalWrite(speed_R_PIN, LOW);
}
/*======================main loop======================*/
//void loop() {
//  // 控制相关变量
//  int moving_flag = 0; // 移动标志位
//  float control_signal = 0; // 控制信号
//
//  // 计算时间差
//  now_time = millis();
//  dif_time = (now_time - pas_time) / 1000; // 单位为秒
//  pas_time = now_time;
//
//  // 更新IMU数据
//  if (imu_9250.dataReady()) {
//    imu_9250.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
//    UpdateIMUData();
//    kalman();
//  }
//
//  // PID
//  control_signal = pid_control();
//
//  // 限制控制信号的范围
//  if (control_signal >= 0) {
//    control_signal = constrain(control_signal, MIN_SPEED, MAX_SPEED);
//  } else {
//    control_signal = constrain(control_signal, -MAX_SPEED, -MIN_SPEED);
//  }
//
//  // 检查接收到的消息
//  re_command command = check_receiver();
//  if (command == forward) {
//    control_signal = throttle;
//  } else if (command == backward) {
//    control_signal = -throttle;
//  }
//  
//  // 将信号应用于电机
//  if (control_signal < 0) {
//    analogWrite(speed_L_PIN, abs(control_signal));
//    analogWrite(speed_R_PIN, abs(control_signal));
//    digitalWrite(dir1_L_PIN, HIGH);
//    digitalWrite(dir2_L_PIN, LOW);
//    digitalWrite(dir1_R_PIN, HIGH);
//    digitalWrite(dir2_R_PIN, LOW);
//  } else {
//    analogWrite(speed_L_PIN, control_signal);
//    analogWrite(speed_R_PIN, control_signal);
//    digitalWrite(dir1_L_PIN, LOW);
//    digitalWrite(dir2_L_PIN, HIGH);
//    digitalWrite(dir1_R_PIN, LOW);
//    digitalWrite(dir2_R_PIN, HIGH);
//  }
//
//  // 打印调试信息
//  if (PID_debug_mode) {
//    printIMUData(control_signal);
//  }
//}
void loop() {
  // 左转0度
  leftTurn(90);
  delay(1000); // 延迟1秒
  Serial.println("===============左转结束==============");
  // 右转90度
  rightTurn(90);
  delay(1000); // 延迟1秒
  Serial.println("===============右转结束==============");
}
