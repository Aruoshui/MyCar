

#include <GP2Y0.h> //红外测距，但是没用到
#include <Arduino.h> //默认包
#include <string.h>  //字符串处理头文件
#include <SoftwareSerial.h> //通讯包



SoftwareSerial BT(9, 8);  //定义蓝牙串口的引脚，蓝牙只需接4根线，Vcc-5V GND-GND TXD-Pin9 RXD-Pin8

/***************************start摄像头start****************************/
/*
  测试日志
*/
/*
  bug:(未解决)
  bug:(已解决)


*/
#include <DFRobot_HuskyLens.h> //二哈摄像头
#include <HUSKYLENS.h>
#include <HUSKYLENSMindPlus.h>
#include <HuskyLensProtocolCore.h>
HUSKYLENS huskylens;
void printResult(HUSKYLENSResult result);
//二哈连线方法:green line >> SDA; blue line >> SCL

/***************************end摄像头end****************************/



/***************************start电机（基础控制）start****************************/
//此处的控制都是依靠驱动板，详情就看文档里边


const unsigned long interval = 300;  // 测量间隔（毫秒）
unsigned long previousMillis = 0;


boolean turnleft = true;
int qhlength;
unsigned long rx_time = 0;
unsigned char cmd[8] = { 0x19, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11 };

const int DSD = 10;  //Default Servo Delay (默认电机运动延迟时间)
static int i = 0;

void youpian(int zuo, int you)  //700 300
{
  setcmd1('m', -zuo);
  setcmd1('M', zuo);
  setcmd2('m', you);
  setcmd2('M', -you);
}
void zuopian(int zuo, int you)  //300 700
{
  setcmd1('m', -zuo);
  setcmd1('M', zuo);
  setcmd2('m', you);
  setcmd2('M', -you);
}

void forward(int qian)  //600
{
  setcmd1('m', -qian);
  setcmd1('M', qian);
  setcmd2('m', qian);
  setcmd2('M', -qian);
}
void back(int hou)  //600
{
  setcmd1('m', hou);
  setcmd1('M', -hou);
  setcmd2('m', -hou);
  setcmd2('M', hou);
}
void stop() {
  setcmd1('m', 0);
  setcmd1('M', 0);
  setcmd2('m', 0);
  setcmd2('M', 0);
}
void left(int zuo)  //500
{
  setcmd1('m', zuo);
  setcmd1('M', -zuo);
  setcmd2('m', zuo);
  setcmd2('M', -zuo);
}

void right1(int you)  //500
{
  setcmd1('m', -you - 300);
  setcmd1('M', you + 300);
  setcmd2('m', -you);
  setcmd2('M', you);
}
void right(int you)  //500
{
  setcmd1('m', -you);
  setcmd1('M', you);
  setcmd2('m', -you);
  setcmd2('M', you);
}

void setcmd1(unsigned char Cmd_Send, long MotorSpeed) {
  cmd[2] = Cmd_Send;
  cmd[3] = (unsigned char)(MotorSpeed >> 24);
  cmd[4] = (unsigned char)(MotorSpeed >> 16);
  cmd[5] = (unsigned char)(MotorSpeed >> 8);
  cmd[6] = (unsigned char)(MotorSpeed);
  Serial2.write(cmd, 8);
  return;
}

void setcmd2(unsigned char Cmd_Send, long MotorSpeed) {
  cmd[2] = Cmd_Send;
  cmd[3] = (unsigned char)(MotorSpeed >> 24);
  cmd[4] = (unsigned char)(MotorSpeed >> 16);
  cmd[5] = (unsigned char)(MotorSpeed >> 8);
  cmd[6] = (unsigned char)(MotorSpeed);
  Serial3.write(cmd, 8);
  return;
}
/***************************end电机（基础控制）end****************************/
//
//
//
//
//
/***************************start机械臂start****************************/
/*
  测试日志
*/
/*
  bug:(未解决)
  bug:(已解决)


*/

#include <SPI.h>
#include <string.h>
#include <Pixy2I2C.h>
#include <Servo.h> //舵机控制相关

bool leave = false;
Pixy2I2C pixy;


Servo base;
Servo dabi;
Servo xiaobi;
Servo claw;
//4个舵机初始角度0;
const float dabiFromPos = 110;   //初始110
const float xiaobiFromPos = 90;  //初始90
const float baseFromPos = 85;    //初始90
const float clawFromPos = 180;   //初始220
//3个舵机目标角度
const float dabiGetPos = 5;     //目标5
const float xiaobiGetPos = 30;  //目标30
const float clawGetPos = 120;   //目150
float pos = 1;
float MyPos = 1;
bool flag = false;             //抓取放下逻辑判断标志
bool flag_catch_drop = false;  //抓取放下逻辑判断标志
bool flag_arrive = true;       //摄像头判断是否到达位置标志

int j;
void leftCatch() {
  //抓取
  Serial.println("抓取");
  gotoPosition(xiaobiGetPos, xiaobi);
  delay(100);
  gotoPosition(clawGetPos, claw);  //打开
  delay(100);
  gotoPosition(170, base);
  delay(100);
  gotoPosition(dabiGetPos, dabi);
  delay(200);
  //返回
  Serial.println("返回");
  gotoPosition(clawFromPos, claw);  //抓紧
  delay(100);
  gotoPosition(dabiFromPos, dabi);
  delay(100);
  gotoPosition(baseFromPos, base);
  delay(100);
  gotoPosition(xiaobiFromPos, xiaobi);
  leave = !leave;
}
void leftDrop() {
  //抓取
  Serial.println("leftDrop");
  Serial.println("抓取");
  gotoPosition(xiaobiGetPos, xiaobi);
  delay(100);
  gotoPosition(clawFromPos, claw);
  delay(100);
  gotoPosition(170, base);
  delay(100);
  gotoPosition(dabiGetPos, dabi);
  //返回
  delay(200);
  Serial.println("返回");
  gotoPosition(clawGetPos, claw);
  delay(100);
  gotoPosition(dabiFromPos, dabi);
  delay(100);
  gotoPosition(clawFromPos, claw);
  delay(100);
  gotoPosition(baseFromPos, base);
  delay(100);
  gotoPosition(xiaobiFromPos, xiaobi);
}
void rightCatch() {
  Serial.println("rightCatch");
  Serial.println("抓取");
  gotoPosition(xiaobiGetPos, xiaobi);
  delay(100);
  gotoPosition(clawGetPos, claw);
  delay(100);
  gotoPosition(0, base);
  delay(100);
  gotoPosition(dabiGetPos, dabi);
  //返回
  Serial.println("返回");
  gotoPosition(clawFromPos, claw);
  delay(100);
  gotoPosition(dabiFromPos, dabi);
  delay(100);
  gotoPosition(baseFromPos, base);
  delay(100);
  gotoPosition(xiaobiFromPos, xiaobi);
}
void rightDrop() {
  //抓取
  Serial.println("righrDrop");
  Serial.println("抓取");
  gotoPosition(xiaobiGetPos, xiaobi);
  delay(100);
  gotoPosition(clawFromPos, claw);
  delay(100);
  gotoPosition(0, base);
  delay(100);
  gotoPosition(dabiGetPos, dabi);
  //返回
  delay(1000);
  Serial.println("返回");
  gotoPosition(clawGetPos, claw);
  delay(100);
  gotoPosition(dabiFromPos, dabi);
  delay(100);
  gotoPosition(clawFromPos, claw);
  delay(100);
  gotoPosition(baseFromPos, base);
  delay(100);
  gotoPosition(xiaobiFromPos, xiaobi);
}
void gotoPosition(float toPos, Servo servo2go) {
  Serial.println("调用了gotoPosition函数");
  float i = servo2go.read();
  if (i <= toPos) {
    do {
      servo2go.write(i);
      i++;
      delay(DSD);
    } while (i <= toPos);
  } else {
    do {
      servo2go.write(i);
      i--;
      delay(DSD);
    } while (i >= toPos);
  }
}
void pixyy(bool flag_arrive, float poss) {

  if (flag_arrive && (poss >= 260 && poss <= 225)) {
    //小车停止
    stop();


    //else小车微调整位置
  } else if (poss < 225) {
    forward(500);
  } else if (poss > 260) {
    back(500);
  }
}
void arm() {
  //机械臂
  //后期armmm应该传参数，来判断是哪一个摄像头的数据
  //目前只测试左边的，故设置pos为1
  pos = 0;
  flag_catch_drop = !flag_catch_drop;
  position(pos, flag_catch_drop);
  //position函数完成，摄像头眼中已无小球
  if (flag_catch_drop) {
    flag_arrive = true;
  } else {
    flag_arrive = false;
  }
  //车子可以移动
}
void position(float pos, bool flag) {
  if (pos == 1) {  //此时向左
    if (flag == true) {
      leftCatch();
    } else {
      leftDrop();
    }
  } else {  //此时向右
    if (flag == true) {
      rightCatch();
    } else {
      rightDrop();
    }
  }
}
void hand() {
  while (true) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
      Serial.println(pixy.ccc.numBlocks);
      Serial.println(pixy.ccc.blocks[j].m_x);
      float poss = pixy.ccc.blocks[j].m_x;
      pixyy(flag_arrive, poss);
      if (flag_arrive && (poss >= 225 && poss <= 260)) {
        stop();
        flag_arrive = false;
        arm();
        break;
      }
    } else {
      forward(500);
    }
  }
}

/***************************end机械臂end****************************/
//
//
//
//
//
/***************************start陀螺仪（角度）start****************************/
/*
  测试日志
*/
/*
  bug:(未解决)
  bug:(已解决)

*/
#include "Wire.h"

const int addr = 0x68;

uint16_t cal_int;
int16_t temp;
int acc_axis[4];
int gyro_axis[4];

float angle_pitch;
float angle_pitch_acc;
long acc_x;
long acc_y;
long acc_z;
long acc_total_vector[1];

double gyro_axis_cal[4];
double gyro_pitch;
double gyro_yaw;

float angle_Kp = 0.5;
float angle_Ki = -0.001;
float angle_Kd = 0;

float angle_prev_error = 0;
float angle_integral = 0;
unsigned long angle_prev_time = 0;

float init_speed = 500;  //初始速度为500
float speed1 = 0;
float speed2 = 0;

//下面这些注释是设计使用PID控制转角速度的

// /*先通过目标航向角和当前航向角推算出需要多少车轮差速值；再用基础速度加减输出速度;负数是左转、正数是右转*/
// void angleByPID(int speed, int target) {
//   unsigned long current_time = millis();
//   float dt = (current_time - angle_prev_time) / 1000.0;
//   float current_angle = getPitchAngle();  // 获取当前角度
//   float error = target - current_angle;
//   // 将角度误差控制在 [-180, 180] 范围内
//   if (error > 180.0) {
//     error -= 360.0;
//   } else if (error < -180.0) {
//     error += 360.0;
//   }
//   float output = angle_Kp * error + angle_Ki * angle_integral + angle_Kd * (error - angle_prev_error) / dt;
//   angle_prev_error = error;
//   angle_integral += error * dt;
//
//   /* 对积分项进行限制 */
//   float integral_limit = 100.0;
//   if (angle_integral > integral_limit) {
//     angle_integral = integral_limit;
//   } else if (angle_integral < -integral_limit) {
//     angle_integral = -integral_limit;
//   }
//   int constrained_output = constrain(output, -speed, speed);
//   speed1 = init_speed + constrained_output;
//   speed2 = init_speed - constrained_output;
//   current_angle = getPitchAngle();
//   if (current_angle > target) {
//     do {
//       zuopian(speed2, speed1);          // 前边数值小于后边
//       current_angle = getPitchAngle();  // 获取当前角度
//     } while (current_angle > target);
//   } else if (current_angle < target) {
//     do {
//       youpian(speed1, speed2);                          // 前边数值大于后边
//       current_angle = current_angle = getPitchAngle();  // 获取当前角度
//     } while (current_angle < target);
//   }
//   resetAnglePID();
// }
//
///*先通过目标航向角和当前航向角推算出需要多少车轮差速值；再用基础速度加减输出速度;负数是左转、正数是右转*/
// void angleByPID(int speed, int target) {
//   Serial.println("");
//   unsigned long current_time = millis();
//   float dt = (current_time - angle_prev_time) / 1000.0;
//   float current_angle = getPitchAngle();  // 获取当前角度
//   float error = target - current_angle;
//   // 将角度误差控制在 [-180, 180] 范围内
//   if (error > 180.0) {
//     error -= 360.0;
//   } else if (error < -180.0) {
//     error += 360.0;
//   }
//   float output = angle_Kp * error + angle_Ki * angle_integral + angle_Kd * (error - angle_prev_error) / dt;
//   angle_prev_error = error;
//   angle_integral += error * dt;
//
//   /* 对积分项进行限制 */
//   float integral_limit = 100.0;
//   if (angle_integral > integral_limit) {
//     angle_integral = integral_limit;
//   } else if (angle_integral < -integral_limit) {
//     angle_integral = -integral_limit;
//   }
//   int constrained_output = constrain(output, -speed, speed);
//   speed1 = init_speed + constrained_output;
//   speed2 = init_speed - constrained_output;
//   current_angle = getPitchAngle();
//   Serial.print("当前角度");
//   Serial.println(current_angle);
//   if (current_angle > target) {
//     Serial.println("当前角度大于初始角度，往左偏调整");
//     do {
//       zuopian(speed2, speed1);          // 前边数值小于后边
//       current_angle = getPitchAngle();  // 获取当前角度
//       Serial.print("当前角度");
//       Serial.println(current_angle);
//     } while (current_angle > target);
//     Serial.println("当前角度与初始角度一致，结束调整");
//   } else if (current_angle < target) {
//     Serial.println("当前角度小于初始角度，往右偏调整");
//     do {
//       youpian(speed1, speed2);          // 前边数值大于后边
//       current_angle = getPitchAngle();  // 获取当前角度
//       Serial.print("当前角度");
//       Serial.println(current_angle);
//     } while (current_angle < target);
//     Serial.println("当前角度与初始角度一致，结束调整");
//   } else if (current_angle < target) {
//     Serial.println("当前角度与初始角度一致，不做转向调整");
//   }
//   resetAnglePID();
// }
//
// /*重置控制器状态*/
// void resetAnglePID() {
//   angle_prev_error = 0;
//   angle_integral = 0;
//   angle_prev_time = millis();
// }

  
        /***************************MPU9250传感器相关，其实就是度角度，我们这里是github上学习的姿态解算的源代码，没有库，但是原理需要知道****************************/

void callibrate() {
  Serial.println("Callibrating");
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    if (cal_int % 50 == 0)
      digitalWrite(13, !digitalRead(13));
    receive();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];
    delay(3);
  }
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
  gyro_axis_cal[3] /= 2000;
  Serial.println("Gyro Calibrated");
  delay(1000);
}

void init_gyro() {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  callibrate();
}

void receive() {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(addr, 14, true);
  while (Wire.available() < 14)
    ;
  acc_axis[1] = Wire.read() << 8 | Wire.read();
  acc_axis[2] = Wire.read() << 8 | Wire.read();
  acc_axis[3] = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_axis[1] = Wire.read() << 8 | Wire.read();
  gyro_axis[2] = Wire.read() << 8 | Wire.read();
  gyro_axis[3] = Wire.read() << 8 | Wire.read();
  if (cal_int == 2000)
    gyro_axis[1] -= gyro_axis_cal[1];
  gyro_axis[2] -= gyro_axis_cal[2];
  gyro_axis[3] -= gyro_axis_cal[3];
}

void filter() {
  angle_pitch += gyro_axis[1] * 0.0000611;

  acc_total_vector[0] = sqrt((acc_axis[1] * acc_axis[1]) + (acc_axis[2] * acc_axis[2]) + (acc_axis[3] * acc_axis[3]));

  angle_pitch_acc = asin((float)acc_axis[2] / acc_total_vector[0] * 57.296);
}

float getPitchAngle() {
  receive();
  filter();
  return angle_pitch;  // 返回角度值
}
          /***************************MPU9250传感器相关****************************/

/*左转90度*/
void leftWithYaw(int target_yaw) {
  float init_yaw = getPitchAngle();
  Serial.print("init_yaw:");
  Serial.println(init_yaw);
  //  float target_yaw = 17.5;//需要多次测量得出初始位置和转正的角度恒定差值
  float current_yaw;
  do {
    left(500);
    current_yaw = getPitchAngle();
    Serial.print("current_yaw:");
    Serial.println(current_yaw);
  } while (abs(current_yaw - init_yaw) < target_yaw);
}
/*右转90度*/
void rightWithYaw(int target_yaw) {
  float init_yaw = getPitchAngle();
  Serial.print("init_yaw:");
  Serial.println(init_yaw);
  //  float target_yaw = 18.3;//需要多次测量得出初始位置和转正的角度恒定差值
  float current_yaw;
  do {
    right(500);
    current_yaw = getPitchAngle();
    Serial.print("current_yaw:");
    Serial.println(current_yaw);
  } while (abs(current_yaw - init_yaw) < target_yaw);
}
void rightWithYaw1(int target_yaw) {
  float init_yaw = getPitchAngle();
  Serial.print("init_yaw:");
  Serial.println(init_yaw);
  //  float target_yaw = 18.3;//需要多次测量得出初始位置和转正的角度恒定差值
  float current_yaw;
  do {
    right1(500);
    current_yaw = getPitchAngle();
    Serial.print("current_yaw:");
    Serial.println(current_yaw);
  } while (abs(current_yaw - init_yaw) < target_yaw);
}

        /***************************通过角度对左转右转控制，增快速度，消除惯性影响****************************/
// /*左转90度*/
// void leftWithYawPID() {
//   float init_yaw = getPitchAngle();
//   Serial.print("init_yaw:");
//   Serial.println(init_yaw);
//   float differ = 17.5;  //需要多次测量得出初始位置和转正的角度恒定差值
//   float current_yaw;
//   float speed;
//   do {
//     speed = angleByPID(init_yaw);
//     Serial.print("angleByPID返回speed:");
//     Serial.println(speed);
//     resetAnglePID();  // 重置角度控制器的状态
//     left(speed);
//     current_yaw = getPitchAngle();
//     Serial.print("current_yaw:");
//     Serial.println(current_yaw);
//   } while (abs(current_yaw - init_yaw) < differ);
// }
// /*右转90度*/
// void rightWithYawPID() {
//   float init_yaw = getPitchAngle();
//   Serial.print("init_yaw:");
//   Serial.println(init_yaw);
//   float differ = 18.3;  //需要多次测量得出初始位置和转正的角度恒定差值
//   float current_yaw;
//   float speed;
//   do {
//     speed = angleByPID(init_yaw);
//     Serial.print("angleByPID返回speed:");
//     Serial.println(speed);
//     resetAnglePID();  // 重置角度控制器的状态
//     right(speed);
//     current_yaw = getPitchAngle();
//     Serial.print("current_yaw:");
//     Serial.println(current_yaw);
//   } while (abs(current_yaw - init_yaw) < differ);
// }
          /***************************通过角度对左转右转控制，增快速度，消除惯性影响****************************/

/***************************end陀螺仪（角度控制）end****************************/
//
//
//
//
//
/***************************start超声波（距离处理）start****************************/
/*
  测试日志
*/
/*
  bug:(未解决)
  bug:(已解决)


*/

#include "SR04.h"    //超声波测距头文件

#define TRIG_PIN_1 18  // 1号超声波18引脚为输入
#define ECHO_PIN_1 19  // 1号超声波19引脚为控制

#define TRIG_PIN_2 24  // 2号超声波24引脚为输入
#define ECHO_PIN_2 25  // 2号超声波25引脚为控制

#define TRIG_PIN_3 22  // 3号超声波22引脚为输入
#define ECHO_PIN_3 23  // 3号超声波23引脚为控制

#define TRIG_PIN_4 53  // 4号超声波40引脚为输入
#define ECHO_PIN_4 52  // 4号超声波39引脚为控制

#define TRIG_PIN_a 28  // a号超声波28引脚为输入
#define ECHO_PIN_a 29  // a号超声波29引脚为控制

#define TRIG_PIN_b 38  // b号超声波37引脚为输入
#define ECHO_PIN_b 37  // b号超声波38引脚为控制

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long pulseDuration = pulseIn(echoPin, HIGH, 30000);
  float distance = pulseDuration * 0.0343 / 2;

  return distance;
}
//卡曼尔滤波算法,测距更加稳定
float klm(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long pulseDuration = pulseIn(echoPin, HIGH, 30000);  // 加时
  float distance = pulseDuration * 0.0343 / 2;                  // 计算距离

  float P = 1;
  float X = 0;
  float Q = 0.01;  // 噪声
  float R = 0.2;   // R如果很大，更相信预测值，那么传感器反应就会迟钝，反之相反

  float X_ = X + 0;
  float P_ = P + Q;
  float K = P_ / (P_ + R);

  X = X_ + K * (distance - X_);
  P = P_ - K * P_;
  return X;
}
//教材上使用的滤波
float calculateDistance(int distype) {
  float distances[10];
  switch (distype) {
    case 1:
      for (int i = 0; i < 10; i++) {
        distances[i] = klm(TRIG_PIN_1, ECHO_PIN_1);
      }
      break;
    case 2:
      for (int i = 0; i < 10; i++) {
        distances[i] = klm(TRIG_PIN_2, ECHO_PIN_2);
      }
      break;
    case 3:
      for (int i = 0; i < 10; i++) {
        distances[i] = klm(TRIG_PIN_3, ECHO_PIN_3);
      }
      break;
    case 4:
      for (int i = 0; i < 10; i++) {
        distances[i] = klm(TRIG_PIN_4, ECHO_PIN_4);
      }
      break;
    case 5:
      for (int i = 0; i < 10; i++) {
        distances[i] = klm(TRIG_PIN_a, ECHO_PIN_a);
      }
      break;
    case 6:
      for (int i = 0; i < 10; i++) {
        distances[i] = klm(TRIG_PIN_b, ECHO_PIN_b);
      }
      break;
    default:
      BT.println("InvalidType!");
      return -2;
  }

  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += distances[i];
  }
  float average = sum / 10.0;
  float standardDeviation = 0;
  for (int i = 0; i < 10; i++) {
    standardDeviation += (distances[i] - average) * (distances[i] - average);
  }
  standardDeviation = sqrt(standardDeviation / 10);  // 修正标准差计算，应该除以10.0
  float midDistance = distances[4];
  float residualError = abs(midDistance - average);
  if (residualError <= 3 * standardDeviation) {
    return midDistance;
  } else {
    return average;
  }
}

/***************************end超声波（距离处理）end****************************/



/***************************start距离PID控制（距离处理）start****************************/

float distance_Kp = 200;
float distance_Ki = 0.5;
float distance_Kd = 0.1;
float distance_prev_error = 0;
float distance_integral = 0;
unsigned long distance_prev_time = 0;

int forward_speed = 0;
int back_speed = 0;
int left_speed = 0;
int right_speed = 0;

void usePID(int speed, int target, int movetype) {
  unsigned long current_time = millis();                    //当前时间毫秒数
  float dt = (current_time - distance_prev_time) / 1000.0;  //计算时间差分，单位为秒
  float distance_3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
  float distance_4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
  //  float distance_3 = calculateDistance(3);
  //  float distance_4 = calculateDistance(4);
  float error = abs((distance_3 + distance_4) / 2 - target);  //计算误差值
  BT.print("error: ");
  BT.println(error);
  float output = distance_Kp * error + distance_Ki * distance_integral + distance_Kd * (error - distance_prev_error) / dt;
  distance_prev_error = error;        //更新以便下次循环
  distance_integral += (error * dt);  //积分项累加
  /*对积分项进行限制*/
  float integral_limit = 100.0;  //设置积分项上下限
  if (distance_integral > integral_limit) {
    distance_integral = integral_limit;
  } else if (distance_integral < -integral_limit) {
    distance_integral = -integral_limit;
  }
  int constrained_output = constrain(output, -speed, speed);
  switch (movetype) {
    case 1:  //前进
      forward(constrained_output);
      forward_speed = speed;
      break;
    case 2:  //后退
      back(constrained_output);
      back_speed = speed;
      break;
    default:
      BT.println("Invalid type!!!");
      break;
  }
}

/*重置控制器状态*/
void resetPID() {
  distance_prev_error = 0;
  distance_integral = 0;
  distance_prev_time = millis();
}

void forwardPID(int speed, int target) {  //前进至目标距离（3和4）停下
  float distance_3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
  float distance_4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
  BT.print("distance3:");
  BT.println(distance_3);
  BT.print("distance4:");
  BT.println(distance_4);
  if (distance_3 < target && distance_4 < target) {
    stop();  // 停止车辆
    BT.print("距离前方：");
    BT.println(target);
    BT.println("停车");
  } else {
    do {
      usePID(speed, target, 1);
      distance_3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
      distance_4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
      BT.print("distance3:");
      BT.println(distance_3);
      BT.print("distance4:");
      BT.println(distance_4);
    } while (distance_3 > target || distance_4 > target);
  }
  stop();
  resetPID();  // 重置PID控制器状态
}

void backPID(int speed, int target) {  //后退至目标距离（3和4）停下
  float distance_3 = calculateDistance(3);
  float distance_4 = calculateDistance(4);
  BT.print("distance3:");
  BT.println(distance_3);
  BT.print("distance4:");
  BT.println(distance_4);
  if (distance_3 > target && distance_4 > target) {
    stop();  // 停止车辆
    BT.print("距离前方：");
    BT.println(target);
    BT.println("停车");
  } else {
    do {
      usePID(speed, target, 2);
      distance_3 = calculateDistance(3);
      distance_4 = calculateDistance(4);
      BT.print("distance3:");
      BT.println(distance_3);
      BT.print("distance4:");
      BT.println(distance_4);
    } while (distance_3 < target || distance_4 < target);
  }
  stop();
  resetPID();  // 重置PID控制器状态
}


// float angle_Kp = 20;         // 比例系数
// float angle_Ki = 0.2;        // 积分系数
// float angle_Kd = 0.1;        // 微分系数
// float angle_prev_error = 0;  // 上次误差
// float angle_integral = 0;    // 积分项
// unsigned long angle_prev_time = 0;

// float angleByPID(float init_yaw) {
//   unsigned long current_time = millis();
//   float dt = (current_time - angle_prev_time) / 1000.0;
//   float current_yaw = getPitchAngle();
//   float error = abs(current_yaw - init_yaw);
//   float output = angle_Kp * error + angle_Ki * angle_integral + angle_Kd * (error - angle_prev_error) / dt;
//   angle_prev_error = error;
//   angle_integral += (error * dt);

//   // 对积分项进行限制
//   float integral_limit = 100.0;
//   if (angle_integral > integral_limit) {
//     angle_integral = integral_limit;
//   } else if (angle_integral < -integral_limit) {
//     angle_integral = -integral_limit;
//   }

//   int constrained_output = constrain(output, -1000, 1000);

//   angle_prev_time = current_time;  // 更新时间

//   // 返回更准确的差值
//   return constrained_output;
// }

// /*重置控制器状态*/
// void resetAnglePID() {
//   angle_prev_error = 0;
//   angle_integral = 0;
//   angle_prev_time = millis();
// } 

/***************************end距离PID控制（距离处理）end****************************/
//
//
//
//
//
/***************************start各部分初始化start****************************/
void setup() {
  Serial.begin(9600);  //与电脑通信波特率

   //BT.begin(9600);
  Serial.println("Welcome!");
  Serial2.begin(115200);  //驱动板初始化
  Serial3.begin(115200);
  Serial2.write(0x55);
  delay(10);
  Serial3.write(0x55);
  delay(10);

      /***************************MPU9250传感器的初始化****************************/
  Wire.begin();
  TWBR = 12;  // Set I2C frequency to 400kHz
  init_gyro();
  delay(20);
      /***************************MPU9250传感器的初始化****************************/

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);

  pinMode(TRIG_PIN_a, OUTPUT);
  pinMode(ECHO_PIN_a, INPUT);

  pinMode(TRIG_PIN_b, OUTPUT);
  pinMode(ECHO_PIN_b, INPUT);

  pixy.init(); //pixy摄像头

      /***************************机械臂初始化****************************/
  base.attach(4);
  dabi.attach(5);
  xiaobi.attach(6);
  claw.attach(7);
  base.write(baseFromPos);
  dabi.write(dabiFromPos);
  xiaobi.write(xiaobiFromPos);
  claw.write(clawFromPos);  //抓紧
      /***************************机械臂初始化****************************/

      /***************************二哈摄像头初始化****************************/
  Wire.begin();
  huskylens.begin(Wire);
  huskylens.request();
  huskylens.isLearned();
  huskylens.available();
  Serial.println("Begin success!");
      /***************************二哈摄像头初始化****************************/
}
/***************************end各部件初始化end****************************/
//
//
//
//
//
/***************************start小车路径规划&运动逻辑start****************************/
/*
  测试日志
*/
/*
  bug:(未解决)
  bug:(已解决)


*/

void loop() {
  float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
  delay(10);
  float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
  delay(10);
  float distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
  delay(10);
  float distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
  delay(10);
  float distance_a = measureDistance(TRIG_PIN_a, ECHO_PIN_a);
  delay(10);
  float distance_b = measureDistance(TRIG_PIN_b, ECHO_PIN_b);
  delay(10);
  /*待会一定要细说！！！！！*/
  float finish = 25;
  if (distance_b > 20) {
    finish = 8;
  } else {
    finish = 25;
  }




  forwardPID(3000, 21);
  stop();
  delay(100);
  leftWithYaw(17.99);
  /*开始在桥面上行驶*/
  int count = 0;
  float target = getPitchAngle();
  //float current;
  Serial.print("上桥初始角度(目标角度)");
  Serial.println(target);
  do {
    Serial.println("已经在桥面上");
    forward(2500);
    distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
    distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
    BT.print("distance1:");
    BT.println(distance1);
    BT.print("distance2:");
    BT.println(distance2);
    BT.print("distance3:");
    BT.println(distance3);
    BT.print("distance4:");
    BT.println(distance4);
    //  偏向调整
      //  angleByPID(500, target);
  } while ((distance3 > 7 && distance4 > 7) || (distance3 == 0 || distance4 == 0));  //排除：还在桥上 但是前方检测距离小于8
                                                                                     //  forwardPID(1500, 8);
                                                                                     //  rightWithYaw(17.8);
                                                                                     //  stop();
                                                                                     //  delay(100);

  /*右转对准放置台*/

  stop();
  delay(100);
  leftWithYaw(17.95);
  stop();
  delay(100);
  forwardPID(3000, 7);
  stop();
  delay(100);
  leftWithYaw(17.95);
  stop();
  delay(100);
  hand();





  /*-------------------------过障碍门，预期来到告示牌前-------------------*/

  do {
    forward(2000);
    distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
    BT.print("distance1:");
    BT.println(distance1);
    BT.print("distance2:");
    BT.println(distance2);
  } while (distance1 < 80 && distance2 < 85);

  delay(10);
  rightWithYaw(18.15);
  BT.println("预期左转完毕");
  distance_a = measureDistance(TRIG_PIN_a, ECHO_PIN_a);
  if (distance_a < 40) {
    BT.println("后退");
    do {
      back(1500);
      distance_a = measureDistance(TRIG_PIN_a, ECHO_PIN_a);
      BT.print("distance_a");
      BT.println(distance_a);
      delay(10);
    } while (distance_a <= 40);
    int d1 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    delay(10);
    BT.print("预期d1为右侧从左边门框开始的起点，d1:");
    BT.println(d1);
    distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
    BT.print("distance_3");
    BT.println(distance3);
    BT.print("distance_4");
    BT.println(distance4);
    while (distance3 < d1 + 5 && distance4 < d1 + 5) {
      back(1500);
      distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
      distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
      BT.print("distance_3");
      BT.println(distance3);
      BT.print("distance_4");
      BT.println(distance4);
    }
  } else {
    do {
      BT.println("前进");
      forward(1500);
      distance_a = measureDistance(TRIG_PIN_a, ECHO_PIN_a);
      BT.print("distance_a");
      BT.println(distance_a);
      delay(10);
    } while (distance_a > 40);
    stop();
    delay(100);
    int d1 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    delay(10);
    BT.print("预期d1为右侧从左边门框开始的起点，d1:");
    BT.println(d1);
    distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
    BT.print("distance_3");
    BT.println(distance3);
    BT.print("distance_4");
    BT.println(distance4);
    while (distance3 < d1 + 5 || distance4 < d1 + 5) {
      back(1000);
      distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
      distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
      BT.print("distance_3");
      BT.println(distance3);
      BT.print("distance_4");
      BT.println(distance4);
    }
  }

  BT.println("车身到达通道宽度的中点，准备右转");
  leftWithYaw(18.15);
  BT.println("预期右转完毕");

  forwardPID(3000, 15);
  // distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
  // distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
  // BT.print("distance_3");
  // BT.println(distance3);
  // BT.print("distance_4");
  // BT.println(distance4);
  // while (distance3 > 15 && distance4 > 15) {
  //   forward(2000);
  //   distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
  //   distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
  //   BT.print("distance_3");
  //   BT.println(distance3);
  //   BT.print("distance_4");
  //   BT.println(distance4);
  // }

  BT.println("到达左右标识牌前！");




  // /*-------------------------过障碍门，预期来到告示牌前-------------------*/

  int maxDetectionAttempts = 9;  // 设置最大的检测次数
  int detectedCountObject1 = 0;  // 记录成功识别物体1的次数
  int detectedCountObject2 = 0;  // 记录成功识别物体2的次数

  for (int attempt = 0; attempt < maxDetectionAttempts; attempt++) {
    huskylens.request();
    delay(500);  // 等待一段时间以获取稳定的识别结果

    if (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      BT.println(result.ID);

      if (result.ID == 2) {
        BT.println("成功识别物体2！");
        detectedCountObject2++;
      } else {
        BT.println("成功识别物体1！");
        detectedCountObject1++;
      }
    }
  }
  // 根据成功识别的次数来判断最终动作
  if (detectedCountObject1 > maxDetectionAttempts / 2) {
    BT.println("物体1被稳定识别，执行物体1的操作！");
    /*情况一：左转*/
    BT.println("左转");
    leftWithYaw(17.95);

    forwardPID(3000, 8);
    rightWithYaw(17.95);
    forwardPID(3000, 80);
    rightWithYaw(17.8);

    distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
    BT.print("distance_3");
    BT.println(distance3);
    BT.print("distance_4");
    BT.println(distance4);
    while (distance3 > 10 && distance4 > 10) {
      forward(2000);
      distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
      distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
      BT.print("distance_3");
      BT.println(distance3);
      BT.print("distance_4");
      BT.println(distance4);
    }
    leftWithYaw(18);
  } else {
    BT.println("物体2被稳定识别，执行物体2的操作！");
    // 在这里执行物体2的操作
    /*情况二：右转*/
    BT.println("右转");
    rightWithYaw(18.15);




    forwardPID(3000, 10);
    leftWithYaw(18);
    distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
    distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
    BT.print("distance_3");
    BT.println(distance3);
    BT.print("distance_4");
    BT.println(distance4);
    while (distance3 > 10 && distance4 > 10) {
      forward(2000);
      distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
      distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
      BT.print("distance_3");
      BT.println(distance3);
      BT.print("distance_4");
      BT.println(distance4);
    }
    leftWithYaw(17.8);
    distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
    BT.print("distance_1");
    BT.println(distance1);
    BT.print("distance_2");
    BT.println(distance2);
    while (distance1 < 50 && distance2 < 50) {
      forward(2000);
      distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
      distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
      BT.print("distance_1");
      BT.println(distance1);
      BT.print("distance_2");
      BT.println(distance2);
    }

    rightWithYaw(18.15);
    forwardPID(3000, 80);
  }

  // /*hand函数是机械臂抓取放下的函数，住区和放下都一样，直接调用就行*/
  //hand();
  while (true) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
      Serial.println(pixy.ccc.numBlocks);
      Serial.println(pixy.ccc.blocks[j].m_x);
      float poss = pixy.ccc.blocks[j].m_x;
      pixyy(flag_arrive, poss);
      if (flag_arrive && (poss >= 190 && poss <= 225)) {
        stop();
        flag_arrive = false;
        arm();
        break;
      }
    } else {
      forward(500);
    }
  }

  forwardPID(3000, finish);
  rightWithYaw(18.1);
  forwardPID(3000, 8);
  // hand();
  //leftWithYaw(17.95);
  while (true) {
    stop();
  }

  /***************************end小车路径规划&运动逻辑end****************************/
}