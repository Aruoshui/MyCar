// #include <Arduino.h>
// #include <SoftwareSerial.h>

// SoftwareSerial BT(9, 8); //定义蓝牙串口的引脚，蓝牙只需接4根线，Vcc-5V GND-GND TXD-Pin9 RXD-Pin8

// #define TRIG_PIN_1 18  // 1号超声波16引脚为输入
// #define ECHO_PIN_1 19  // 1号超声波17引脚为控制

// #define TRIG_PIN_2 24  // 2号超声波24引脚为输入
// #define ECHO_PIN_2 25  // 2号超声波25引脚为控制

// #define TRIG_PIN_3 22  // 3号超声波22引脚为输入
// #define ECHO_PIN_3 23  // 3号超声波23引脚为控制

// #define TRIG_PIN_4 40  // 4号超声波40引脚为输入
// #define ECHO_PIN_4 39  // 4号超声波39引脚为控制

// #define TRIG_PIN_a 28  // a号超声波28引脚为输入
// #define ECHO_PIN_a 29  // a号超声波29引脚为控制

// #define TRIG_PIN_b 37  // b号超声波37引脚为输入
// #define ECHO_PIN_b 38  // b号超声波38引脚为控制

// const unsigned long interval = 300;  // 测量间隔（毫秒）
// unsigned long previousMillis = 0;

// int count = 0;//控制获取超声波次数，最开始时，获取的超声波数据都是0
// boolean turnleft = true;
// int qhlength;
// unsigned long rx_time = 0;
// unsigned char cmd [8] = {0x19,0x88,0x00,0x00,0x00,0x00,0x00,0x11};
// const int DSD = 20; //Default Servo Delay (默认电机运动延迟时间)
// static int i = 0;

// void setup() {
//   Serial.begin(9600);
//   BT.begin(9600);
//   Serial2.begin(115200);  //驱动板初始化
//   Serial3.begin(115200);
//   Serial2.write(0x55);
//   delay(10);
//   Serial3.write(0x55);
//   delay(10);
//   pinMode(TRIG_PIN_1, OUTPUT);
//   pinMode(ECHO_PIN_1, INPUT);

//   pinMode(TRIG_PIN_2, OUTPUT);
//   pinMode(ECHO_PIN_2, INPUT);

//   pinMode(TRIG_PIN_3, OUTPUT);
//   pinMode(ECHO_PIN_3, INPUT);

//   pinMode(TRIG_PIN_4, OUTPUT);
//   pinMode(ECHO_PIN_4, INPUT);

//   pinMode(TRIG_PIN_a, OUTPUT);
//   pinMode(ECHO_PIN_a, INPUT);

//   pinMode(TRIG_PIN_b, OUTPUT);
//   pinMode(ECHO_PIN_b, INPUT);
// }
// void youpian(int zuo,int you) //700 300
// {
//  setcmd1('m',-zuo);
//  setcmd1('M',zuo);
//  setcmd2('m',you);
//  setcmd2('M',-you);
// }
// void zuopian(int zuo,int you) //300 700
// {
//  setcmd1('m',-zuo);
//  setcmd1('M',zuo);
//  setcmd2('m',you);
//  setcmd2('M',-you);
// }


// void forward(int qian) //600
// {
//  setcmd1('m',-qian);
//  setcmd1('M',qian);
//  setcmd2('m',qian);
//  setcmd2('M',-qian);
// }
// void back(int hou) //600
// {
//  setcmd1('m',hou);
//  setcmd1('M',-hou);
//  setcmd2('m',-hou);
//  setcmd2('M',hou);
// }
// void stop()
// {
//  setcmd1('m',0);
//  setcmd1('M',0);
//  setcmd2('m',0);
//  setcmd2('M',0);
// }
// void left(int zuo ) //500
// {
//  setcmd1('m',zuo);
//  setcmd1('M',-zuo);
//  setcmd2('m',zuo);
//  setcmd2('M',-zuo);
// }
// void right(int you) //500
// {
//  setcmd1('m',-you);
//  setcmd1('M',you);
//  setcmd2('m',-you);
//  setcmd2('M',you);
// }
// void setcmd1(unsigned char Cmd_Send,long MotorSpeed)
// {
//  cmd[2]=Cmd_Send;
//  cmd[3] = (unsigned char)(MotorSpeed >> 24);
//  cmd[4] = (unsigned char)(MotorSpeed >> 16);
//  cmd[5] = (unsigned char)(MotorSpeed >> 8);
//  cmd[6] = (unsigned char)(MotorSpeed);
//  Serial2.write(cmd,8);
//  return;
// }
// void setcmd2(unsigned char Cmd_Send,long MotorSpeed)
// {
//  cmd[2]=Cmd_Send;
//  cmd[3] = (unsigned char)(MotorSpeed >> 24);
//  cmd[4] = (unsigned char)(MotorSpeed >> 16);
//  cmd[5] = (unsigned char)(MotorSpeed >> 8);
//  cmd[6] = (unsigned char)(MotorSpeed);
//  Serial3.write(cmd,8);
//  return;
// }

// float measureDistance(int trigPin, int echoPin) {
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   unsigned long pulseDuration = pulseIn(echoPin, HIGH, 30000);  // 加时
//   float distance = pulseDuration * 0.0343 / 2;                  // 计算距离

//   return distance;
// }



// void loop() {
//     float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
//     float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
//     float distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
//     float distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);

//     Serial.println("****************************");
//     Serial.print("Distance from sensor 1: ");
//     Serial.print(distance1);
//     Serial.println(" cm");

//     Serial.print("Distance from sensor 2: ");
//     Serial.print(distance2);
//     Serial.println(" cm");

//     Serial.println("****************************");
//     Serial.print("Distance from sensor 3: ");
//     Serial.print(distance3);
//     Serial.println(" cm");

//     Serial.print("Distance from sensor 4: ");
//     Serial.print(distance4);
//     Serial.println(" cm");
// //    Serial.print("Difference: ");
// //    Serial.println(chazhi);
// //      float realdistance1 = 0;
// //      float realdistance2 = 0;
// //      float realdistance3 = 0;
// //      float realdistance4 = 0;
// //      float realdistance_a = 0;
// //      float realdistance_b = 0;
// /*起点调整位置：车身回正*/
// //      realdistance1 = calculateDistance(1);
// //      realdistance2 = calculateDistance(2);
// //      BT.println("起点位置调整前");
// //      BT.print("realdistance1:");BT.println(realdistance1);
// //      BT.print("realdistance2:");BT.println(realdistance2);
// //      if(realdistance1 < realdistance2){
// //          do{
// //            right(200);
// //            realdistance1 = calculateDistance(1);
// //            realdistance2 = calculateDistance(2);
// //          }while((realdistance2 - 0.75) <= realdistance1 && realdistance1 <= (realdistance + 0.75));
// //            }while(realdistance1 != realdistance2);
// //      }
// //      else if(realdistance1 > realdistance2 - 0.75){
// //        else if(realdistance1 > realdistance2){
// //          do{
// //            left(200);
// //            realdistance1 = calculateDistance(1);
// //            realdistance2 = calculateDistance(2);
// //          }while((realdistance2 - 0.75) <= realdistance1 && realdistance1 <= (realdistance + 0.75));
// //            }while(realdistance1 != realdistance2);
// //      }
// //      BT.println("起点位置调整后，1和2距离预期相等");
// //      BT.print("realdistance1:");BT.println(realdistance1);
// //      BT.print("realdistance2:");BT.println(realdistance2);
// /*先前进至桥附近*/
// //      forwardPID(500, 32);
//       do{
//         forward(500);
//         distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
//         distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
//       }while(distance3 > 32 && distance4 > 32);
//       Serial.println("前方距离小于32cm");
// /*向右转至对准桥面*/
//       do{
// //        if((realdistance2 - 0.75) <= realdistance1 && realdistance1 <= (realdistance + 0.75)) break;
// //        if((realdistance1 >= realdistance2) && (realdistance1 < 30 && realdistance2 < 30))break;
//         BT.println("向右转");
//         right(390);
//         distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
//         distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
//         BT.print("distance1:");BT.println(distance1);
//         BT.print("distance2:");BT.println(distance2);
//         delay(100);
//       }while(abs(distance1 - distance2) > 0.5);
//       stop();


// // //  // float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
// // //  // float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
// // //  // float chazhi = distance1 - distance2;
// // //
// // //  // Serial.println("****************************");
// // //  // Serial.print("Distance from sensor 1: ");
// // //  // Serial.print(distance1);
// // //  // Serial.println(" cm");
// // //
// // //  // Serial.print("Distance from sensor 2: ");
// // //  // Serial.print(distance2);
// // //  // Serial.println(" cm");

// //   // Serial.print("Difference: ");
// //   // Serial.println(chazhi);
// //   // delay(1000);
// }

#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(9, 8);  //定义蓝牙串口的引脚，蓝牙只需接4根线，Vcc-5V GND-GND TXD-Pin9 RXD-Pin8

#define TRIG_PIN_1 18  // 1号超声波16引脚为输入
#define ECHO_PIN_1 19  // 1号超声波17引脚为控制

#define TRIG_PIN_2 24  // 2号超声波24引脚为输入
#define ECHO_PIN_2 25  // 2号超声波25引脚为控制

#define TRIG_PIN_3 22  // 3号超声波22引脚为输入
#define ECHO_PIN_3 23  // 3号超声波23引脚为控制

#define TRIG_PIN_4 53  // 4号超声波40引脚为输入
#define ECHO_PIN_4 52  // 4号超声波39引脚为控制

#define TRIG_PIN_a 28  // a号超声波28引脚为输入
#define ECHO_PIN_a 29  // a号超声波29引脚为控制

#define TRIG_PIN_b 37  // b号超声波37引脚为输入
#define ECHO_PIN_b 38  // b号超声波38引脚为控制

const unsigned long interval = 300;  // 测量间隔（毫秒）
unsigned long previousMillis = 0;

int count = 0;  //控制获取超声波次数，最开始时，获取的超声波数据都是0
boolean turnleft = true;
int qhlength;
unsigned long rx_time = 0;
unsigned char cmd[8] = { 0x19, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11 };
const int DSD = 20;  //Default Servo Delay (默认电机运动延迟时间)
static int i = 0;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  Serial2.begin(115200);  //驱动板初始化
  Serial3.begin(115200);
  Serial2.write(0x55);
  delay(10);
  Serial3.write(0x55);
  delay(10);
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
}
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

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long pulseDuration = pulseIn(echoPin, HIGH, 30000);  // 加时
  float distance = pulseDuration * 0.0343 / 2;                  // 计算距离

  return distance;
}



void loop() {
  

  // float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
  // delay(10);
  // float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
  // delay(10);
  // float distance3 = measureDistance(TRIG_PIN_3, ECHO_PIN_3);
  // delay(10);
  // float distance4 = measureDistance(TRIG_PIN_4, ECHO_PIN_4);
  
   float distance1 = calculateDistance(1);
   delay(10);
  float distance2 = calculateDistance(2);
  delay(10);
  float distance3 = calculateDistance(3);
  delay(10);
  float distance4 = calculateDistance(4);

  
  // float distance1 = calculateDistance(1);
  // float distance2 = calculateDistance(2);
  // float distance3 = calculateDistance(3);
  // float distance4 = calculateDistance(4);

    Serial.println("****************************");
  Serial.print("Distance from sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");

  Serial.print("Distance from sensor 2: ");
  Serial.print(distance2);
  Serial.println(" cm");

  Serial.println("****************************");
  Serial.print("Distance from sensor 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  Serial.print("Distance from sensor 4: ");
  Serial.print(distance4);
  Serial.println(" cm");

  forward(1000);
  // BT.println("****************************");
  // BT.print("Distance from sensor 1: ");
  // BT.print(distance1);
  

  
  // BT.print("Distance from sensor 2: ");
  // BT.print(distance2);
  

  // BT.println("****************************");
  // BT.print("Distance from sensor 3: ");
  // BT.print(distance3);
  

  // BT.print("Distance from sensor 4: ");
  // BT.print(distance4);
  
  
}

//卡曼尔滤波算法
float klm(int trig, int echo){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long pulseDuration = pulseIn(echo, HIGH, 30000);  // 加时
  float distance = pulseDuration * 0.0343 / 2;                  // 计算距离
  
  float P=1;
  float X=0;
  float Q=0.01; // 噪声
  float R=0.2; // R如果很大，更相信预测值，那么传感器反应就会迟钝，反之相反
  
  float X_ = X + 0;
  float P_ = P + Q;
  float K = P_ / (P_ + R);
  
  X = X_ + K * (distance - X_);
  P = P_ - K * P_;
  return X;
}
//教材滤波


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
    
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += distances[i];
    }
    float average = sum / 10.0;
    float standardDeviation = 0;
    for (int i = 0; i < 10; i++) {
        standardDeviation += (distances[i] - average) * (distances[i] - average);
    }
    standardDeviation = sqrt(standardDeviation / 10.0); // 修正标准差计算，应该除以10.0
    float midDistance = distances[4];
    float residualError = abs(midDistance - average);
    if (residualError <= 3 * standardDeviation) {
        return midDistance;   
    } else {
        return average;  
    }
}


