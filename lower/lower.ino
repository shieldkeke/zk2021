#include "PID_v1.h"
#include "Servo.h"
#define in1r 32 //.
#define in2r 34 //.
#define in1l 37 //.
#define in2l 39 //.
#define enar 6 //.
#define enal 9 //.
#define encoderAr 21
#define encoderBr A4
#define encoderAl 19
#define encoderBl A5
#define maxSpeed 250
#define DIR A14
#define PUL A15
#define FORWARD 0
#define BACKWARD 1
#define yaw 10
#define START A0  //开机键

//传给机械臂的位置信息
#define HIGH_POSITION 88
#define LOW_POSITION 888
#define LEFT_POSITION 66
#define MID_POSITION 666
#define RIGHT_POSITION 6666

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

//传感器引脚
 
//to do :init the pins
//0~6 from left to right
#define fsensor(a) (2*a)+41
#define bsensor(a) (2*a)+40 
//原来是40~
#define sidesensor 22
//MACRO DIFINITION
#define r_WHEEL 1
#define l_WHEEL 2
#define STOP 0
#define WHITE 0
#define BLACK 1


//中断号和引脚参照
//2 (interrupt 0), 
//3 (interrupt 1),
//18 (interrupt 5), 
//19 (interrupt 4), 
//20 (interrupt 3), 
//21 (interrupt 2)

//周期脉冲数
double Durationr=0;
double Durationl=0;
//目标周期脉冲数
double setpointr=0;
double setpointl=0;

//输出值
double outputr=maxSpeed;
double outputl=maxSpeed;

//传感器状态
int sensorstate[7]={0};
int sensorstate2[7]={0};
int side_sensorstate=0;
//pid参数,暂时公用
double kp=20,ki=20,kd=0.0;//sampletime=25

//4.2,6,0.05 when sampletime=50

//初始化pid对象
//input,output,setpoint,kp,ki,kd,direction
PID myPIDr(&Durationr, &outputr, &setpointr, kp, ki, kd, DIRECT);
PID myPIDl(&Durationl, &outputl, &setpointl, kp, ki, kd, DIRECT);

//初始化云台舵机
Servo myservo;

int cnt = 0;//lines
int cnt_side = 0;
int cnt_back = 0;
float basic_speed = 18;//
int num = 0; 
bool not_waiting = 1; //判断和上位机通讯是不是可用
int get_data[100] = {0};
int cnt__ =  0;
int check = 0;
int start_flag = 0;

void yaw_servo_init()
{
  myservo.attach(yaw);
  if(myservo.attached())
  {
    Serial.println("##");
  }
  else
  {
    Serial.println("**");
  }
}

void PIDinit()
{
  //设置PID为自动模式
  myPIDr.SetMode(AUTOMATIC);
  myPIDl.SetMode(AUTOMATIC);
  
  //设置PID采样频率为100ms
  myPIDr.SetSampleTime(25);
  myPIDl.SetSampleTime(25);

  //最大输出PWM值
  myPIDr.SetOutputLimits(-maxSpeed, maxSpeed);
  myPIDl.SetOutputLimits(-maxSpeed, maxSpeed);
}

void motor(double Speedl,double Speedr)//setpoint 从负到正都有
{   
    bool result;
    double k = 1.2;
    setpointr=-k*Speedr;
    setpointl=k*Speedl;
    result=myPIDr.Compute(); 
    if(result)
    {
    Durationr=0;
    } 
    result=myPIDl.Compute(); 
    if(result)
    {
    Durationl=0;
    } 
    output(r_WHEEL);
    output(l_WHEEL);
    readSensor();
}

void output(int wheelx)
{
  int in1,in2,ena;
  double output;
  switch(wheelx){
  case 1: in1=in1r;in2=in2r;ena=enar;output=outputr;break;
  case 2: in1=in1l;in2=in2l;ena=enal;output=outputl;break;
  }
  if(output>=0)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else
  {
   digitalWrite(in1,LOW);
   digitalWrite(in2,HIGH);
  } 
  analogWrite(ena,fabs(output));
}

void readSensor()
{
  int i=0;
  for(i=0;i<7;i++)
  {
  sensorstate[i]=digitalRead(fsensor(i));
  sensorstate2[i]=digitalRead(bsensor(i));
  }
  side_sensorstate=digitalRead(sidesensor);
}

void stop_for(int t)
{
    
 
  digitalWrite(in1r,LOW);
  digitalWrite(in2r,LOW);
  digitalWrite(in1l,LOW);
  digitalWrite(in2l,LOW);
  delay(t);
}

void stop_pid()
{
  unsigned long t1=0;
  unsigned long t2=0;
  t1=millis();
  while(1)
  {
    t2=millis();

    
    if(t2-t1>500)
      break;
    motor(0,0);
  }  
  digitalWrite(in1r,LOW);
  digitalWrite(in2r,LOW);
  digitalWrite(in1l,LOW);
  digitalWrite(in2l,LOW);
}

void tracking()
{
    static float k;
    float delta_x;
    float delta_y;
    float delta,delta_last;
    float k2;
    float kd;
    static int sum_last = 0;
    int sum,sum2;
    k2 = 0.5;
    k = 0.3*basic_speed;
    kd=0.0;
    readSensor();
    delta_x = (-3*(sensorstate[0]==WHITE) - 2*(sensorstate[1]==WHITE) - (sensorstate[2]==WHITE) + (sensorstate[4]==WHITE) + 2*(sensorstate[5]==WHITE) + 3*(sensorstate[6]==WHITE));
    delta_y = (3*(sensorstate2[0]==WHITE) + 2*(sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) - (sensorstate2[4]==WHITE) - 2*(sensorstate2[5]==WHITE) - 3*(sensorstate2[6]==WHITE));
    sum = (sensorstate[0]==WHITE) + (sensorstate[1]==WHITE) + (sensorstate[2]==WHITE) + (sensorstate[3]==WHITE) + (sensorstate[4]==WHITE) + (sensorstate[5]==WHITE) + (sensorstate[6]==WHITE);
    sum2 = (sensorstate2[0]==WHITE) + (sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) + (sensorstate2[3]==WHITE) + (sensorstate2[4]==WHITE) + (sensorstate2[5]==WHITE) + (sensorstate2[6]==WHITE);
    if(sum)
    {
      delta_x = delta_x/sum;
    }
    if(sum2)
    {
      delta_y = delta_y/(sum2);
    }
    delta = delta_x - delta_y + k2*(delta_x + delta_y);
    motor(basic_speed + k*delta , basic_speed - k*delta);
    delta_last=delta;
}

void tracking_slow()
{
    static float k;
    float delta_x;
    float delta_y;
    float delta,delta_last;
    float k2;
    float kd;
    static int sum_last = 0;
    int sum,sum2;
    k2 = 0.5;
    k = 0.3*basic_speed;
    kd=0.0;
    readSensor();
    delta_x = (-3*(sensorstate[0]==WHITE) - 2*(sensorstate[1]==WHITE) - (sensorstate[2]==WHITE) + (sensorstate[4]==WHITE) + 2*(sensorstate[5]==WHITE) + 3*(sensorstate[6]==WHITE));
    delta_y = (3*(sensorstate2[0]==WHITE) + 2*(sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) - (sensorstate2[4]==WHITE) - 2*(sensorstate2[5]==WHITE) - 3*(sensorstate2[6]==WHITE));
    sum = (sensorstate[0]==WHITE) + (sensorstate[1]==WHITE) + (sensorstate[2]==WHITE) + (sensorstate[3]==WHITE) + (sensorstate[4]==WHITE) + (sensorstate[5]==WHITE) + (sensorstate[6]==WHITE);
    sum2 = (sensorstate2[0]==WHITE) + (sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) + (sensorstate2[3]==WHITE) + (sensorstate2[4]==WHITE) + (sensorstate2[5]==WHITE) + (sensorstate2[6]==WHITE);
    if(sum)
    {
      delta_x = delta_x/sum;
    }
    if(sum2)
    {
      delta_y = delta_y/(sum2);
    }
    delta = delta_x - delta_y + k2*(delta_x + delta_y);
    motor(basic_speed + k*delta-5 , basic_speed - k*delta-5);
    delta_last=delta;
}

void tracking_back()
{
    static float k;
    float delta_x;
    float delta_y;
    float delta,delta_last;
    float k2;
    float kd;
    static int sum_last = 0;
    int sum,sum2;
    k2 = 0.5;
    k = 0.3*basic_speed;
    kd=0.0;
    readSensor();
    delta_x = (-3*(sensorstate[0]==WHITE) - 2*(sensorstate[1]==WHITE) - (sensorstate[2]==WHITE) + (sensorstate[4]==WHITE) + 2*(sensorstate[5]==WHITE) + 3*(sensorstate[6]==WHITE));
    delta_y = (3*(sensorstate2[0]==WHITE) + 2*(sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) - (sensorstate2[4]==WHITE) - 2*(sensorstate2[5]==WHITE) - 3*(sensorstate2[6]==WHITE));
    sum = (sensorstate[0]==WHITE) + (sensorstate[1]==WHITE) + (sensorstate[2]==WHITE) + (sensorstate[3]==WHITE) + (sensorstate[4]==WHITE) + (sensorstate[5]==WHITE) + (sensorstate[6]==WHITE);
    sum2 = (sensorstate2[0]==WHITE) + (sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) + (sensorstate2[3]==WHITE) + (sensorstate2[4]==WHITE) + (sensorstate2[5]==WHITE) + (sensorstate2[6]==WHITE);
    if(sum)
    {
      delta_x = delta_x/sum;
    }
    if(sum2)
    {
      delta_y = delta_y/(sum2);
    }
    delta = delta_x - delta_y + k2*(delta_x + delta_y);
    motor(-basic_speed + k*delta+kd*(delta_last-delta) , -basic_speed - k*delta+kd*(delta_last-delta));
    delta_last=delta;
}

void tracking_t(int t)
{
  int i;
  for (i = 1;i<=fabs(t);i++)
  {
    tracking();
  }  
}

void tracking_back_t(int t)
{
  int i;
  for (i = 1;i<=fabs(t);i++)
  {
    tracking_back();
  }  
}

void side_stop()
{
  readSensor();
  while(side_sensorstate!=WHITE)
  {
    readSensor();
    tracking_slow(); 
  }
  stop_pid();
}

void back_stop()
{
  readSensor();  
  int sum2 = (sensorstate2[0]==WHITE) + (sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) + (sensorstate2[3]==WHITE) + (sensorstate2[4]==WHITE) + (sensorstate2[5]==WHITE) + (sensorstate2[6]==WHITE); 
  while(sum2<5)
  {
    tracking();
    readSensor();
    sum2 = (sensorstate2[0]==WHITE) + (sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) + (sensorstate2[3]==WHITE) + (sensorstate2[4]==WHITE) + (sensorstate2[5]==WHITE) + (sensorstate2[6]==WHITE); 
  }
  stop_pid();
}

void front_stop()
{
  readSensor();  
  int sum = (sensorstate[0]==WHITE) + (sensorstate[1]==WHITE) + (sensorstate[2]==WHITE) + (sensorstate[3]==WHITE) + (sensorstate[4]==WHITE) + (sensorstate[5]==WHITE) + (sensorstate[6]==WHITE); 
  while(sum<5)
  {
    tracking_back();
    readSensor();
    sum = (sensorstate[0]==WHITE) + (sensorstate[1]==WHITE) + (sensorstate[2]==WHITE) + (sensorstate[3]==WHITE) + (sensorstate[4]==WHITE) + (sensorstate[5]==WHITE) + (sensorstate[6]==WHITE); 
  }
  stop_pid();
}

void catch_box()
{
  delay(2000);
}
//chunleft:3 
int catchbox=0;
int flag1=0;
int route[]={0,1,3,0,0,0,0,0,2,
             3,3,3,0,0,0,0,0,2,
             3,3,3,0,0,0,0,0,2,
             3,3,3,0,0,0,0,0,2,3,4};
void car_run()
{ 
    tracking();
    readSensor(); 
    int sum2 = (sensorstate2[0]==WHITE) + (sensorstate2[1]==WHITE) + (sensorstate2[2]==WHITE) + (sensorstate2[3]==WHITE) + (sensorstate2[4]==WHITE) + (sensorstate2[5]==WHITE) + (sensorstate2[6]==WHITE);  
    int sum  = (sensorstate[0]==WHITE) + (sensorstate[1]==WHITE) + (sensorstate[2]==WHITE) + (sensorstate[3]==WHITE) + (sensorstate[4]==WHITE) + (sensorstate[5]==WHITE) + (sensorstate[6]==WHITE);
   
    if(side_sensorstate == WHITE)
    {
      if (cnt_side<cnt)
      {
        cnt_side++;
      }
    }
    if(sum>=5)
    {
        cnt++;
        switch(route[cnt])
        {
          case 0://停下来抓
            side_stop();
            to_catch();
            straight_t(100);
            break;//之后这里就放识别和抓取的函数
          case 1://左转
            back_stop();
            left();
            straight_t(100);
            break;
          case 2://抓加右转
            side_stop();
            to_catch();
            back_stop();
            right();
            straight_t(100);
            break;
          case 3://直走
            side_stop();
            straight_t(100);
            break;
          case 4://倒退回启动区
            back_stop();
            right();
            front_stop();
            motor(-15,-15);
            delay(10000);
            break;
          case 5://右转
            side_stop();
            back_stop();
            right();
            straight_t(100);
            break;
        }
    }
}

void turnClockwise(double speed)
{
    motor(speed,-speed);
}
void turnLeft_t(int t)
{
  int i;
  for (i = 1;i<=t;i++)
  {
    turnClockwise(-basic_speed);
    delay(1);
  }
}

void straight_t(int t)
{
  int i;
  for (i = 1;i<=fabs(t);i++)
  {
    if(t>0)
    motor(15,15);
    else 
    motor(-15,-15);
    delay(1);
  }  
}
void turnLeft()
{
  readSensor();
  while(sensorstate[0]!=WHITE)
  {
    turnClockwise(-basic_speed+10);
  }
  while(sensorstate[1]!=WHITE)
  {
    turnClockwise(-basic_speed+10);
  }
  while(sensorstate[3]!=WHITE)
  {
    turnClockwise(-basic_speed+15);
  }
//  while(sensorstate2[4]!=WHITE)
//  {
//    turnClockwise(-basic_speed+15);
//   }
  stop_pid();
  delay(200);
}

void turnRight()
{
  readSensor();
  while(sensorstate[6]!=WHITE)
  {
    turnClockwise(basic_speed-10);//要不要把这个速度改大一点？
  }
  while(sensorstate[5]!=WHITE)
  {
    turnClockwise(basic_speed-10);
  }
  while(sensorstate[4]!=WHITE)
  {
    turnClockwise(basic_speed-10);
  }
  while(sensorstate[3]!=WHITE)
  {
    turnClockwise(basic_speed-15);
  }
  stop_pid();
  delay(200);
}

void left()
{
  stop_pid();
//  tracking_back_t(1000);//调整转弯位置
  straight_t(-400);
  stop_pid();
  turnClockwise(-basic_speed);
  delay(300);
  turnLeft();
  stop_pid();
  motor(-basic_speed,-basic_speed);
}

void right()
{
  stop_pid();
//  straight_t(-600);//调整转弯位置
//  tracking_back_t(1500);
  straight_t(-600);
  stop_pid();
  turnRight();
  stop_pid();
  motor(-basic_speed,-basic_speed);
}

void motorInit()
{
  pinMode(in1r,OUTPUT);
  pinMode(in2r,OUTPUT);
  pinMode(enar,OUTPUT);
  pinMode(in1l,OUTPUT);
  pinMode(in2l,OUTPUT);
  pinMode(enal,OUTPUT);
}

void sensorInit()
{
    int i=0;
  for(i=0;i<7;i++)
  {
    pinMode(fsensor(i),INPUT);
    pinMode(bsensor(i),INPUT);
  }
  pinMode(sidesensor,INPUT);
}  

//中断号参照：
//2 (interrupt 0), 
//3 (interrupt 1),
//18 (interrupt 5), 
//19 (interrupt 4), 
//20 (interrupt 3), 
//21 (interrupt 2)

void counterr()//二倍频计数
{
 if(digitalRead(encoderAr)==LOW)
 {
  if(digitalRead(encoderBr)==LOW) Durationr++;
  else Durationr--;
 }
 else 
 {
  if(digitalRead(encoderBr)==LOW) Durationr--;
  else Durationr++;
 }
}

void counterl()//二倍频计数
{
 if(digitalRead(encoderAl)==LOW)
 {
  if(digitalRead(encoderBl)==LOW) Durationl++;
  else Durationl--;
 }
 else 
 {
  if(digitalRead(encoderBl)==LOW) Durationl--;
  else Durationl++;
 }
}

void encoderInit()
{
  pinMode(encoderAr,INPUT);  
  pinMode(encoderBr,INPUT);  
  attachInterrupt(2, counterr, CHANGE);
  pinMode(encoderAl,INPUT);  
  pinMode(encoderBl,INPUT);  
  attachInterrupt(4, counterl, CHANGE);
}

////////////机械臂

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  Serial1.write(buf, 10);
}

void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}
// 和上位机串口通信函数
int get_positions(int &n,int a[])
{
  char head;
  int i,j;
  if (not_waiting)
  {
    Serial.print('@');
    not_waiting = 0;
    return 0;
  }
  while (Serial.available())//用while 保证读到最新的
  {
    head = Serial.read();
    if (head=='#')
    {     
      n = Serial.parseInt(); 
      for (i = 1;i<=n;i++)
      {
        a[i] = Serial.parseInt();  
      }
      not_waiting = 1;//不知道这样会不会出问题
    }
  }
  if (not_waiting)
  {
    return 1;
  }
  return 0;
}

void move_motor(int dir,int h, int v)
{
  int i=0;
  if ((h==LEFT_POSITION||h==RIGHT_POSITION)&&(v==HIGH_POSITION))
  {
    for(i=0;i<100;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL,LOW);
      delayMicroseconds(500);
    }
    for(i=0;i<3000;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(300);
      digitalWrite(PUL,LOW);
      delayMicroseconds(300);
    }
    for(i=0;i<100;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL,LOW);
      delayMicroseconds(500);
    }
  }
  else
  {
    for(i=0;i<100;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL,LOW);
      delayMicroseconds(500);
    }
    for(i=0;i<2500;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(300);
      digitalWrite(PUL,LOW);
      delayMicroseconds(300);
    }
    for(i=0;i<100;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL,LOW);
      delayMicroseconds(500);
    }
  }
}

void myservo_move(int angle)
{
  int start;
  start = myservo.read();
  while(abs(start - angle)>=10)
  {
    start += (angle - start)/abs(angle - start) * 10;
    myservo.write(start);
    delay(80);
  }
  myservo.write(angle);
}

void wait_mode()
{
  myservo_move(100);
  delay(500);
  LobotSerialServoMove(1,550,1000);
  delay(1000);
  LobotSerialServoMove(2,750,1000);
  LobotSerialServoMove(3,900,1000);
  LobotSerialServoMove(4,850,1000);
  delay(1000);
}

void put_task(int v)
{
  if(v == HIGH_POSITION)
  {
    myservo_move(137);
    LobotSerialServoMove(1,550,1000);
    delay(1000);
    LobotSerialServoMove(1,550,1000);
    delay(1000);
  }
  else if(v == LOW_POSITION)
  {
    myservo_move(137);
    LobotSerialServoMove(2,700,1000);
    delay(1000);
    LobotSerialServoMove(1,550,1000);
    delay(1000);
    LobotSerialServoMove(1,550,1000);
    delay(1000);
  }
}

//用于将购物车与车连接
void get_car_connected()
{
  LobotSerialServoMove(4,950,1000);
  LobotSerialServoMove(3,500,1000);
  LobotSerialServoMove(1,800,1000);
  LobotSerialServoMove(2,500,1000);
  delay(1000);
  myservo_move(140);
  LobotSerialServoMove(2,200,1000);
  delay(1000);
  LobotSerialServoMove(2,500,1000);
  LobotSerialServoMove(4,800,1000);
  delay(1000);
}

//慢速调整步进电机，避免手拧
void motor_adjust(int dir)
{
  while(1)
  {
    int i=0;
    for(i=0;i<500;i++)
    {
      digitalWrite(DIR,dir);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(500);
      digitalWrite(PUL,LOW);
      delayMicroseconds(500);
    }
    delay(500);
  }
}
bool  left_object = 0;
bool  right_object = 0;
bool  mid_object = 0;
int high_p;

void get_objects()
{
  int i;
  int width = 640;
  for(i=1;i<=num;i++)
  {
    if(get_data[i] < 0)
    {
      break;
    }
    if (get_data[i]<=width/3)
    {
      left_object = 1;
    }
    else if((get_data[i]<=2*width/3))
    {
      mid_object = 1;
    }
    else
    {
      right_object = 1;
    }
  }
}
void camera_move_up()
{
  LobotSerialServoMove(6,840,500);
  high_p = HIGH_POSITION;
  delay(800);
}
void camera_move_down()
{
  LobotSerialServoMove(6,950,500);
  high_p = LOW_POSITION;
  delay(800);
}
int catch_flag;
void to_catch()
{
  camera_move_up();
  __to_catch();
  camera_move_down();
  __to_catch();
  if(catch_flag)
  {
    wait_mode();
    catch_flag = 0;
  }
}
void __to_catch()
{
  left_object = 0;
  right_object = 0;
  mid_object = 0;
  long t1 = millis();
  while(1)//超时
  { 
    long t2 = millis();
    if (t2-t1>2500)
    {
      break;
    }
    if(get_positions(num,get_data))
    {
      get_objects();
      break;
    }
  }
  if(mid_object)
  {
    catch_task(MID_POSITION,high_p);
    catch_flag = 1;
  }
  if(left_object)
  {
    catch_flag = 1;
    catch_task(LEFT_POSITION,high_p);
  }
  if(right_object)
  {
    catch_flag = 1;
    catch_task(RIGHT_POSITION,high_p);
  }
}

void catch_task(int h,int v)
{
  int HP;
  switch(h){
    case LEFT_POSITION: HP=43;break;
    case MID_POSITION: HP=25;break;
    case RIGHT_POSITION: HP=10;break; 
  }
  if(v==HIGH_POSITION)
  {
    myservo_move(80);
    LobotSerialServoMove(4,500,1000);
    LobotSerialServoMove(3,500,1000);
    LobotSerialServoMove(2,0,1000);
    delay(1000);
    myservo_move(HP);
    LobotSerialServoMove(4,580,1000);
    LobotSerialServoMove(3,680,1000);
    LobotSerialServoMove(2,240,1000);
    delay(300);   
    move_motor(FORWARD,h,v);
    LobotSerialServoMove(1,850,1000);
    delay(1500);
    LobotSerialServoMove(2,350,1000);
    move_motor(BACKWARD,h,v);
    LobotSerialServoMove(1,900,1000);
    LobotSerialServoMove(4,500,1000);
    LobotSerialServoMove(3,500,1000);
    LobotSerialServoMove(2,50,1300);
    delay(500);
    myservo_move(80);
    LobotSerialServoMove(1,900,1000);
    LobotSerialServoMove(4,800,1000);
    LobotSerialServoMove(3,500,1000);
    LobotSerialServoMove(2,300,1000);
  }
  else if(v==LOW_POSITION)
  {
    LobotSerialServoMove(2,900,1000);
    LobotSerialServoMove(3,850,1000);
    LobotSerialServoMove(4,870,1000);
    delay(1000);
    myservo_move(HP-2);
    delay(300);
    LobotSerialServoMove(2,700,1000);
    move_motor(FORWARD,h,v);
    LobotSerialServoMove(1,850,1000);
    delay(1000);
    move_motor(BACKWARD,h,v);
    LobotSerialServoMove(2,850,1000);
    delay(1000);
  }
  put_task(v);
}

///////////初始化
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  motorInit();
  yaw_servo_init();
  encoderInit();  
  PIDinit();
  sensorInit();
  pinMode(A14,OUTPUT);
  pinMode(A15,OUTPUT);
}

void loop() {
  while(1)
  {
    bool start_switch = digitalRead(START);
    if(start_switch)
    {
      break;
    }
  }
  if(start_flag == 0)
  {
    //delay(10000);
    get_car_connected();
    wait_mode();
    start_flag = 1;
  }

  car_run();
}
