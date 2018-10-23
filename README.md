# arduino_bike-s_gun
### arduino+mpu6050实现自动车撑

   #include "Wire.h"
   #include "I2Cdev.h"
   #include "MPU6050.h"
   MPU6050 accelgyro;
   //---------------
   int16_t ax, ay, az;
   int16_t gx, gy, gz;
   //========================
   int i;
   int pin = 13;
   volatile int state = LOW;
   int flog=0,flog1=0;
   int flog_up=0;
   int flog_speed=0;
   unsigned long times=0;
   #define Gry_offset -20     // 陀螺仪偏移量
   #define Gyr_Gain 0.00763358    //对应的1G
   #define pi 3.14159
   /*********** PID控制器参数 *********/
   float kp, ki, kd,kpp; 
   float angleA,omega;
   //float P[2][2] = {{ 1, 0 },{ 0, 1 }};
   //float Pdot[4] ={ 0,0,0,0};
   //static const double C_0 = 1;
   //float abcd=0.0;
   float LOutput,ROutput;   
   char buffer [4];
   //--------------------------------------
   float LSpeed_Need=0.0,RSpeed_Need=0.0;
   //char data;
   int data,adata;
   int speed_=0;

   unsigned long now;
   unsigned long preTime = 0;
   float SampleTime = 0.08;  //-------------------互补滤波+PID 采样时间0.08 s
   unsigned long lastTime;
   float Input, Output, Setpoint;
   float errSum,dErr,error,lastErr,f_angle;
   int timeChange; 

   void setup() {
    Wire.begin();
      pinMode(pin, OUTPUT);
     attachInterrupt(0, blink, RISING);//当int.0电平改变时,触发中断函数blink
      pinMode(9, OUTPUT);
     pinMode(10, OUTPUT);
   accelgyro.initialize();
   Serial.begin(9600); 
   }

   void loop() {
     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
     angleA= atan2(ay , az) * 180 / pi+0.5;   
     omega=  Gyr_Gain * (gx +  Gry_offset); 
     PIDD();

    }

    void  PIDD(){ 
       //------------------互补滤波 ------------------------
     unsigned long now = millis();                           // 当前时间(ms)

       float dt = (now - preTime) / 1000.0;                    // 微分时间(ms)
       preTime = now;  
       float K = 0.8;                    
       float A = K / (K + dt);                    
       f_angle = A * (f_angle + omega * dt) + (1-A) * angleA;  // 互补滤波算法  
         unsigned long last_times=times;
         delay(600);
         Serial.print(last_times);Serial.print("\r"); Serial.print(times);Serial.print("\n");
         if(last_times==times) {times=0;flog_speed=0;}
         else {flog_speed=1;}

       if((f_angle<-50&&flog_up==1)||flog_speed==1)
       {delay(200);
       flog++;
       if(f_angle<-50)
       if(flog==1)
       {up();
       flog1=0;
       flog_up=0;
       }
       }
           if(f_angle>20&&flog_up==0&&flog_speed==0)
       {delay(200);
       flog1++;
       if(f_angle>20)
       if(flog1==1)
       {down();
       flog=0;
       flog_up=1;
       }
       }

    }
    void up(){                                   //车撑上升
       digitalWrite(9, HIGH);   
     digitalWrite(10, LOW);
     for(i=0;i<3;i++)
     delay(1000); 
     delay(500);// wait for a second
       digitalWrite(9, LOW);   
     digitalWrite(10, LOW);
     }

   void down(){                                  //车撑下降
       digitalWrite(9, LOW);    
     digitalWrite(10, HIGH);
       for(i=0;i<3;i++)
     delay(1000);                       // wait for a second
     delay(500);
         digitalWrite(9, LOW);   
     digitalWrite(10, LOW);
     }
     void blink()//中断函数
   {
     state = !state;
     digitalWrite(pin, state);
     times=times+1;
   }
