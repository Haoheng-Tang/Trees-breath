
/**************library**********************************************/
#include <SoftwareSerial.h>

#include "DFRobot_mmWave_Radar.h"

/******************************************************************************/
//rgb_led
int redPin = 11;
int greenPin = 10;
int bluePin = 9;
uint8_t blueValue = 0;
uint8_t greenValue = 0;
int redValue = 0;
int redMax = 245;
int redMin = 0;
int fadeAmount = 1;
int sleep = 500;

//sound_sensor
int sound_din=4;
int sound_ain=A0;
int ad_value;

//mmWave radar
SoftwareSerial mySerial(3, 2);
DFRobot_mmWave_Radar sensor(&mySerial);
int radarPin = 13;        //定义mmWave_radar输入引脚
int radarState = 0;       //读取当前的mmWave_radar状态
int radarLastState = 0;   //上一个mmWave_radar状态
int radarduration = 0;    //mmWave_radar持续感应时长
int n = 0;                //计数
int m = 0;                //一个周期内的计数
int threshold = 4;        //推送一次的计数阈值

//time
unsigned long start = 0;
unsigned long radarloop = 0;
unsigned long radarstart = 0;
unsigned long radarend = 0;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);

 /******************set up mmWave radar***********************************/
  pinMode(radarPin, OUTPUT);
  sensor.factoryReset();    //恢复出厂设置
  sensor.DetRangeCfg(0, 1);    //设置感应距离，最远为9m
  sensor.OutputLatency(0, 0);  //设置输出延时
}


void loop()
{
  /**********************mmWave_radar***********************************/
  if(millis() > radarloop + 1000){
    radarState = sensor.readPresenceDetection();
    digitalWrite(radarPin, radarState);
    radarloop = millis();
  }
  if(radarState == 1 && radarLastState == 0)  //如果读入是高电平，说明有人进入区域
  {
    radarstart = millis();       //开始感应时间
    n = n+1;                     //计数一次
    m = m+1;                     //周期内计数一次
    radarLastState = radarState;   //记录按键状态
    sleep = 0;                    //马上吸气
  }
  else if(radarState == 0 && radarLastState == 1){
    radarend = millis();           //结束感应时间
    radarLastState = radarState;    //记录按键状态
    radarduration = radarend - radarstart;  //计算持续感应时长
    sleep = 500;                   //平静状态

    Serial.print(radarduration);
    Serial.println(n);
  }

   /*************sound_sensor*****************************************/
  ad_value=analogRead(sound_ain);
  if(digitalRead(sound_din)==LOW)
  {
    Serial.print("ad_value:");
    Serial.print(ad_value*5.0/1024);
    Serial.println("V");

    //改变灯光亮度
    if(redValue + ad_value*5.0/52 < redMax){
      redValue = redValue + ad_value*5.0/52;
    }
    else{
      redValue = redMax;
    }
  }


  /*************RGB_LED_breathing***********************************/ 
  // set brightness within the range of (redMin, redMax)
  if (redValue <redMin){
    redValue = redMin;
  }
  if (redValue >redMax){
    redValue = redMax;
  }
  //if(n<20){
  greenValue = redValue *0.48;
  blueValue = redValue *0.53;    
  /*}
  else if(n>=20 && n <30){
  greenValue = redValue * 0.222;
  blueValue = redValue * 0.01;        
  }
  else{
  greenValue = redValue * 0.10;
  blueValue = redValue * 0;      
  }
  */

  if (blueValue <0){
    blueValue = 0;
  }
  if (blueValue >255){
    blueValue = 255;
  }

  if (greenValue <0){
    greenValue = 0;
  }
  if (greenValue >255){
    greenValue = 255;
  }

  setColor(redValue, greenValue, blueValue); 

  // change the brightness for next time through the loop:
  redValue = redValue + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (redValue < redMin) {
    fadeAmount = -fadeAmount;
    delay(sleep);
  }

  if (redValue > redMax) {
    fadeAmount = -fadeAmount;
  }
  
  // wait for 6 milliseconds to see the dimming effect
  delay(6);
  
}


//rgb function
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
