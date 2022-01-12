/**************library***********************************************/
#include <L3G4200D.h>
L3G4200D gyroscope;

#include <SoftwareSerial.h> 
SoftwareSerial mySerial(13, 12); // RX, TX  通过软串口连接esp8266

/**********************************************************************/
//rgb_led
int redPin = 11;
int greenPin = 10;
int bluePin = 9;
uint8_t blueValue = 0;
uint8_t greenValue = 0;
int redValue = 0;
int redMax = 175;
int redMin = 0;
int fadeAmount = 1;

//gyroscope
int GyBtnState;          //读取当前的gyroscope状态
int GyLastState = LOW;   //上一个gyroscope状态

//ir_sensor
int IrBtnState;          //读取当前的IR_sensor状态
int IrLastState = LOW;   //上一个IR_sensor状态
int IrPin = 8;           //定义IR_sensor输入引脚
int n = 0;               //计数
int m = 0;               //一个周期内的计数
int threshold = 4;       //推送一次的计数阈值

//sound_sensor
int sound_din=2;
int sound_ain=A0;
int ad_value;

 
void setup()
{ 
  Serial.begin(115200);
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  

  /******************set up gyroscope****************************************/
  Serial.println("Initialize L3G4200D");
  /*
  while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }
  */

  if(gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)){
    setColor(200, 0, 0); 
    delay(200);
    setColor(0, 0, 0); 
    delay(200);
  }

  /*
  // Check selected scale
  Serial.print("Selected scale: ");
 
  switch(gyroscope.getScale())
  {
    case L3G4200D_SCALE_250DPS:
      Serial.println("250 dps");
      break;
    case L3G4200D_SCALE_500DPS:
      Serial.println("500 dps");
      break;
    case L3G4200D_SCALE_2000DPS:
      Serial.println("2000 dps");
      break;
    default:
      Serial.println("unknown");
      break;
  }
 
  // Check Output Data Rate and Bandwidth
  Serial.print("Output Data Rate: ");
 
  switch(gyroscope.getOdrBw())
  {
    case L3G4200D_DATARATE_800HZ_110:
      Serial.println("800HZ, Cut-off 110");
      break;
    case L3G4200D_DATARATE_800HZ_50:
      Serial.println("800HZ, Cut-off 50");
      break;
    case L3G4200D_DATARATE_800HZ_35:
      Serial.println("800HZ, Cut-off 35");
      break;
    case L3G4200D_DATARATE_800HZ_30:
      Serial.println("800HZ, Cut-off 30");
      break;
    case L3G4200D_DATARATE_400HZ_110:
      Serial.println("400HZ, Cut-off 110");
      break;
    case L3G4200D_DATARATE_400HZ_50:
      Serial.println("400HZ, Cut-off 50");
      break;
    case L3G4200D_DATARATE_400HZ_25:
      Serial.println("400HZ, Cut-off 25");
      break;
    case L3G4200D_DATARATE_400HZ_20:
      Serial.println("400HZ, Cut-off 20");
      break;
    case L3G4200D_DATARATE_200HZ_70:
      Serial.println("200HZ, Cut-off 70");
      break;
    case L3G4200D_DATARATE_200HZ_50:
      Serial.println("200HZ, Cut-off 50");
      break;
    case L3G4200D_DATARATE_200HZ_25:
      Serial.println("200HZ, Cut-off 25");
      break;
    case L3G4200D_DATARATE_200HZ_12_5:
      Serial.println("200HZ, Cut-off 12.5");
      break;
    case L3G4200D_DATARATE_100HZ_25:
      Serial.println("100HZ, Cut-off 25");
      break;
    case L3G4200D_DATARATE_100HZ_12_5:
      Serial.println("100HZ, Cut-off 12.5");
      break;
    default:
      Serial.println("unknown");
      break;
  }
  */

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate();
 
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  gyroscope.setThreshold(3);

 
  mySerial.begin(115200);
}
 
void loop()
{
    /********************IR_sensor***************************************/
  IrBtnState = digitalRead(IrPin);
  if(IrBtnState == HIGH && IrLastState == LOW)  //如果读入是高电平，说明有人进入区域
  {
    n = n+1;                     //计数一次
    m = m+1;                     //周期内计数一次
    IrLastState = IrBtnState;    //记录按键状态
    //fadeAmount = 3*fadeAmount;     //呼吸加快
  }
  else if(IrBtnState == LOW && IrLastState == HIGH){
    IrLastState = IrBtnState;    //记录按键状态
    //fadeAmount = fadeAmount/3;     //呼吸减慢
  }

  if(m >= threshold){
    m = 0;
  }
  Serial.println(n);


  /********************Gyroscope****************************************/
  // Read normalized values
  Vector raw = gyroscope.readRaw();
 
  // Read normalized values in deg/sec
  Vector norm = gyroscope.readNormalize();


  if(norm.XAxis != 0 || norm.YAxis != 0 || norm.ZAxis != 0)
  {
    GyBtnState = HIGH;
    }
  else{
    GyBtnState = LOW;
  }

  if(GyBtnState == HIGH && GyLastState == LOW)  //有人触动
  {
    fadeAmount = 6*fadeAmount;     //呼吸加快
    GyLastState = GyBtnState;    //记录按键状态
    //Serial.println("touched");
  }
  else if(GyBtnState == LOW && GyLastState == HIGH){
    GyLastState = GyBtnState;    //记录按键状态
    fadeAmount = fadeAmount/6;     //呼吸减慢
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
  
  /*************RGB_LED_breathing************************************/
  // set brightness within the range of (redMin, redMax)
  if (redValue <redMin){
    redValue = redMin;
  }
  if (redValue >redMax){
    redValue = redMax;
  }

  if(n<4){
  greenValue = redValue;
  blueValue = redValue;    
  }
  else if(n>=4 && n <8){
  greenValue = redValue * 0.5;
  blueValue = redValue * 0.08;        
  }
  else{
  greenValue = redValue * 0.33;
  blueValue = redValue * 0.01;      
  }

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
  if (redValue <= redMin || redValue >= redMax) {
    fadeAmount = -fadeAmount;
  }


  Serial.println(redValue);
  Serial.println(greenValue);
  Serial.println(blueValue);
  
  Serial.println(n);
  
  // wait for 12 milliseconds to see the dimming effect
  delay(12);
}


//rgb function
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
