/**************library***********************************************/
#include <SoftwareSerial.h> 
SoftwareSerial mySerial(13, 12); // RX, TX  通过软串口连接esp8266，

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

float rblue = 0.53;
float rgreen = 0.48;


//ir_sensor
int IrBtnState;          //读取当前的IR_sensor状态
int IrLastState = LOW;   //上一个IR_sensor状态
int IrPin = 8;           //定义IR_sensor输入引脚
int n = 0;               //计数
int m = 0;               //一个周期内的计数
int threshold = 8;       //推送一次的计数阈值


//wifi_module
String ssid ="Tanghownn";                                   //WIFI名称
String password="tangswifi";                                //WIFI密码
String uid = "87eb612d0fba5ba35bf0356389613cb4";             // 用户私钥，巴法云控制台获取
String type = "1";                                           // 1表示是预警消息，默认即可
String device = "ir_sensor";                                 // 设备名称
String msg = "There have been 30 people staying under the tree. 10 of them were...5 of them were...";                           //发送的消息
String msg2 = "Tree's breath";                               //消息备注，可为空
int delaytime = 0;                                           //贤者时间，单位秒。默认为0是立即发送，如果设置了该值，则响应秒数内再发送也不会通知


void setup() {
  // 声明引脚为输出或输入模式
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
  
  pinMode(IrPin, INPUT);
  Serial.begin(115200);


  /*********************set up wifi module******************************/
  // Open serial communications and wait for port to open:
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
  mySerial.begin(115200);
  mySerial.println("AT+RST");   // 初始化重启一次esp8266
  delay(1500);
  echo();
  mySerial.println("AT");
  echo();
  delay(500);
  mySerial.println("AT+CWMODE=3");  // 设置Wi-Fi模式
  echo();
  mySerial.println("AT+CWJAP=\"" +ssid+"\",\"" + password + "\"");  // 连接Wi-Fi
  echo();
  delay(10000);
}

void loop() {
  /********************IR_sensor***************************************/
  IrBtnState = digitalRead(IrPin);
  if(IrBtnState == HIGH && IrLastState == LOW)  //如果读入是高电平，说明有人进入区域
  {
    n = n+1;                     //计数一次
    m = m+1;                     //周期内计数一次
    IrLastState = IrBtnState;    //记录按键状态
    fadeAmount = 3*fadeAmount;     //呼吸加快
    rblue = rblue*0.75;           //感应8次变橙
    rgreen = rgreen*0.82;         //感应8次变橙
  }
  else if(IrBtnState == LOW && IrLastState == HIGH){
    IrLastState = IrBtnState;    //记录按键状态
    fadeAmount = fadeAmount/3;     //呼吸减慢
  }


  /*************RGB_LED_breathing************************************/  
  // set brightness within the range of (redMin, redMax)
  if (redValue <redMin){
    redValue = redMin;
  }
  if (redValue >redMax){
    redValue = redMax;
  }

  /*
  if(n<8){
  greenValue = redValue * 0.48;
  blueValue = redValue * 0.53;    
  }
  else if(n>=8 && n <16){
  greenValue = redValue * 0.2;
  blueValue = redValue * 0.08;        
  }
  else{
  greenValue = redValue * 0.29;
  blueValue = redValue * 0.01;      
  }
  */

  if(rblue >1.3){
    rblue = 1.3;
  }
  if(rblue <0.05){
    rblue = 0.05;
  }

  if(rgreen >0.85){
    rgreen = 0.85;
  }
  if(rgreen <0.1){
    rgreen = 0.1;
  }

  greenValue = redValue *rgreen;
  blueValue = redValue *rblue; 

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
  
  // wait for 6 milliseconds to see the dimming effect
  delay(6);
  
  
  /*******************wechat post**************************************/
  if(m >= threshold){
    if (mySerial.available()) {
      Serial.write(mySerial.read());
      post();
    }
    if (Serial.available()) {
      mySerial.write(Serial.read());
      post();
    }
    m = 0;
  }

  Serial.println(m);
}


//rgb function
void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

//wifi function
void echo(){
  delay(50);
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
 
void post(){
  String postData;
  //Post Data
  postData = "uid="+uid+"&type=" + type +"&time="+delaytime+"&device="+device+"&msg="+msg+"&msg2="+msg2;
  mySerial.println("AT+CIPMODE=1");
  echo();
  mySerial.println("AT+CIPSTART=\"TCP\",\"ai.bemfa.com\",80");  // 连接服务器的80端口
  delay(1000);
  echo();
  mySerial.println("AT+CIPSEND"); // 进入TCP透传模式，接下来发送的所有消息都会发送给服务器
  echo();
  mySerial.print("POST /api/wechat/v1/"); // 开始发送post请求
  mySerial.print(" HTTP/1.1\r\nHost: ai.bemfa.com\r\nContent-Type: application/x-www-form-urlencoded\r\nConnection:close\r\nContent-Length:"); // post请求的报文格式
  mySerial.print(postData.length()); // 需要计算post请求的数据长度
  mySerial.print("\r\n\r\n"); 
  mySerial.println(postData); // 结束post请求
  delay(3000);
  echo();
  mySerial.print("+++"); // 退出tcp透传模式，用println会出错
  delay(2000);
}
