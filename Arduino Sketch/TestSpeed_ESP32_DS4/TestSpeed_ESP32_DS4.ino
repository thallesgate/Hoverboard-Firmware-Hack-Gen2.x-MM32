//    Tested with Arduino Pro Mini 3.3V and Hoverboard-TX to pin 9 and Hoverboard-RX to pin 8
//
//    PB6 (Hoverboard-TX) and PB7 (Hoverboard-RX) can handle 5V I/O-Level :-)
//
#include <PS4Controller.h>
//    please share feedback to https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
#define _DEBUG      // debug output to first hardware serial port
//#define DEBUG_RX    // additional hoverboard-rx debug output

#define ESP32       // comment out if using Arduino
#define HOVERBOARD_MM32  // uncomment if a MM32 Hoverboard firmware is connected (and no GD32 or STM32)
#define REMOTE_UARTBUS  // one serial bus to control them all :-)
#define SEND_MILLIS 100   // send commands to hoverboard every SEND_MILLIS millisesonds

#include "util.h"
#include "hoverserial.h"

#define BAUDRATE 19200   // 19200 is default on hoverboard side because Arudino Nano SoftwareSerial can not do 115200
//const int pinRX = 39, pinTX = 37;   // Wemos S2 Mini
const int pinRX = 16, pinTX = 17;   // Wemos Lolin32
#define oSerialHover Serial1    // ESP32
#define l1DriverID 2
#define l2DriverID 4
#define r1DriverID 1
#define r2DriverID 3
SerialHover2Server oHoverFeedback;

#define REFRESH_RATE        30          // [Hz] Sending time interval
const long waitInterval = 1000 / REFRESH_RATE; // [ms] Wait time needed to achieve the desired refresh rate
unsigned long previousMillis = 0; // [ms] Used for the non-blocking delay


float linearCmd = 0;
float angularCmd = 0;
int stateCmd = 0;
int drivemodeCmd = 0;
int maxSpeed = 500;
bool enableSend = false;

#define LINEAR_LIMIT 4.0//0.170 //[M/s] -> 10 RPM +- 10m/minuto
#define ANGULAR_LIMIT 8.0 //1 //[RAD/s] -> 1 rotacao a cada 2 segundos
#define MOTOR_LIMIT 6.0 
float l_speed_cmd = 0.0;//Desired speed for left wheel in m/s
float r_speed_cmd = 0.0;//Desired speed for right wheel in m/s

int received_linear_cmd = 0;
int received_angular_cmd = 0;

#define WHEEL_RADIUS 0.080                 //160MM DIAMETER Wheel radius, in M
#define WHEEL_BASE 0.437               //Wheelbase, in M
const double wheel_circumference = WHEEL_RADIUS * PI * 2; 
// ########################## DS4 ##########################
// PS4 analog range: -+128
#define CONTROLLER_DEADZONE 10.0
unsigned long lastTimeStamp = 0;

uint8_t  wState = 4;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Ensure input is within bounds
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }
  
  // Calculate the mapped value
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void handleController()
{
  int lStickX = PS4.LStickY();
  int rStickY = PS4.RStickX();

  if(lStickX <= CONTROLLER_DEADZONE && lStickX >= (CONTROLLER_DEADZONE*-1.0)){
    linearCmd = 0.0f;
  }else{
    linearCmd = (float)lStickX;
  }
  if(rStickY <= CONTROLLER_DEADZONE && rStickY >= (CONTROLLER_DEADZONE*-1.0)){
    angularCmd = 0.0f;
  }else{
    angularCmd = (float)rStickY;
  }
  
  bool bSend = false;
  
  if(PS4.Left()){
    drivemodeCmd = 0;
    bSend = true;
  }else if(PS4.Up()){
    drivemodeCmd = 1;
    bSend = true;
  }else if(PS4.Right()){
    drivemodeCmd = 2;
    bSend = true;
  }else if(PS4.Down()){
    drivemodeCmd = 3;
    bSend = true;
  }
  
  if(bSend){
    oHoverConfig.iDriveMode = drivemodeCmd;
    oHoverConfig.iSlave = l1DriverID;
    HoverSendData(oSerialHover,oHoverConfig);
    HoverLogConfigMM32(oHoverConfig);

    oHoverConfig.iDriveMode = drivemodeCmd;
    oHoverConfig.iSlave = l2DriverID;
    HoverSendData(oSerialHover,oHoverConfig);
    HoverLogConfigMM32(oHoverConfig);
    
    oHoverConfig.iDriveMode = drivemodeCmd;
    oHoverConfig.iSlave = r1DriverID;
    HoverSendData(oSerialHover,oHoverConfig);
    HoverLogConfigMM32(oHoverConfig);

    oHoverConfig.iDriveMode = drivemodeCmd;
    oHoverConfig.iSlave = r2DriverID;
    HoverSendData(oSerialHover,oHoverConfig);
    HoverLogConfigMM32(oHoverConfig);
    bSend = false;
  }
  
  //PS4.LStickX(),
  //PS4.LStickY(),
  //PS4.RStickX(),
  //PS4.RStickY(),
  //PS4.Left(),
  //PS4.Down(),
  //PS4.Right(),
  //PS4.Up(),
  //PS4.Square(),
  //PS4.Cross(),
  //PS4.Circle(),
  //PS4.Triangle(),
  //PS4.L1(),
  //PS4.R1(),
  //PS4.L2(),
  //PS4.R2(),  
  //PS4.Share(),
  //PS4.Options(),
  //PS4.PSButton(),
  //PS4.Touchpad(),
  //PS4.Charging(),
  //PS4.Audio(),
  //PS4.Mic(),
  //PS4.Battery());

  if (millis() - lastTimeStamp > 50)
  {
    #ifdef _DEBUG
    //Serial.println("Enabled: " + String(enableSend) + " linearCmd: " + String(linearCmd) + " angularCmd: " + String(angularCmd));
    #endif
    lastTimeStamp = millis();
  }
}

void handleMotorCmd(float linearCmd, float angularCmd, float linearLimit, float angularLimit){
  //0-256

  double linear_velocity_x_req = 0.0;
  double angular_velocity_z_req = 0.0;
  
  linear_velocity_x_req = mapFloat(linearCmd, -128.0f, 128.0f, LINEAR_LIMIT*-1.0, LINEAR_LIMIT);
  angular_velocity_z_req = mapFloat(angularCmd, -128.0f, 128.0f, ANGULAR_LIMIT*-1.0, ANGULAR_LIMIT);

  double l_speed_req = linear_velocity_x_req - angular_velocity_z_req*(WHEEL_BASE/2.0);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  double r_speed_req = linear_velocity_x_req + angular_velocity_z_req*(WHEEL_BASE/2.0);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  //Serial.println(" | linear_velocity: " + String(linear_velocity_x_req) + " angular_velocity: " + String(angular_velocity_z_req) + " l_speed_req: " + String(l_speed_req) + " r_speed_req: " + String(r_speed_req));
  //l_speed_cmd = (l_speed_req/WHEEL_RADIUS) / (2.0*PI); // Converting m/s to Rad/s to Rev/s * 60 to RPM
  //r_speed_cmd = (r_speed_req/WHEEL_RADIUS) / (2.0*PI); // Converting m/s to Rad/s to Rev/s * 60 to RPM

  l_speed_cmd = (l_speed_req / wheel_circumference) * 60.0;
  r_speed_cmd = (r_speed_req / wheel_circumference) * 60.0;

}

void onConnect()
{
  #ifdef _DEBUG
  Serial.println("Connected!. Enabling.");
  #endif
  enableSend = true;
}

void onDisConnect()
{
  #ifdef _DEBUG
  Serial.println("Disconnected!. Disabling.");
  #endif
  enableSend = false; 
}

void setup()
{
  delay(500);
  #ifdef _DEBUG
    Serial.begin(115200);
    Serial.println("Hello Hoverbaord Gen2.target.board :-)");
  #endif
  PS4.attach(handleController);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  
  // Serial interface, baud, RX GPIO, TX GPIO
  // Note: The GPIO numbers will not necessarily correspond to the
  // pin number printed on the PCB. Refer to your ESP32 documentation for pin to GPIO mappings.
  HoverSetupEsp32(oSerialHover,BAUDRATE,pinRX,pinTX); 

  pinMode(LED_BUILTIN, OUTPUT);
}

uint8_t  iSendId = 0;   // only for UartBus
int iLog = -1;  // -1: print log of all slaves, 0: only print log of slaveId 0

int reqSpeed = 0;

void CheckConsoleMM32()
{
    if (!Serial.available())  // if there is terminal data comming from user
    {return;}
    
  String sReceived = Serial.readStringUntil('\n');
  Serial.println(sReceived);
  boolean bSend = false;
  int  iSendTo = -1;
  String sCmd = ShiftValue(sReceived, " ");
  if (isUInt(sCmd))
  {
    iSendTo = sCmd.toInt();
    Serial.print(iSendTo);Serial.print("\t");
    sCmd = ShiftValue(sReceived, " ");
    
  }
  
  if ( (sCmd == "m") || (sCmd == "mode"))
  {
    oHoverConfig.iDriveMode = ShiftValue(sReceived, "\n").toInt();
    bSend = true;
  }
  else if ( (sCmd == "bl") || (sCmd == "batlow"))
  {
    oHoverConfig.fBattEmpty = ShiftValue(sReceived, "\n").toFloat();
    bSend = true;
  }
  else if ( (sCmd == "bh") || (sCmd == "bathi"))
  {
    oHoverConfig.fBattFull = ShiftValue(sReceived, "\n").toFloat();
    bSend = true;
  }
  else if ( (sCmd == "si") || (sCmd == "slave"))
  {
    oHoverConfig.iSlaveNew = ShiftValue(sReceived, "\n").toInt();
    bSend = true;
  }
  else if ( (sCmd == "l") || (sCmd == "log"))
  {
    iLog = ShiftValue(sReceived, "\n").toInt();
  }
  else
  {
    Serial.print("unkown command: "); 
  }
  Serial.print(sCmd); Serial.print("\t value:"); Serial.println(ShiftValue(sReceived, "\n"));

  if (bSend)
  {
    for (int iTo=1; iTo<=4; iTo++)
    {
      if (  (iSendTo<0) || (iSendTo == iTo)  )
      {
        oHoverConfig.iSlave = iTo;
        HoverSendData(oSerialHover,oHoverConfig);
        HoverLogConfigMM32(oHoverConfig);
      }
    }
  }

}

unsigned long iLast = 0;
unsigned long iNext = 0;
unsigned long iTimeNextState = 3000;

void loop()
{
  unsigned long iNow = millis();
  digitalWrite(LED_BUILTIN, (iNow%2000) < 500);
  //digitalWrite(39, (iNow%500) < 250);
  //digitalWrite(37, (iNow%500) < 100);

  //int iSpeed = 3 * (ABS( (int)((iNow/20+100) % 400) - 200) - 100);   // repeats from +300 to -300 to +300 :-)
  //int iSteer = 1 * (ABS( (int)((iNow/400+100) % 400) - 200) - 100);   // repeats from +100 to -100 to +100 :-)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= waitInterval) {
    previousMillis = currentMillis;
    //Non-blocking Task
      handleController();
    if(enableSend){
      handleMotorCmd(linearCmd, angularCmd, LINEAR_LIMIT, ANGULAR_LIMIT);
    }else{
      handleMotorCmd(0.0, 0.0, LINEAR_LIMIT, ANGULAR_LIMIT);
    }
  }
  if (iNow > iTimeNextState)
  {
    iTimeNextState = iNow + 3000;
    wState = wState << 1;
    if (wState == 64) wState = 1;  // remove this line to test Shutoff = 128
  }
  
  boolean bReceived;   
  while (bReceived = Receive(oSerialHover,oHoverFeedback))
  {
    #ifdef REMOTE_UARTBUS
      if (  (iLog < 0) || (iLog == oHoverFeedback.iSlave)  )
    #endif
    {
      DEBUGT("millis",iNow-iLast);
      //DEBUGT("lSpeed",l_speed_cmd);
      //DEBUGT("rSpeed",r_speed_cmd);
      HoverLog(oHoverFeedback);
      iLast = iNow;
    }
  }

  if (iNow > iNext)
  {
    //DEBUGN("time",iNow)
    //Serial.print("L: ");
    //Serial.print(l_speed_cmd);
    //Serial.print(" R: ");
    //Serial.println(r_speed_cmd);
    #ifdef REMOTE_UARTBUS
      
      switch(iSendId++)
      {
      case 0: // left motor
        //HoverSend(oSerialHover,0,CLAMP(iSpeed + iSteer,-1000,1000),wState);  // hoverboard will answer immediatly on having received this message ...
        HoverSend(oSerialHover,l1DriverID,CLAMP(l_speed_cmd,maxSpeed*-1,maxSpeed),wState);  // hoverboard will answer immediatly on having received this message ...
        break;
      case 1: // left motor
        //HoverSend(oSerialHover,0,CLAMP(iSpeed + iSteer,-1000,1000),wState);  // hoverboard will answer immediatly on having received this message ...
        HoverSend(oSerialHover,l2DriverID,CLAMP(l_speed_cmd,maxSpeed*-1,maxSpeed),wState);  // hoverboard will answer immediatly on having received this message ...
        break;
      case 2: // right motor
        //HoverSend(oSerialHover,1,-CLAMP(iSpeed - iSteer,-1000,1000),wState);  // hoverboard will answer immediatly on having received this message ...
        HoverSend(oSerialHover,r1DriverID,-CLAMP(r_speed_cmd,maxSpeed*-1,maxSpeed),wState);  // hoverboard will answer immediatly on having received this message ...
        break;
      case 3: // right motor
        //HoverSend(oSerialHover,1,-CLAMP(iSpeed - iSteer,-1000,1000),wState);  // hoverboard will answer immediatly on having received this message ...
        HoverSend(oSerialHover,r2DriverID,-CLAMP(r_speed_cmd,maxSpeed*-1,maxSpeed),wState);  // hoverboard will answer immediatly on having received this message ...
        iSendId = 0;
        break;
      }
      iNext = iNow + SEND_MILLIS/2;
      
      CheckConsoleMM32();
    #endif

  }

}
