/*  PROJECT FUGU FIRMWARE V1.10  (DIY 1kW Open Source MPPT Solar Charge Controller)
 *  By: TechBuilder (Angelo Casimiro)
 *  FIRMWARE STATUS: Verified Stable Build Version
 *  (Contact me for the experimental beta versions)
 *  -----------------------------------------------------------------------------------------------------------
 *  DATE CREATED:  02/07/2021 
 *  DATE MODIFIED: 30/08/2021
 *  -----------------------------------------------------------------------------------------------------------
 *  CONTACTS:
 *  GitHub - www.github.com/AngeloCasi (New firmware releases will only be available on GitHub Link)
 *  Email - casithebuilder@gmail.com
 *  YouTube - www.youtube.com/TechBuilder
 *  Facebook - www.facebook.com/AngeloCasii
 *  -----------------------------------------------------------------------------------------------------------
 *  PROGRAM FEATURES:
 *  - MPPT Perturbed Algorithm With CC-CV
 *  - WiFi & Bluetooth BLE Blynk Phone App Telemetry
 *  - Selectable Charger/PSU Mode (can operate as a programmable buck converter)
 *  - Dual Core ESP32 Unlocked (using xTaskCreatePinnedToCore(); )
 *  - Precision ADC Tracking Auto ADS1115/ADS1015 Detection (16bit/12bit I2C ADC)
 *  - Automatic ACS712-30A Current Sensor Calibration
 *  - Equipped With Battery Disconnect & Input Disconnect Recovery Protection Protocol
 *  - LCD Menu Interface (with settings & 4 display layouts)
 *  - Flash Memory (non-volatile settings save function)
 *  - Settable PWM Resolution (8bit-16bit)
 *  - Settable PWM Switching Frequency (1.2kHz - 312kHz)
 *  -----------------------------------------------------------------------------------------------------------
 *  PROGRAM INSTRUCTIONS:
 *  1.) Watch YouTube video tutorial first before using
 *  2.) Install the required Arduino libraries for the ICs
 *  3.) Select Tools > ESP32 Dev Board Board 
 *  4.) Do not modify code unless you know what you are doing
 *  5.) The MPPT's synchronous buck converter topology is code dependent, messing with the algorithm
 *      and safety protection protocols can be extremely dangerous especially when dealing with HVDC.
 *  6.) Install Blynk Legacy to access the phone app telemetry feature
 *  7.) Input the Blynk authentication in this program token sent by Blynk to your email after registration
 *  8.) Input WiFi SSID and password in this program
 *  9.) When using WiFi only mode, change "disableFlashAutoLoad = 0" to = 1 (LCD and buttons not installed)
 *      this prevents the MPPT unit to load the Flash Memory saved settings and will load the Arduino variable
 *      declarations set below instead
 *  -----------------------------------------------------------------------------------------------------------
 *  GOOGLE DRIVE PROJECT LINK: coming soon
 *  INSTRUCTABLE TUTORIAL LINK: coming soon
 *  YOUTUBE TUTORIAL LINK: www.youtube.com/watch?v=ShXNJM6uHLM
 *  GITHUB UPDATED FUGU FIRMWARE LINK: github.com/AngeloCasi/FUGU-ARDUINO-MPPT-FIRMWARE
 *  -----------------------------------------------------------------------------------------------------------
 *  ACTIVE CHIPS USED IN FIRMWARE:
 *  - ESP32 WROOM32
 *  - ADS1115/ADS1015 I2C ADC
 *  - ACS712-30A Current Sensor IC
 *  - IR2104 MOSFET Driver
 *  - CH340C USB TO UART IC
 *  - 16X2 I2C Character LCD

 *  OTHER CHIPS USED IN PROJECT:
 *  - XL7005A 80V 0.4A Buck Regulator (2x)
 *  - AMS1115-3.3 LDO Linear Regulator 
 *  - AMS1115-5.0 LDO Linear Regulator  
 *  - CSD19505 N-ch MOSFETS (3x)
 *  - B1212 DC-DC Isolated Converter
 *  - SS310 Diodes
 */
//================================ MPPT FIRMWARE LCD MENU INFO =====================================//
// 下面几行是 MPPT 的 LCD 菜单界面上显示的固件版本信息     //
//==================================================================================================//
String 
firmwareInfo      = "V1.10   ",
firmwareDate      = "30/08/21",
firmwareContactR1 = "www.youtube.com/",  
firmwareContactR2 = "TechBuilder     ";        
           
//====================== ARDUINO LIBRARIES (ESP32 Compatible Libraries) ============================//
// 您必须下载并安装以下库才能对 MPPT 进行编程 //
// 以获取“MPPT”教程                             //
//============================================================================================= ====//
#include <EEPROM.h>                 //系统参数 - EEPROM 库（作者：Arduino）
#include <Wire.h>                   //系统参数 - WIRE 库（作者：Arduino）
#include <SPI.h>                    //系统参数 - SPI 库（作者：Arduino）
#include <WiFi.h>                   //系统参数 - WiFi 库（作者：Arduino）
#include <WiFiClient.h>             //系统参数 - WiFi 库（作者：Arduino）
#include <BlynkSimpleEsp32.h>       //系统参数 - 手机应用程序的 Blynk WiFi 库 
#include <LiquidCrystal_I2C.h>      //系统参数 - ESP32 LCD 兼容库（作者：Robojax）
#include <Adafruit_ADS1X15.h>       //系统参数 - ADS1115/ADS1015 ADC 库（作者：Adafruit）
LiquidCrystal_I2C lcd(0x27,16,2);   //系统参数 - 配置 LCD RowCol 大小和 I2C 地址
TaskHandle_t Core2;                 //系统参数 - 用于 ESP32 双核操作
Adafruit_ADS1015 ads;               //系统参数 - ADS1015 ADC 库（作者：Adafruit）如果您使用的是 ADS1115，请删除此行
//Adafruit_ADS1115 ads;             //系统参数 - ADS1115 ADC 库（作者：Adafruit）如果您使用的是 ADS1115，请取消注释

//======================================  用户参数 ===========================================//
//下面的参数是没有MPPT充电器设置时使用的默认参数 //
//通过 LCD 菜单界面或手机 WiFi 应用程序设置或保存。这里的一些参数//
//将允许您覆盖或解锁高级用户的功能（不在 LCD 菜单上的设置）//
//==================================================================================================//
#define backflow_MOSFET 27          //系统参数 -回流 MOSFET 
#define buck_IN         33          //系统参数 - 降压 MOSFET 驱动器 PWM引脚  
#define buck_EN         32          //系统参数 - 降压 MOSFET 驱动器使能引脚
#define LED             2           //系统参数 - LED指示灯 GPIO 引脚 
#define FAN             16          //系统参数 -风扇 GPIO 引脚
#define ADC_ALERT       34          //系统参数 - 风扇 GPIO 引脚
#define TempSensor      35          //系统参数 - 温度传感器 GPIO 引脚
#define buttonLeft      18          //系统参数 - LCD 左键 GPIO 引脚
#define buttonRight     19          //系统参数 - LCD 右键 GPIO 引脚
#define buttonBack      17          //系统参数 - LCD 返回 GPIO 引脚
#define buttonSelect    23          //系统参数 - LCD 选择 GPIO 引脚

//========================================= WiFi SSID ==============================================//
//此 MPPT 固件使用 Blynk 手机应用程序和 arduino 库进行控制和数据遥测 //
//填写您的 WiFi SSID 和密码。您还必须获得自己的身份验证令牌 //
//从 Blynk 平台注册后的电子邮件。//
//==================================================================================================//
char 
auth[] = "RY8WPkFrp_eJN9F5pK2WkwhjpF0YNKxN",    //   用户参数 - 输入 Blynk 身份验证令牌
ssid[] = "Gao",                                //   用户参数 - 输入您的 WiFi SSID
pass[] = "13145211";                          //   用户参数 - 输入您的 WiFi 密码

//====================================== 用户参数 ==========================================//
//下面的参数是没有MPPT充电器设置时使用的默认参数 //
//通过 LCD 菜单界面或手机 WiFi 应用程序设置或保存。这里的一些参数//
//将允许您覆盖或解锁高级用户的功能（不在 LCD 菜单上的设置）//
//=================================================================================================//
bool                                  
MPPT_Mode               = 1,           //   USER PARAMETER - 启用 MPPT 算法，当禁用充电器时使用 CC-CV 算法
output_Mode             = 1,           //   USER PARAMETER - 0 = PSU 模式, 1 = 充电器模式
disableFlashAutoLoad    = 0,           //   USER PARAMETER - 强制 MPPT 不使用闪存保存的设置，启用此“1”默认为已编程的固件设置
enablePPWM              = 1,           //   USER PARAMETER - 启用预测 PWM，这加快了调节速度（仅适用于电池充电应用）
enableWiFi              = 1,           //   USER PARAMETER - 启用 WiFi 连接
enableFan               = 1,           //   USER PARAMETER - 启用冷却风扇
enableBluetooth         = 1,           //   USER PARAMETER - 启用蓝牙连
enableLCD               = 1,           //   USER PARAMETER - 启用 接LCD 显示
enableLCDBacklight      = 1,           //   USER PARAMETER - 启用 LCD 显示器的背光
overrideFan             = 0,           //   USER PARAMETER - 风扇始终开启
enableDynamicCooling    = 0;           //   USER PARAMETER - 启用 PWM 冷却控制 
int
serialTelemMode         = 1,           //  USER PARAMETER - 选择串行遥测数据馈送（0 - 禁用串行，1 - 显示所有数据，2 - 显示基本，3 - 仅数字）
pwmResolution           = 11,          //  USER PARAMETER - PWM 位分辨率 
pwmFrequency            = 39000,       //  USER PARAMETER - PWM 开关频率 - Hz（用于降压）
temperatureFan          = 60,          //  USER PARAMETER - 风扇开启的温度阈值
temperatureMax          = 90,          //  USER PARAMETER - 过热，超过时系统关闭（摄氏度）
telemCounterReset       = 0,           //  USER PARAMETER - 每隔一次重置 Telem 数据（0 = 从不，1 = 日，2 = 周，3 = 月，4 = 年）
errorTimeLimit          = 1000,        //  USER PARAMETER - 重置错误计数器的时间间隔（毫秒）
errorCountLimit         = 5,           //  USER PARAMETER - 最大错误数 
millisRoutineInterval   = 250,         //  USER PARAMETER - 例程函数的时间间隔刷新率 (ms)
millisSerialInterval    = 5,           //  USER PARAMETER - USB 串行数据馈送的时间间隔刷新率 (ms)
millisLCDInterval       = 1000,        //  USER PARAMETER - LCD 显示器的时间间隔刷新率 (ms)
millisWiFiInterval      = 1000,        //  USER PARAMETER - WiFi 遥测的时间间隔刷新率 (ms)
millisLCDBackLInterval  = 1000,        //  USER PARAMETER - 用户参数 - WiFi 遥测的时间间隔刷新率 (ms)
backlightSleepMode      = 0,           //  USER PARAMETER - - 0 = 从不, 1 = 10 秒, 2 = 5 分钟, 3 = 1 小时, 4 = 6 小时, 5 = 12 小时, 6 = 1 天, 7 = 3 天, 8 = 1 周, 9 = 1个月
baudRate                = 500000;      //  用户参数 - USB 串行波特率 (bps)

float 
voltageBatteryMax       = 12.6000,     //   USER PARAMETER - 最大电池充电电压（输出 V）
voltageBatteryMin       = 9.1000,     //   USER PARAMETER - 最小电池充电电压（输出 V）
currentCharging         = 30.0000,     //   USER PARAMETER - 最大充电电流（A - 输出）
electricalPrice         = 0.6500;      //   USER PARAMETER - 每千瓦时的输入电价（美元/千瓦时，欧元/千瓦时，比索/千瓦时）


//================================== 校准参数 =======================================//
//可以调整以下参数以设计您自己的 MPPT 充电控制器。只修改 //
//如果你知道你在做什么，下面的值。以下值已针对 // 进行了预校准
// TechBuilder (Angelo S. Casimiro) 设计的 MPPT 充电控制器 //
//=================================================================================================//
bool
ADS1015_Mode            = 1;          //  校准参数 - 对于 ADS1015 ADC 模型使用 1，对于 ADS1115 ADC 模型使用 0
int
ADC_GainSelect          = 2,          //  校准参数 - ADC 增益选择 (0→±6.144V 3mV/bit, 1→±4.096V 2mV/bit, 2→±2.048V 1mV/bit)
avgCountVS              = 3,          //  校准参数 - 电压传感器平均采样计数（推荐：3）
avgCountCS              = 4,          //  校准参数 - 电流传感器平均采样计数（推荐：4）
avgCountTS              = 500;        //  校准参数 - 温度传感器平均采样计数
float
inVoltageDivRatio       = 40.2156,    //  校准参数 - 输入分压器传感器比率（更改此值以校准电压传感器）
outVoltageDivRatio      = 40.2156,    //  校准参数 - 输出分压器传感器比率（更改此值以校准电压传感器）
vOutSystemMax           = 80.0000,    //  校准参数 - 最大输入电压
cOutSystemMax           = 50.0000,    //  校准参数 - 最大输出电压
ntcResistance           = 10000.00,    //  校准参数 - NTC 温度传感器的电阻。如果您使用 10k NTC，请更改为 10000.00
voltageDropout          = 1.0000,     //  校准参数 - 降压稳压器的压降电压（由于最大占空比限制而存在 DOV）
voltageBatteryThresh    = 1.5000,     //  校准参数 - 达到此电压时断电（输出 V）
currentInAbsolute       = 31.0000,    //  校准参数 - 系统可以处理的最大输入电流（A - 输入）


currentOutAbsolute      = 50.0000,    //  校准参数 - 系统可以处理的最大输出电流（A - 输入）
PPWM_margin             = 99.5000,    //  校准参数 - 预测 PWM 的最小工作占空比 (%)
PWM_MaxDC               = 97.0000,    //  校准参数 - 最大工作占空比 (%) 90%-97% 是好的
efficiencyRate          = 1.0000,     //  校准参数 - 理论降压效率（十进制百分比）
currentMidPoint         = 2.5250,     //  校准参数 - 电流传感器中点 (V)
currentSens             = 0.0000,     //  校准参数 - 电流传感器灵敏度 (V/A)
currentSensV            = 0.0330,     //  校准参数 - 电流传感器灵敏度 (mV/A)
vInSystemMin            = 8.000;     //  校准参数 - 系统识别最低电压

//===================================== 电流传感器灵敏度 (V/A) =========================================//
//不要更改本节中的参数值。下面的值是系统使用的变量 //
//进程。更改值可能会损坏 MPPT 硬件。请保持原样！然而， //
//您可以访问这些变量来获取您的模组所需的数据。//
//=================================================================================================//
bool
buckEnable            = 0,           // SYSTEM PARAMETER - 降压启用状态
fanStatus             = 0,           // SYSTEM PARAMETER - 风扇活动状态（1 = 开，0 = 关）
bypassEnable          = 0,           // SYSTEM PARAMETER - 
chargingPause         = 0,           // SYSTEM PARAMETER - 
lowPowerMode          = 0,           // SYSTEM PARAMETER - 
buttonRightStatus     = 0,           // SYSTEM PARAMETER -
buttonLeftStatus      = 0,           // SYSTEM PARAMETER - 
buttonBackStatus      = 0,           // SYSTEM PARAMETER - 
buttonSelectStatus    = 0,           // SYSTEM PARAMETER -
buttonRightCommand    = 0,           // SYSTEM PARAMETER - 
buttonLeftCommand     = 0,           // SYSTEM PARAMETER - 
buttonBackCommand     = 0,           // SYSTEM PARAMETER - 
buttonSelectCommand   = 0,           // SYSTEM PARAMETER -
settingMode           = 0,           // SYSTEM PARAMETER -
setMenuPage           = 0,           // SYSTEM PARAMETER -
boolTemp              = 0,           // SYSTEM PARAMETER -
flashMemLoad          = 0,           // SYSTEM PARAMETER -  
confirmationMenu      = 0,           // SYSTEM PARAMETER -      
WIFI                  = 0,           // SYSTEM PARAMETER - 
BNC                   = 0,           // SYSTEM PARAMETER -  
REC                   = 0,           // SYSTEM PARAMETER - 
FLV                   = 0,           // SYSTEM PARAMETER - 
IUV                   = 0,           // SYSTEM PARAMETER - 
IOV                   = 0,           // SYSTEM PARAMETER - 
IOC                   = 0,           // SYSTEM PARAMETER - 
OUV                   = 0,           // SYSTEM PARAMETER - 
OOV                   = 0,           // SYSTEM PARAMETER - 
OOC                   = 0,           // SYSTEM PARAMETER - 
OTE                   = 0;           // SYSTEM PARAMETER - 
int
inputSource           = 0,           // SYSTEM PARAMETER - 0 = MPPT 没有电源，1 = MPPT 使用太阳能作为电源，2 = MPPT 使用电池作为电源
avgStoreTS            = 0,           // SYSTEM PARAMETER - 温度传感器使用非侵入式平均，这是用于平均平均的累加器
temperature           = 0,           // SYSTEM PARAMETER -
sampleStoreTS         = 0,           // SYSTEM PARAMETER - TS AVG 第 n 个样本
pwmMax                = 0,           // SYSTEM PARAMETER -
pwmMaxLimited         = 0,           // SYSTEM PARAMETER -
PWM                   = 0,           // SYSTEM PARAMETER -
PPWM                  = 0,           // SYSTEM PARAMETER -
pwmChannel            = 0,           // SYSTEM PARAMETER -
batteryPercent        = 0,           // SYSTEM PARAMETER -
errorCount            = 0,           // SYSTEM PARAMETER -
menuPage              = 0,           // SYSTEM PARAMETER -
subMenuPage           = 0,           // SYSTEM PARAMETER -
ERR                   = 0,           // SYSTEM PARAMETER - 
conv1                 = 0,           // SYSTEM PARAMETER -
conv2                 = 0,           // SYSTEM PARAMETER -
intTemp               = 0;           // SYSTEM PARAMETER -
float
VSI                   = 0.0000,      // SYSTEM PARAMETER - 原始输入电压传感器 ADC 电压
VSO                   = 0.0000,      // SYSTEM PARAMETER - 原始输出电压传感器 ADC 电压
CSI                   = 0.0000,      // SYSTEM PARAMETER - 原始电流传感器 ADC 电压
CSI_converted         = 0.0000,      // SYSTEM PARAMETER - 实际电流传感器 ADC 电压
TS                    = 0.0000,      // SYSTEM PARAMETER - 原始温度传感器 ADC 值
powerInput            = 0.0000,      // SYSTEM PARAMETER - 输入功率（太阳能）以瓦特为单位
powerInputPrev        = 0.0000,      // SYSTEM PARAMETER - 先前存储的 MPPT 算法的输入功率变量（瓦特）
powerOutput           = 0.0000,      // SYSTEM PARAMETER - 输出功率（电池或充电功率，以瓦特为单位）
energySavings         = 0.0000,      // SYSTEM PARAMETER - 法定货币（比索、美元、欧元等）的能源节约
voltageInput          = 0.0000,      // SYSTEM PARAMETER - 输入电压（太阳能电压）
voltageInputPrev      = 0.0000,      // SYSTEM PARAMETER - 先前存储的 MPPT 算法的输入电压变量
voltageOutput         = 0.0000,      // SYSTEM PARAMETER - 输入电压（电池电压）
currentInput          = 0.0000,      // SYSTEM PARAMETER - 输出功率（电池或充电电压）
currentOutput         = 0.0000,      // SYSTEM PARAMETER - 输出电流（电池或充电电流，以安培为单位）
TSlog                 = 0.0000,      // SYSTEM PARAMETER -  NTC 热敏电阻热感应代码的一部分
ADC_BitReso           = 0.0000,      // SYSTEM PARAMETER - 系统检测 ADS1015/ADS1115 ADC 的适当位分辨率因子
daysRunning           = 0.0000,      // SYSTEM PARAMETER - 存储 MPPT 设备自上次通电以来运行的总天数
Wh                    = 0.0000,      // SYSTEM PARAMETER - 存储收集到的累积能量（瓦特小时）
kWh                   = 0.0000,      // SYSTEM PARAMETER - 存储收集到的累积能量（千瓦时）
MWh                   = 0.0000,      // SYSTEM PARAMETER - 存储收集到的累积能量（兆瓦时）
loopTime              = 0.0000,      // SYSTEM PARAMETER -
outputDeviation       = 0.0000,      // SYSTEM PARAMETER - 输出电压偏差 (%)
buckEfficiency        = 0.0000,      // SYSTEM PARAMETER - 测量降压转换器功率转换效率（仅适用于我的双电流传感器版本）
floatTemp             = 0.0000,
vOutSystemMin         = 0.0000;     //  CALIB PARAMETER - 
unsigned long 
currentErrorMillis    = 0,           //SYSTEM PARAMETER -
currentButtonMillis   = 0,           //SYSTEM PARAMETER -
currentSerialMillis   = 0,           //SYSTEM PARAMETER -
currentRoutineMillis  = 0,           //SYSTEM PARAMETER -
currentLCDMillis      = 0,           //SYSTEM PARAMETER - 
currentLCDBackLMillis = 0,           //SYSTEM PARAMETER - 
currentWiFiMillis     = 0,           //SYSTEM PARAMETER - 
currentMenuSetMillis  = 0,           //SYSTEM PARAMETER - 
prevButtonMillis      = 0,           //SYSTEM PARAMETER -
prevSerialMillis      = 0,           //SYSTEM PARAMETER -
prevRoutineMillis     = 0,           //SYSTEM PARAMETER -
prevErrorMillis       = 0,           //SYSTEM PARAMETER -
prevWiFiMillis        = 0,           //SYSTEM PARAMETER -
prevLCDMillis         = 0,           //SYSTEM PARAMETER -
prevLCDBackLMillis    = 0,           //SYSTEM PARAMETER -
timeOn                = 0,           //SYSTEM PARAMETER -
loopTimeStart         = 0,           //SYSTEM PARAMETER - 用于循环循环秒表，记录循环开始时间
loopTimeEnd           = 0,           //SYSTEM PARAMETER - 用于循环循环秒表，记录循环结束时间
secondsElapsed        = 0;           //SYSTEM PARAMETER - 

//====================================== 主程序 =============================================//
// The codes below contain all the system processes for the MPPT firmware. Most of them are called //
// from the 8 .ino tabs. The codes are too long, Arduino tabs helped me a lot in organizing them.  //
// The firmware runs on two cores of the Arduino ESP32 as seen on the two separate pairs of void   //
// setups and loops. The xTaskCreatePinnedToCore() freeRTOS function allows you to access the      //
// unused ESP32 core through Arduino. Yes it does multicore processes simultaneously!              // 
//=================================================================================================//

//================= CORE0: SETUP (DUAL CORE MODE) =====================//
void coreTwo(void * pvParameters){
 setupWiFi();                                              //TAB#7 - WiFi Initialization
//================= CORE0: LOOP (DUAL CORE MODE) ======================//
  while(1){
    Wireless_Telemetry();                                   //TAB#7 - Wireless telemetry (WiFi & Bluetooth)
    
}}
//================== CORE1: SETUP (DUAL CORE MODE) ====================//
void setup() { 
  
  //串行初始化           
  Serial.begin(baudRate);                                   //Set serial baud rate
  Serial.println("> Serial Initialized");                   //Startup message
  
  // GPIO 引脚初始化
  pinMode(backflow_MOSFET,OUTPUT);                          
  pinMode(buck_EN,OUTPUT);
  pinMode(LED,OUTPUT); 
  pinMode(FAN,OUTPUT);
  pinMode(TS,INPUT); 
  pinMode(ADC_ALERT,INPUT);
  pinMode(buttonLeft,INPUT); 
  pinMode(buttonRight,INPUT); 
  pinMode(buttonBack,INPUT); 
  pinMode(buttonSelect,INPUT); 
  
  //PWM INITIALIZATION
  ledcSetup(pwmChannel,pwmFrequency,pwmResolution);          //Set PWM Parameters
  ledcAttachPin(buck_IN, pwmChannel);                        //Set pin as PWM
  ledcWrite(pwmChannel,PWM);                                 //Write PWM value at startup (duty = 0)
  pwmMax = pow(2,pwmResolution)-1;                           //Get PWM Max Bit Ceiling
  pwmMaxLimited = (PWM_MaxDC*pwmMax)/100.000;                //Get maximum PWM Duty Cycle (pwm limiting protection)
  
  //ADC INITIALIZATION
  ADC_SetGain();                                             //Sets ADC Gain & Range
  ads.begin();                                               //Initialize ADC

  //GPIO INITIALIZATION                          
  buck_Disable();

  //ENABLE DUAL CORE MULTITASKING
  xTaskCreatePinnedToCore(coreTwo,"coreTwo",10000,NULL,0,&Core2,0);
  
  //INITIALIZE AND LIOAD FLASH MEMORY DATA
  EEPROM.begin(512);
  Serial.println("> FLASH MEMORY: STORAGE INITIALIZED");  //Startup message 
  initializeFlashAutoload();                              //Load stored settings from flash memory       
  Serial.println("> FLASH MEMORY: SAVED DATA LOADED");    //Startup message 

  //LCD INITIALIZATION
  if(enableLCD==1){
    lcd.begin();
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);
    lcd.print("MPPT INITIALIZED");
    lcd.setCursor(0,1);
    lcd.print("FIRMWARE ");
    lcd.print(firmwareInfo);    
    delay(1500);
    lcd.clear();
  }

  //SETUP FINISHED
  Serial.println("> MPPT HAS INITIALIZED");                //Startup message

}
//================== CORE1: LOOP (DUAL CORE MODE) ======================//
void loop() {
  Read_Sensors();         //TAB#2 - Sensor data measurement and computation
  Device_Protection();    //TAB#3 - Fault detection algorithm  
  System_Processes();     //TAB#4 - Routine system processes 
  Charging_Algorithm();   //TAB#5 - Battery Charging Algorithm                    
  Onboard_Telemetry();    //TAB#6 - Onboard telemetry (USB & Serial Telemetry)
  LCD_Menu();             //TAB#8 - Low Power Algorithm
}
