void System_Processes(){
  ///////////////// 风扇冷却 /////////////////
  if(enableFan==true){
    if(enableDynamicCooling==false){                                   //静态 PWM 冷却模式（2 针风扇 - 无需迟滞，温度数据仅在 'avgCountTS' 或每 500 个循环周期后刷新）
      if(overrideFan==true){fanStatus=true;}                           //强制风扇
      else if(temperature>=temperatureFan){fanStatus=1;}               //达到设定的风扇温度时打开风扇
      else if(temperature<temperatureFan){fanStatus=0;}                //达到设定的风扇温度时关闭风扇
      digitalWrite(FAN,fanStatus);                                     //向风扇 MOSFET 发送数字信号
    }
    else{}                                                             //动态 PWM 冷却模式（3 针风扇 - 即将推出）
  }
  else{digitalWrite(FAN,LOW);}                                         //风扇禁用
  
  //////////// 循环时间秒表 ////////////
  loopTimeStart = micros();                                            //记录开始时间
  loopTime = (loopTimeStart-loopTimeEnd)/1000.000;                     //计算循环周期速度 
  loopTimeEnd = micros();                                              //记录结束时间

  ///////////// 自动数据重置  /////////////
  if(telemCounterReset==0){}                                           //从不重置
  else if(telemCounterReset==1 && daysRunning>1)  {resetVariables();}  //每日重置
  else if(telemCounterReset==2 && daysRunning>7)  {resetVariables();}  //每周重置
  else if(telemCounterReset==3 && daysRunning>30) {resetVariables();}  //每月重置
  else if(telemCounterReset==4 && daysRunning>365){resetVariables();}  //每年重置
  ///////////// LOW POWER MODE /////////////
  if(lowPowerMode==1){}   
  else{}      
}

void factoryReset(){
  EEPROM.write(0,1);  //存储: 充电算法（1 = MPPT 模式）
  EEPROM.write(12,1); //存储: 充电器/PSU 模式选择（1 = 充电器模式）
  EEPROM.write(1,12); //存储: 最大电池电压（整体）
  EEPROM.write(2,0);  //存储: 最大电池电压（十进制）
  EEPROM.write(3,9);  //存储: 最小电池电压（整体）
  EEPROM.write(4,0);  //存储: 最小电池电压 （十进制）
  EEPROM.write(5,30); //存储: 充电电流（整体）)
  EEPROM.write(6,0);  //存储: 充电电流（十进制）
  EEPROM.write(7,1);  //存储: 风扇启用 (Bool)
  EEPROM.write(8,60); //存储: 风扇温度（整数）
  EEPROM.write(9,90); //存储: 关机温度（整数）
  EEPROM.write(10,1); //存储: 启用 WiFi（布尔值）
  EEPROM.write(11,1); //存储: 启用自动加载（默认开启）
  EEPROM.write(13,0); //存储: LCD 背光睡眠定时器（默认值：0 = 从不）
  EEPROM.commit();
  loadSettings();
}

void loadSettings(){ 
  MPPT_Mode          = EEPROM.read(0);                       // 加载保存的充电模式设置
  output_Mode        = EEPROM.read(12);                      // 加载保存的充电模式设置
  voltageBatteryMax  = EEPROM.read(1)+(EEPROM.read(2)*.01);  // 加载保存的最大电池电压设置
  voltageBatteryMin  = EEPROM.read(3)+(EEPROM.read(4)*.01);  // 加载保存的最低电池电压设置
  currentCharging    = EEPROM.read(5)+(EEPROM.read(6)*.01);  // 加载保存的充电电流设置
  enableFan          = EEPROM.read(7);                       // 加载保存的风扇启用设置
  temperatureFan     = EEPROM.read(8);                       // 加载保存的风扇温度设置
  temperatureMax     = EEPROM.read(9);                       // 加载保存的关机温度设置
  enableWiFi         = EEPROM.read(10);                      // 加载保存的 WiFi 启用设置  
  flashMemLoad       = EEPROM.read(11);                      // 加载保存的闪存自动加载功能
  backlightSleepMode = EEPROM.read(13);                      // 加载保存的液晶背光睡眠定时器
  }

void saveSettings(){
  EEPROM.write(0,MPPT_Mode);           //存储：算法 
  EEPROM.write(12,output_Mode);        //充电/PSU 模式选择 
  conv1 = voltageBatteryMax*100;       //最大电池电压 获取整数
  conv2 = conv1%100;                   //最大电池电压 获取十进制数并转换为整数
  EEPROM.write(1,voltageBatteryMax);
  EEPROM.write(2,conv2);
  conv1 = voltageBatteryMin*100;       //最小电池电压  获取整数
  conv2 = conv1%100;                   //最小电池电压 获取十进制数并转换为整数
  EEPROM.write(3,voltageBatteryMin);
  EEPROM.write(4,conv2);
  conv1 = currentCharging*100;         //充电电流
  conv2 = conv1%100;
  EEPROM.write(5,currentCharging);
  EEPROM.write(6,conv2);
  EEPROM.write(7,enableFan);           //风扇启用
  EEPROM.write(8,temperatureFan);      //风扇温度
  EEPROM.write(9,temperatureMax);      //关机温度
  EEPROM.write(10,enableWiFi);         //启用 WiFi
//EEPROM.write(11,flashMemLoad);       //启用自动加载（必须排除批量保存，酌情取消注释）
  EEPROM.write(13,backlightSleepMode); // STORE: LCD背光睡眠定时器
  EEPROM.commit();                     //将设置更改保存到闪存
}
void saveAutoloadSettings(){
  EEPROM.write(11,flashMemLoad);       //STORE: 启用自动加载
  EEPROM.commit();                     //将设置更改保存到闪存
}
void initializeFlashAutoload(){
  if(disableFlashAutoLoad==0){
    flashMemLoad = EEPROM.read(11);       //加载保存的自动加载（必须从批量保存中排除，酌情取消注释）
    if(flashMemLoad==1){loadSettings();}  //从闪存加载存储的设置
  } 
}
