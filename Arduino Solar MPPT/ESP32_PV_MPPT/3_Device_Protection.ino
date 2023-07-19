void backflowControl(){                                                // PV 回流控制（输入 MOSFET 
  if(output_Mode==0){bypassEnable=1;}                                  //PSU 模式：强制回流 MOSFET 打开
  else{                                                                //强制回流 MOSFET 打开
    if(voltageInput>voltageOutput+voltageDropout){bypassEnable=1;}     //打开回流 MOSFET（非 MPPT 充电器模式时默认开启）
    else{bypassEnable=0;}                                              //充电器模式：输入欠压 - 关闭旁路 MOSFET 并防止 PV 回流
  }
  digitalWrite(backflow_MOSFET,bypassEnable);                          //信号回流 MOSFET GPIO 引脚   
}

void Device_Protection(){
  //错误计数器复位
  currentRoutineMillis = millis();
  if(currentErrorMillis-prevErrorMillis>=errorTimeLimit){                                           //每 millisErrorInterval (ms) 运行例程
    prevErrorMillis = currentErrorMillis;                                                           //存储上一次
    if(errorCount<errorCountLimit){errorCount=0;}                                                   //如果在 x 毫秒前低于限制，则重置错误计数  
    else{}                                                                                          //添加：如果仍然存在太多错误，则睡眠和充电暂停
  } 
  //故障检测    
  ERR = 0;                                                                                          //重置本地错误计数器
  backflowControl();                                                                                //运行回流电流保护协议   
  if(temperature>temperatureMax)                           {OTE=1;ERR++;errorCount++;}else{OTE=0;}  //检测到系统过热
  if(currentInput>currentInAbsolute)                       {IOC=1;ERR++;errorCount++;}else{IOC=0;}  //输入电流已达到绝对极限
  if(currentOutput>currentOutAbsolute)                     {OOC=1;ERR++;errorCount++;}else{OOC=0;}  //输出电流已达到绝对极限
  if(voltageOutput>voltageBatteryMax+voltageBatteryThresh) {OOV=1;ERR++;errorCount++;}else{OOV=0;}  //输出电压已达到绝对极限                     
  if(voltageInput<vInSystemMin&&voltageOutput<vInSystemMin){FLV=1;ERR++;errorCount++;}else{FLV=0;}  //系统电压极低（无法恢复运行）

  if(output_Mode==0){                                                                               //PSU MODE 特定保护协议
    REC = 0; BNC = 0;                                                                               //清除恢复和电池未连接布尔标识符
    if(voltageInput<voltageBatteryMax+voltageDropout){IUV=1;ERR++;errorCount++;}else{IUV=0;}        //输入电压低于电池电压（仅适用于 psu 模式）                     
  }
  else{                                                                                             //Charger MODE 特定保护协议
    backflowControl();                                                                              //启用回流电流检测和控制                           
    if(voltageOutput<vInSystemMin)                   {BNC=1;ERR++;}      else{BNC=0;}               //BNC - BATTERY NOT CONNECTED（仅适用于充电器模式，不在 MPPT 模式下不将 BNC 视为错误)
    if(voltageInput<voltageBatteryMax+voltageDropout){IUV=1;ERR++;REC=1;}else{IUV=0;}               //输入电压低于最大电池充电电压（仅适用于充电器模式）     
  } 
}
