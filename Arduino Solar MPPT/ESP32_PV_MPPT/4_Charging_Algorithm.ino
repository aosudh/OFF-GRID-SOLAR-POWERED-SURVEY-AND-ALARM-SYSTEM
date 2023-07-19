void buck_Enable(){                                                                  //启用 MPPT 降压转换器
  buckEnable = 1;
  digitalWrite(buck_EN,HIGH);
  digitalWrite(LED,HIGH);
}
void buck_Disable(){                                                                 //禁用 MPPT 降压转换器
  buckEnable = 0; 
  digitalWrite(buck_EN,LOW);
  digitalWrite(LED,LOW);
  PWM = 0;
}   
void predictivePWM(){                                                                //预测PWM 算法 
  if(voltageInput<=0){PPWM=0;}                                                       ///当电压输入为零时防止不确定答案
  else{PPWM =(PPWM_margin*pwmMax*voltageOutput)/(100.00*voltageInput);}              //计算预测 PWM Floor 并存储在变量中
  PPWM = constrain(PPWM,0,pwmMaxLimited);
}   

void PWM_Modulation(){
  if(output_Mode==0){PWM = constrain(PWM,0,pwmMaxLimited);}                          //PSU MODE PWM = PWM OVERFLOW PROTECTION（将下限限制为 0%，上限限制为最大允许占空比）
  else{
    predictivePWM();                                                                 //运行和计算预测 pwm floor
    PWM = constrain(PWM,PPWM,pwmMaxLimited);                                         //CHARGER MODE PWM - 将下限限制为 PPWM，上限限制为最大允许占空比）                                     
  } 
  ledcWrite(pwmChannel,PWM);                                                         //设置 PWM 占空比并在启用降压时写入 GPIO
  buck_Enable();                                                                     //开启 MPPT 降压 (IR2104)
}
     
void Charging_Algorithm(){
  if(ERR>0||chargingPause==1){buck_Disable();}                                       //当出现错误或充电暂停用于暂停覆盖时关闭 MPPT 降压
  else{
    if(REC==1){                                                                      // IUV RECOVERY - (仅对充电模式有效)
      REC=0;                                                                         //重置 IUV 恢复布尔标识符
      buck_Disable();                                                                //在 PPWM 初始化之前禁用降压
      lcd.setCursor(0,0);lcd.print("POWER SOURCE    ");                              //显示液晶信息
      lcd.setCursor(0,1);lcd.print("DETECTED        ");                              //显示液晶信息
      Serial.println("> Solar Panel Detected");                                      //显示串口信息
      Serial.print("> Computing For Predictive PWM ");                               //显示串口信息
      for(int i = 0; i<40; i++){Serial.print(".");delay(30);}                        //For loop "loading... 效果
      Serial.println("");                                                            //在串行上显示下一行的换行符  
      Read_Sensors();
      predictivePWM();
      PWM = PPWM; 
      lcd.clear();
    }  
    else{                                                                            //NO ERROR PRESENT - 继续电源转换
      /////////////////////// CC-CV BUCK PSU ALGORITHM ////////////////////////////// 
      if(MPPT_Mode==0){                                                              // CC-CV PSU 模式
        if(currentOutput>currentCharging)       {PWM--;}                             //电流高于 → 降低占空比
        else if(voltageOutput>voltageBatteryMax){PWM--;}                             //电压高于 → 降低占空比   
        else if(voltageOutput<voltageBatteryMax){PWM++;}                             //当输出低于充电电压时增加占空比（仅用于 CC-CV 模式）
        else{}                                                                       //当达到设定的输出电压时什么都不做 
        PWM_Modulation();                                                            //将 PWM 信号设置为 Buck PWM GPIO       
      }     
        ///////////////////////  MPPT & CC-CV 充电算法 ///////////////////////  
      else{                                                                                                                                                         
        if(currentOutput>currentCharging){PWM--;}                                      //电流高于 → 降低占空比
        else if(voltageOutput>voltageBatteryMax){PWM--;}                               //电压高于 → 降低占空比  
        else{                                                                          //MPPT 算法
          if(powerInput>powerInputPrev && voltageInput>voltageInputPrev)     {PWM--;}  //  ↑P ↑V ; →MPP //D--
          else if(powerInput>powerInputPrev && voltageInput<voltageInputPrev){PWM++;}  //  ↑P ↓V ; MPP← //D++
          else if(powerInput<powerInputPrev && voltageInput>voltageInputPrev){PWM++;}  //  ↓P ↑V ; MPP→ //D++
          else if(powerInput<powerInputPrev && voltageInput<voltageInputPrev){PWM--;}  //  ↓P ↓V ; ←MPP  //D--
          else if(voltageOutput<voltageBatteryMax)                           {PWM++;}  //  MP MV ; 达到 MPP 
          powerInputPrev   = powerInput;                                               //  存储以前记录的功率
          voltageInputPrev = voltageInput;                                             //存储先前记录的电压        
        }   
        PWM_Modulation();                                                              //将 PWM 信号设置为 Buck PWM GPIO                                                                     
      }  
    }
  }
}
