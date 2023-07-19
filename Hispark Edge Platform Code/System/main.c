#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "math.h"

#include "iot_gpio_ex.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_uart.h"
#include "iot_watchdog.h"
#include "iot_errno.h"

//定义接收来自GPS数据的数组长度1024
#define UART_BUFF_SIZE 1024
#define UART_BUFF_SIZE1 10240
//测试测量时间间隔5s
#define test_time 5000000
//实际测量时间间隔1s
#define U_SLEEP_TIME   1000000
#define U_SLEEP_TIME1  50000
//假定建筑物高度
#define height_building 50

/**********************************************************************************************************************/

void Uart1GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_0);

    // 设置GPIO0的管脚复用关系为UART1_TX Set the pin reuse relationship of GPIO0 to UART1_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_0, IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_1);

    // 设置GPIO1的管脚复用关系为UART1_RX Set the pin reuse relationship of GPIO1 to UART1_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_1, IOT_IO_FUNC_GPIO_1_UART1_RXD);
}

void Uart2GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_11);

    // 设置GPIO11的管脚复用关系为UART2_TX Set the pin reuse relationship of GPIO11 to UART2_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_11, IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_12);

    // 设置GPIO12的管脚复用关系为UART2_RX Set the pin reuse relationship of GPIO12 to UART2_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_12, IOT_IO_FUNC_GPIO_12_UART2_RXD);
}
/**********************************************************************************************************************/

void Uart1Config(void)
{
    uint32_t ret1;

    /* 初始化UART1配置，波特率 115200，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART1 configuration, baud rate is 115200, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    ret1 = IoTUartInit(HI_UART_IDX_1, &uart_attr);

    if (ret1 != IOT_SUCCESS) {
        printf("Init Uart1 Falied Error No : %d\n", ret1);
        return;
    }
}

void Uart2Config(void)
{
    uint32_t ret2;

    /* 初始化UART2配置，波特率 115200，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART2 configuration, baud rate is 115200, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    ret2 = IoTUartInit(HI_UART_IDX_2, &uart_attr);

    if (ret2 != IOT_SUCCESS) {
        printf("Init Uart2 Falied Error No : %d\n", ret2);
        return;
    }
}

/**********************************************************************************************************************/

//将GPS $GNGGA ddmm.mmmm（度分）转换成常用坐标dd.ddddd （WGS-84国际标准下）
double dm_to_dd(double dm) {
    // 获取度部分
    double degree = (int)(dm / 100);
    // 获取分部分
    double minute = dm - degree * 100;
    // 将度分转换为小数形式
    double decimal = degree + minute / 60;
    
    return decimal;
}

//计算基准站和移动站距离
double Distance(double lata, double loga, double latb, double logb) 
{
    double EARTH_RADIUS = 6371.0;
    double PI = 3.14;
    double distance = 0.0;
    double degree = 180;

    double lat_a = 0.0;
    double lat_b = 0.0;
    double log_a = 0.0;
    double log_b = 0.0;
    
    //转弧度
    lat_a = lata  * PI / degree;
    lat_b = latb  * PI / degree;
    log_a = loga  * PI / degree;
    log_b = logb  * PI / degree;
 
    double dis = cos(lat_b) * cos(lat_a) * cos(log_b -log_a) + sin(lat_a) * sin(lat_b);
    
    distance = EARTH_RADIUS * acos(dis);
    
    return distance*1000;//km转换成m
}





double p1_last = 0;
double x1_last = 0;
//过程噪音
#define P_Q 0.001
//测量噪声
#define M_R 0.05

/*
Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
r参数调整滤波后的曲线与实测曲线的相近程度，r越小越接近。
q参数调滤波后的曲线平滑程度，q越小越平滑。
*/
double KalmanFilter1(double ResrcData)
{
    double R1 = M_R;
    double Q1 = P_Q;
    double x1_mid = x1_last;
    double x1_now;
    double p1_mid ;
    double p1_now;
    double kg1;
    //这里p_last 等于 kalmanFilter_A 的p直接取0
    x1_mid=x1_last;//x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p1_mid=p1_last+Q1;//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    /*
    卡尔曼滤波的五个重要公式
    */
    kg1=p1_mid/(p1_mid+R1);//kg为kalman filter，R 为噪声
    x1_now=x1_mid+kg1*(ResrcData-x1_mid);//估计出的最优值
    p1_now=(1-kg1)*p1_mid;//最优值对应的covariance
    p1_last = p1_now;//更新covariance 值
    x1_last = x1_now;//更新系统状态值
    return x1_now;
}

double p2_last = 0;
double x2_last = 0;

/*
Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
r参数调整滤波后的曲线与实测曲线的相近程度，r越小越接近。
q参数调滤波后的曲线平滑程度，q越小越平滑。
*/
double KalmanFilter2(double ResrcData)
{
    double R2 = M_R;
    double Q2 = P_Q;
    double x2_mid = x2_last;
    double x2_now;
    double p2_mid ;
    double p2_now;
    double kg2;
    //这里p_last 等于 kalmanFilter_A 的p直接取0
    x2_mid=x2_last;//x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p2_mid=p2_last+Q2;//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    /*
    卡尔曼滤波的五个重要公式
    */
    kg2=p2_mid/(p2_mid+R2);//kg为kalman filter，R 为噪声
    x2_now=x2_mid+kg2*(ResrcData-x2_mid);//估计出的最优值
    p2_now=(1-kg2)*p2_mid;//最优值对应的covariance
    p2_last = p2_now;//更新covariance 值
    x2_last = x2_now;//更新系统状态值
    return x2_now;
}

/**********************************************************************************************************************/

static void UartTask(void)
{
    const char *data = "Hello OpenHarmony !!!\n";
    uint32_t count = 0;
    uint32_t len1 = 0;
    uint32_t len2 = 0;
    uint32_t len0 = 0;
    unsigned char uartReadBuff[UART_BUFF_SIZE] = {0};
    unsigned char uartReadBuff1[UART_BUFF_SIZE1] = {0};
    unsigned char uartReadBuff2[UART_BUFF_SIZE] = {0};
    unsigned char uartReadBuff0[UART_BUFF_SIZE] = {0};

    // 对UART1的一些初始化 Some initialization of UART1
    Uart1GpioInit();
    // 对UART1参数的一些配置 Some configurations of UART1 parameters
    Uart1Config();

    // 对UART2的一些初始化 Some initialization of UART2
    Uart2GpioInit();
    // 对UART2参数的一些配置 Some configurations of UART2 parameters
    Uart2Config();

    while (1) {
        // 通过UART1 接收数据 Receive data through UART1
        //usleep(test_time);
        usleep(U_SLEEP_TIME);
        
        len1 = IoTUartRead(HI_UART_IDX_1, uartReadBuff, UART_BUFF_SIZE);
        if (len1 > 0) {
            // 把接收到来自GPS模块的数据打印出来 Print the received data

            printf("\r\n--------------------------------------------------------------\r\n");
            printf("[ %d ]\r\n", count);
            printf("The serial port information output by the GPS module is : \r\n");
            printf("%s \r\n",uartReadBuff);

            
            /***********************************解析GPS模块发送的 NMEA 0183协议的数据帧**************************************/

            /**********************************获取$GNGGA的经纬度信息（WGS-84国际标准下）***************************************/
            // 查找$GNGGA所在的行
            char *nmea_gngga = strstr(uartReadBuff, "$GNGGA");
            if (nmea_gngga == NULL) {
                 printf("[$GNGGA,WGS-84]The information on the $GNGGA line not found\n");
            }else{
                 printf("[$GNGGA,WGS-84]The information on the $GNGGA line is found\n");
            }

            // 使用strtok函数分割 $GNGGA所在的行
            char *token;
            token = strtok(nmea_gngga, ",");
            char *nmea_fields[15]; //$GNGGA所在的行有15个数
            int i = 0;
            while (token != NULL) {
                 nmea_fields[i++] = token;
                 token = strtok(NULL, ",");
                 if (i >= 15) break;
            }
        
            // 获取第2,3,4,5,6,7,9个字段的值(即GPS模块接收的经纬度坐标，卫星状态，卫星数量，海拔高度以及UTC时间)
            char ddmm_output[128];
            char state_output[1];
            char satellite_number[2];
            char altitude_output[8];
            char latitude[8];
            char longtitude[6];
            char s_latitude[8];
            char s_longtitude[6];
            char time[4];
            int current_state;
            char state[1];
            char zero[1];
            int z = 0;
            float f_time;

            if (i > 10)
            {  
                //将第3,4,5,6个字段的值合并在一起，即ddmm_output的值为GPS模块接收的经纬度坐标(ddmm.mmmmm格式),例如3209.22695 N,11841.95567 E
                //当GPS没有连接信号时，这个数值打印出来是错误的。
                //nmea_fields[1]表示UTC时间
                //nmea_fields[2]表示纬度
                //nmea_fields[3]来判断是北纬还是南纬，
                //nmea_fields[4]表示经度，
                //nmea_fields[5]来判断是东经还是西经
                //nmea_fields[6]判断GPS状况
                //nmea_fields[7]表示当前的卫星数量
                //nmea_fields[9]表示海拔高度
                    
                sprintf(state_output, "%s", nmea_fields[6]);
                sprintf(satellite_number, "%s", nmea_fields[7]);
                sprintf(altitude_output, "%s", nmea_fields[9]);
                sprintf(ddmm_output, " %s %s %s %s", nmea_fields[2], nmea_fields[3], nmea_fields[4], nmea_fields[5]);
                sprintf(time, "%s", nmea_fields[1]);
                printf("[$GNGGA,WGS-84]Data (ddmm.mmmmm) is : %s \n", ddmm_output);//ddmm_output的值为GPS模块接收的经纬度坐标(ddmm.mmmmm格式)
                    

                /******将GPS $GNGGA ddmm.mmmm（度分）转换成常用坐标dd.ddddd******/
                //将字符数组nmea_fields[2]转换为double类型
                double num0= 0.0;
                num0 = atof(nmea_fields[2]);
                //将字符数组nmea_fields[2]由ddmm.mmmm（度分）转换成常用坐标dd.ddddd （WGS-84国际标准下）
                double dd0 = dm_to_dd(num0);

                //将字符数组nmea_fields[4]转换为double类型
                double num1 = 0.0;
                num1 = atof(nmea_fields[4]);
                //将字符数组nmea_fields[4]由ddmm.mmmm（度分）转换成常用坐标dd.ddddd （WGS-84国际标准下）
                double dd1 = dm_to_dd(num1);

                //通过卡尔曼滤波减小系统精度误差
                dd0 = KalmanFilter1(dd0);
                dd1 = KalmanFilter2(dd1);
                    
                sprintf(latitude, "%0.8f", dd0);
                sprintf(longtitude, "%0.8f", dd1);
                sprintf(s_latitude, "%0.5f", dd0);
                sprintf(s_longtitude, "%0.5f", dd1);
                sprintf(zero, "%d", z);        
                f_time = atof(nmea_fields[9]);
                f_time = f_time*10;
                sprintf(nmea_fields[9], "%0.0f", f_time);

                //通过差值计算得到当前的偏移量
                double distance = 0;
                double latitude1, latitude2, longtitude1, longtitude2;
                latitude1 = latitude2;
                latitude2 = dd0;
                longtitude1 = longtitude2;
                longtitude2 = dd1;
                distance = Distance(latitude1, longtitude1, latitude2, longtitude2);

                if(strcmp(state_output, "1")==0)
                    current_state = 3;//单点测量-波动状态
                if(strcmp(state_output, "5")==0)
                    current_state = 3;//RTK（Float）测量-波动状态
                if(strcmp(state_output, "4")==0)//RTK（Fixed）测量
                {
                    if(distance/height_building <= 0.003)
                        current_state = 1;//正常状态
                    if(distance/height_building > 0.003)
                        current_state = 2;//危险状态
                }
                if(strcmp(state_output, "0")==0)
                    current_state = 0;//无信号状态
                   
                sprintf(state, "%d", current_state);

                /*
                    for (int a =0;a<11;a++)
                {
                    printf("%s\n",nmea_fields[a]);//测试分隔的数据是否正确
                }
                */

                printf("[$GNGGA,WGS-84]Longitude data (dd.ddddd) is : %s %s\n", nmea_fields[5], s_longtitude);//输出当前经度
                printf("[$GNGGA,WGS-84]Latitude data (dd.ddddd) is : %s %s\n", nmea_fields[3], s_latitude);//输出当前纬度
                printf("[$GNGGA,WGS-84]Current satellite state is : %s\n", state_output);//输出当前GPS状态
                printf("[$GNGGA,WGS-84]Satellite number is : %s\n", satellite_number);//输出当前使用的卫星数量
                printf("[$GNGGA,WGS-84]Altitude is : %s\n", nmea_fields[9]);//输出目前的海拔高度
                printf("[$GNGGA,WGS-84]UTC Time is : %s\n", nmea_fields[1]);//输出当前的UTC时间
                printf("Current system State is : %s\n", state);//输出当前的系统状态
                printf("Distance: %0.8f\n", distance);//输出计算出的两点距离
                printf("Gradient inclination is %0.6f\n", distance/height_building);//输出当前的偏移度

                /**************************************************************/

                //IoTUartWrite(HI_UART_IDX_2, (unsigned char*)nmea_fields[5], strlen(nmea_fields[5]));//判断东经还是西经
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)s_longtitude, strlen(s_longtitude));//具体经度值
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)zero, strlen(zero));//占空填写数据库
                //IoTUartWrite(HI_UART_IDX_2, (unsigned char*)nmea_fields[3], strlen(nmea_fields[3]));//判断北纬还是南纬
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)s_latitude, strlen(s_latitude));//具体纬度值
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)state, strlen(state));//当前的系统状态
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)satellite_number, strlen(satellite_number));//当前GPS模块使用的卫星数量
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)nmea_fields[9], strlen(nmea_fields[9]));//当前的测量出的海拔高度
                for(int i =0; i< 9; i++)
                {
                    IoTUartWrite(HI_UART_IDX_2, (unsigned char*)zero, strlen(zero));
                }
                //IoTUartWrite(HI_UART_IDX_2, (unsigned char*)nmea_fields[1], strlen(nmea_fields[1]));//当前的UTC时间
            }else   { printf("[$GNGGA]Latitude and longitude data no found\n"); }
            /********************************************************************************/

            /****************************************************************************************************************/

        }
        count++;

    }

}

/**********************************************************************************************************************/

void UartExampleEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();

    attr.name = "UartTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 25 *1024; // 任务栈大小*1024 
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)UartTask, NULL, &attr) == NULL) {
        printf("[UartTask] Failed to create UartTask!\n");
    }
}

APP_FEATURE_INIT(UartExampleEntry);