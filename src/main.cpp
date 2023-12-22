#include <Arduino.h>
#include "stdlib.h"
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "HTTPUpdate.h" //HTTP OTA库
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "esp_task_wdt.h" //设置看门狗用
#include "blufi.h"

#include "esp_bt.h"
#include "esp_adc_cal.h"
#include "TFT_eSPI.h"
#include "SPI.h"

#include "bmp.h"      //导入图库
#include "font_diy.h" //导入字库
// #include "HttpsOTAUpdate.h"//HTTPS OTA库

// SEN55 Read Measurement
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;
int Aqi_PM25_value = 0;
//*****屏幕变量*****//
TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h
// TFT_eSprite sprite = TFT_eSprite(&tft);  // 创建一个 TFT_eSprite 对象，tft 是你的 TFT 显示对象
char Screen_page = 0; // 调试测试
char Screen_chage = 0;
int Screen_Aqi = -1;
float Screen_temp = -255;
int Screen_hum = -255;
int Screen_pm25 = -1;
int Screen_pm10 = -1;
int Screen_voc = -1;
int Screen_nox = -1;
int Screen_wifi = -1;
int Screen_weatemp_max = -255;
int Screen_weatemp_min = -255;
int MQTT_weatemp_max = -254;
int MQTT_weatemp_min = -254;
int Screen_power = -1;
float power;
char Screen_Rotation = 0; // 屏幕方向，正向：0，反向：1
char MQTT_Rotation = 0;   // 屏幕方向，正向：0，反向：1
/*WiFi配置*/
const char *wifi_id = "1801";
const char *wifi_psw = "realtemp";
const char *ntpServer = "pool.ntp.org"; // 网络时间
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;
int wifi_flag = 0;
uint8_t macAddr[6]; // 定义macAddr为uint8_t类型的数组，这个数组含有6个元素。
uint8_t btAddr[6];  // 蓝牙MAC地址

//*****MQTT协议数据声明部分*****//
WiFiClient espClient;                                                   // 定义wifiClient实例
long lastMsg = 0;                                                       // 记录上一次发送信息的时长
const char *mqtt_server = "221.224.143.146";                            // 使用HIVEMQ 的信息中转服务
const int port = 1883;                                                  // 端口号
char TOPIC[20] = "set";                                                 // 订阅信息主题
char client_id[20] = "RTair-";                                          // 标识当前设备的客户端编号
void Mqtt_getcallback(char *topic, byte *payload, unsigned int length); // 回调函数
PubSubClient client(mqtt_server, port, Mqtt_getcallback, espClient);    // 定义PubSubClient的实例
//*****MQTT接收解析JSON数据声明部分*****//
StaticJsonDocument<200> jsonBuffer; // 声明一个JsonDocument对象，长度200
DeserializationError jsonerror;     // 反序列化JSON
char jsonweath[20] = "";
char screenweath[20] = "";
//*****OTA升级相关数据声明部分*****//
static const char *OTA_url = "http://221.224.143.146:9800/center/firmware.bin"; // state url of your firmware image+
// String OTA_Burl = "http://bin.bemfa.com/b/3BcMzgxODlhNjllNDcyNGQyY2JiNTM0MTExMjA0MmRmNGQ=Air.bin";//远程固件链接（测试）
int ota_state = 0;
char ota_version[] = "A0.01_1";
/*// HttpsOTAUpdateClass HttpsOTA;
static HttpsOTAStatus_t otastatus;
static const char *server_certificate = "-----BEGIN CERTIFICATE-----\n" \
     "MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\n" \
     "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
     "DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\n" \
     "SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\n" \
     "GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\n" \
     "AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\n" \
     "q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\n" \
     "SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\n" \
     "Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\n" \
     "a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\n" \
     "/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\n" \
     "AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\n" \
     "CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\n" \
     "bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\n" \
     "c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\n" \
     "VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\n" \
     "ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\n" \
     "MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\n" \
     "Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\n" \
     "AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\n" \
     "uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\n" \
     "wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\n" \
     "X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\n" \
     "PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\n" \
     "KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n" \
     "-----END CERTIFICATE-----";
*/

//*****自定义变量声明*****//
char strbuff[20]; // 缓存芯片ID 高2byte，最终存放chipid的数组
uint64_t chipid;  // 读取到的 uint 型芯片ID
char espId_buff[20] = "";
bool led_State = 0;
bool wifi_connect_ok = 0;
bool ble_Scan_Init = 0;
bool ble_Scan_Flag = 0;
bool mqtt_flag = 0;    // // mqtt订阅主题变量 0：shibai 1：开启
bool screenstate = 1;  // 屏幕状态变量 0：关闭 1：开启
bool wifi_pdstate = 0; // WiFi重置变量 0：关闭 1：开启
// //ADC变量
int analogOriginalValueV = 0;
int analogOriginalValueI = 0;
int analogOriginalSumV = 0;
int analogOriginalSumI = 0;
float changeVoltsSumV = 0;
float changeVoltsSumI = 0;
int analogOriginalnum = 0;
// int BAT_PIN = 3;    // select the input pin for the ADC
// #define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
// #define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11
// static esp_adc_cal_characteristics_t adc1_chars;
// bool cali_enable;
//*****引脚控制变量*****
#define Touch_K1 0
#define screen 1
#define ADC_V 3
#define ADC_I 2
#define Rest_K1 10
#define LED1 10

/*//*****WiFi Scan变量*****
char Scan_num=0;
char Scan_time=0;
typedef struct ap_info
{
  unsigned char count;
  String mac;
  signed char rssi;
}apinfo;
apinfo ApInfo;
*/

//*****MQTT发布环境格式拼接*****//
char strbuff1[20] = {0};
char E_buff0[230] = "";
char E_buff1[20] = "{'TempMax':'";
char E_buff2[10] = "1"; // 等于 cardidBuff，仅用于测试
char E_buff3[10] = "','two':'";
char E_buff4[10] = "1"; // 表示当前设备状态，用户自定义，此值仅用于测试
char E_buff5[15] = "','TempMin':'";
char E_buff6[10] = "25"; // 表示当前设备SN号，用户自定义，此值仅用于测试
char E_buff7[10] = "','ten':'";
char E_buff8[10] = "25";
char E_buff9[20] = "','temperature':'";
char E_buff10[10] = "22.1";
char E_buff11[15] = "','humidity':'";
char E_buff12[10] = "22.1";
char E_buff13[15] = "','vocs':'";
char E_buff14[10] = "22.1";
char E_buff15[10] = "','nox':'";
char E_buff16[10] = "22.1";
char E_buff17[18] = "','screenState':'";
char E_buff18[2] = "1";
char E_buff19[20] = "','machineNumber':'";
char E_buff20[15] = "9E10000002";
char E_buff21[20] = "','aqi':'";
char E_buff22[5] = "350";
char E_buff23[20] = "','weather':'";
char E_buff24[20] = "";
char E_buff25[20] = "','ota_version':'";
char E_buff26[20] = "";
char E_buff27[20] = "','power':'";
char E_buff28[20] = "";
char E_buff29[5] = "'}"; // 等于 temperatureBuff，仅用于测试

//********自定义函数声明*******//
void getChipidID(void); // 获取芯片ID 函数
void restoreWiFi(void); // 删除保存的WIFI连接配置信息
void my_client_publish(char *cardidBuff, char *temperatureBuff);
void reconnect();
void Wifi_connect(void);
bool autoConfig(void);
void update_started();
// 当升级结束时，打印日志
void update_finished();
// 当升级中，打印日志
void update_progress(int cur, int total);
// 当升级失败时，打印日志
void update_error(int err);
void showImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data); // 函数声明
#define PI_BUF_SIZE 128
/**********RTOS任务声明**********/
// 任务1 看门狗
#define TASK1_TASK_PRIO 0       // 任务优先级
#define TASK1_STK_SIZE 1024     // 任务堆栈大小
TaskHandle_t Tasks1_TaskHandle; // 任务句柄
void task1(void *pvParameters); // 任务函数

// 任务2 网络连接
#define TASK2_TASK_PRIO 1       // 任务优先级
#define TASK2_STK_SIZE 1024 * 4 // 任务堆栈大小
TaskHandle_t Tasks2_TaskHandle; // 任务句柄
void task2(void *pvParameters); // 任务函数

// 任务3 传感器读取
#define TASK3_TASK_PRIO 1       // 任务优先级
#define TASK3_STK_SIZE 1024 * 8 // 任务堆栈大小
TaskHandle_t Tasks3_TaskHandle; // 任务句柄
void task3(void *pvParameters); // 任务函数

// 任务4 状态展示
#define TASK4_TASK_PRIO 2       // 任务优先级
#define TASK4_STK_SIZE 1024 * 1 // 任务堆栈大小
TaskHandle_t Tasks4_TaskHandle; // 任务句柄
void task4(void *pvParameters); // 任务函数

// 任务5 ADC
#define TASK5_TASK_PRIO 3       // 任务优先级
#define TASK5_STK_SIZE 1024 * 3 // 任务堆栈大小
TaskHandle_t Tasks5_TaskHandle; // 任务句柄
void task5(void *pvParameters); // 任务函数

// 任务6 屏幕
#define TASK6_TASK_PRIO 2              // 任务优先级
#define TASK6_STK_SIZE 1024 * 2        // 任务堆栈大小
TaskHandle_t Tasks6_TaskHandle;        // 任务句柄
void Screeen_task(void *pvParameters); // 任务函数

// 任务7 OTA
#define TASK7_TASK_PRIO 4          // 任务优先级
#define TASK7_STK_SIZE 1024 * 6    // 任务堆栈大小
TaskHandle_t Tasks7_TaskHandle;    // 任务句柄
void OTA_task(void *pvParameters); // 任务函数
// 系统软件复位入口函数
void (*resetFunc)(void) = 0;
class WATCHDOG
{
private:
  hw_timer_t *timer = NULL; // 定时器对象.
  uint8_t Timerindex = 0;   // 硬件定时器编号.默认定时器0.
  uint16_t Timeout = 20000; // 定时器计数.默认10秒超时.
protected:
  static void Callback()
  {
    // 定时器溢出回调函数,直接复位.
    esp_restart();
  }
  void Init()
  {
    if (timer != NULL)
    {
      timerEnd(timer);
    }                                             // 如果之前有设置过定时器,则关闭相应的定时器.
    timer = timerBegin(Timerindex, 80, true);     // 使用硬件定时器，预分频80，向上计数。
    timerAttachInterrupt(timer, Callback, true);  // 设置回调函数，边延触发。
    timerAlarmWrite(timer, Timeout * 1000, true); // 设定中断计数器值，自动重启计数（循环定时）。CPU主频80MHz/80/1000=1毫秒。
    timerAlarmEnable(timer);                      // 开启定时器。
  }

public:
  WATCHDOG()
  {
    Timerindex = 0;
    Timeout = 20000;
  }
  void begin() { Init(); }
  void begin(uint8_t Esp_Timerindex, uint16_t Esp_Timerout)
  {
    Timerindex = Esp_Timerindex, Timeout = Esp_Timerout;
    begin();
  }
  void feed(void) { timerWrite(timer, 0); } // 喂狗.
};
WATCHDOG Watchdog; // 看门狗对象

// SEN55
//  The used commands use up to 48 bytes. On some Arduino's the default buffer
//  space is not large enough
#define MAXBUF_REQUIREMENT 48
#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
// #define USE_PRODUCT_INFO//获取SEN55串口号和版本号，当前不需要
#endif
SensirionI2CSen5x sen5x;

void printModuleVersions()
{
  uint16_t error;
  char errorMessage[256];

  unsigned char productName[32];
  uint8_t productNameSize = 32;

  error = sen5x.getProductName(productName, productNameSize);

  if (error)
  {
    Serial.print("Error trying to execute getProductName(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print("ProductName:");
    Serial.println((char *)productName);
  }

  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  bool firmwareDebug;
  uint8_t hardwareMajor;
  uint8_t hardwareMinor;
  uint8_t protocolMajor;
  uint8_t protocolMinor;

  error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                           hardwareMajor, hardwareMinor, protocolMajor,
                           protocolMinor);
  if (error)
  {
    Serial.print("Error trying to execute getVersion(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print("Firmware: ");
    Serial.print(firmwareMajor);
    Serial.print(".");
    Serial.print(firmwareMinor);
    Serial.print(", ");

    Serial.print("Hardware: ");
    Serial.print(hardwareMajor);
    Serial.print(".");
    Serial.println(hardwareMinor);
  }
}

void printSerialNumber()
{
  uint16_t error;
  char errorMessage[256];
  unsigned char serialNumber[32];
  uint8_t serialNumberSize = 32;

  error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
  if (error)
  {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print("SerialNumber:");
    Serial.println((char *)serialNumber);
  }
}
void Sen55_init()
{
  // Init Sen5x
  Wire.begin(18, 19);
  sen5x.begin(Wire);

  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  if (error)
  {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
  printSerialNumber();
  printModuleVersions();
#endif

  // set a temperature offset in degrees celsius//以摄氏度为单位设置温度偏移
  // Note: supported by SEN54 and SEN55 sensors//注：SEN54和SEN55传感器支持
  // By default, the temperature and humidity outputs from the sensor//默认情况下，传感器的温度和湿度输出
  // are compensated for the modules self-heating. If the module is//对模块自加热进行补偿。如果模块是
  // designed into a device, the temperature compensation might need//设计成一个设备，温度补偿可能需要
  // to be adapted to incorporate the change in thermal coupling and//以适应热耦合的变化，以及
  // self-heating of other device components.//其他设备部件的自加热。

  // A guide to achieve optimal performance, including references//实现最佳性能的指南，包括参考资料
  // to mechanical design-in examples can be found in the app note//到机械设计的例子可以在应用程序说明中找到
  // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.//www.sensirion.com上的“SEN5x–温度补偿说明”。
  // Please refer to those application notes for further information//有关更多信息，请参阅这些申请说明
  // on the advanced compensation settings used//关于使用的高级补偿设置
  // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and//在“setTemperatureOffsetParameters”、“setWarmStartParameter”和
  // `setRhtAccelerationMode`.//`setRht选择模式`。

  // Adjust tempOffset to account for additional temperature offsets//`setRht选择模式`。
  // exceeding the SEN module's self heating.//超过SEN模块的自加热。

  float tempOffset = 0;
  error = sen5x.setTemperatureOffsetSimple(tempOffset);
  if (error)
  {
    Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print("Temperature Offset set to ");
    Serial.print(tempOffset);
    Serial.println(" deg. Celsius (SEN54/SEN55 only");
  }

  int AccelerationMode = 2;
  error = sen5x.setRhtAccelerationMode(AccelerationMode);
  if (error)
  {
    Serial.print("Error trying to execute setRhtAccelerationMode(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print("Temperature AccelerationMode set to ");
    Serial.print(AccelerationMode);
  }
  // Start Measurement
  error = sen5x.startMeasurement();
  if (error)
  {
    Serial.print("Error trying to execute startMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
}
// 传感器读数
void Sen55_Read()
{
  uint16_t error;
  char errorMessage[256];
  char strbuff[20] = {0};
  int slider_num;
  delay(1000);

  error = sen5x.readMeasuredValues( // 返回测量值
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
      noxIndex);

  if (error)
  {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    //   if (isnan(massConcentrationPm1p0))
    //   {
    //     Serial.print("n/a");
    //   }
    //   else
    //   {
    //     Serial.print("MassConcentrationPm1p0:");
    //     Serial.print(massConcentrationPm1p0);
    //     Serial.print("\t");
    //   }

    //   if (isnan(massConcentrationPm2p5))
    //   {
    //     Serial.print("n/a");
    //   }
    //   else
    //   {
    //     Serial.print("MassConcentrationPm2p5:");
    //     Serial.print(massConcentrationPm2p5);
    //     Serial.print("\t");
    //   }

    //   if (isnan(massConcentrationPm10p0))
    //   {
    //     Serial.print("n/a");
    //   }
    //   else
    //   {
    //     Serial.print("massConcentrationPm10p0:");
    //     Serial.print(massConcentrationPm10p0);
    //     Serial.print("\t");
    //   }

    if (massConcentrationPm2p5 >= 0 & massConcentrationPm2p5 <= 35)
    {
      Aqi_PM25_value = (int)(50 - 0) * (massConcentrationPm2p5 - 0) / (35 - 0) + 0;
    }
    else if (massConcentrationPm2p5 > 35 & massConcentrationPm2p5 <= 75)
    {
      Aqi_PM25_value = (int)(100 - 50) * (massConcentrationPm2p5 - 35) / (75 - 35) + 50;
    }
    else if (massConcentrationPm2p5 > 75 & massConcentrationPm2p5 <= 115)
    {
      Aqi_PM25_value = (int)(150 - 100) * (massConcentrationPm2p5 - 75) / (115 - 75) + 100;
    }
    else if (massConcentrationPm2p5 > 115 & massConcentrationPm2p5 <= 150)
    {
      Aqi_PM25_value = (int)(200 - 150) * (massConcentrationPm2p5 - 115) / (150 - 115) + 150;
    }
    else if (massConcentrationPm2p5 > 150 & massConcentrationPm2p5 <= 250)
    {
      Aqi_PM25_value = (int)(300 - 200) * (massConcentrationPm2p5 - 150) / (250 - 150) + 200;
    }
    else if (massConcentrationPm2p5 > 250 & massConcentrationPm2p5 <= 350)
    {
      Aqi_PM25_value = (int)(400 - 300) * (massConcentrationPm2p5 - 250) / (350 - 250) + 300;
    }
    else if (massConcentrationPm2p5 > 350 & massConcentrationPm2p5 <= 500)
    {
      Aqi_PM25_value = (int)(500 - 400) * (massConcentrationPm2p5 - 350) / (500 - 350) + 400;
    }
    else
    {
      Aqi_PM25_value = 500; // AQI大于500
    }
    // Serial.print("Aqi_PM25_value:");
    // Serial.print(Aqi_PM25_value);
    // Serial.print("\t");
    // Serial.print("AmbientHumidity:");

    // if (isnan(ambientHumidity))
    // {
    //   Serial.print("n/a");
    // }
    // else
    // {
    //   Serial.print(ambientHumidity);
    // }
    // Serial.print("\t");

    // Serial.print("AmbientTemperature:");
    // if (isnan(ambientTemperature))
    // {
    //   Serial.print("n/a");
    // }
    // else
    // {
    //   Serial.print(ambientTemperature);
    // }
    // Serial.print("\t");

    // Serial.print("VocIndex:");
    if (isnan(vocIndex))
    {
      vocIndex = 0;
      // Serial.print("n/a");
    }
    else
    {
      // Serial.print(vocIndex);
    }
    // Serial.print("\t");

    // Serial.print("NoxIndex:");
    if (isnan(noxIndex))
    {
      noxIndex = 0;
      // Serial.println("n/a");
    }
    else
    {
      // Serial.println(noxIndex);
    }
    // httpAdd();
  }
}
// MQTT发送数据
void M_send()
{
  bool mqtt_publish_state = 0;

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%d", (int)MQTT_weatemp_max);
  strcat(E_buff0, E_buff1); // 温度上限
  strcat(E_buff0, strbuff1);

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%d", (int)massConcentrationPm2p5);
  strcat(E_buff0, E_buff3); // two
  strcat(E_buff0, strbuff1);

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%d", (int)MQTT_weatemp_min);
  strcat(E_buff0, E_buff5);  // 温度下限
  strcat(E_buff0, strbuff1); //

  strcat(E_buff0, E_buff7); // ten
  sprintf(strbuff1, "%d", (int)massConcentrationPm10p0);
  strcat(E_buff0, strbuff1);

  sprintf(strbuff1, "%.1f", ambientTemperature);
  strcat(E_buff0, E_buff9); // temperature温度
  strcat(E_buff0, strbuff1);

  sprintf(strbuff1, "%d", (int)ambientHumidity);
  strcat(E_buff0, E_buff11); // humidity
  strcat(E_buff0, strbuff1);

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%d", (int)vocIndex);
  strcat(E_buff0, E_buff13); // vocs
  strcat(E_buff0, strbuff1);

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%.1f", (float)noxIndex);
  strcat(E_buff0, E_buff15); // nox
  strcat(E_buff0, strbuff1); //

  sprintf(strbuff1, "%d", (int)screenstate);
  strcat(E_buff0, E_buff17); // screenstate
  strcat(E_buff0, strbuff1);

  strcat(E_buff0, E_buff19); // 设备号
  strcat(E_buff0, espId_buff);

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%d", Aqi_PM25_value);
  strcat(E_buff0, E_buff21); // aqi值 Aqi_PM25_value
  strcat(E_buff0, strbuff1);

  strcat(E_buff0, E_buff23);  // 天气值 jsonweath
  strcat(E_buff0, jsonweath); // E_buff24

  strcat(E_buff0, E_buff25);    // 软件版本值 ota_version
  strcat(E_buff0, ota_version); // E_buff26

  memset(strbuff1, 0, strlen(strbuff1));
  sprintf(strbuff1, "%d", (int)power);
  strcat(E_buff0, E_buff27); // 功率值 power
  strcat(E_buff0, strbuff1); // E_buff28

  strcat(E_buff0, E_buff29);
  // Serial.printf("E_sent buff = %s\r\n", E_buff0);

  mqtt_publish_state = client.publish("Aircharge-Topic", E_buff0);
  if (mqtt_publish_state == true)
    Serial.printf("E_MQTT publish wifi success!\r\n");
  else
    Serial.printf("E_MQTT publish failed!\r\n");

  memset(E_buff0, 0, sizeof(E_buff0));
  memset(strbuff1, 0, strlen(strbuff1));
}
/*******************************用户函数代码部分--开始*******************************/
// WiFi连接函数
void Wifi_connect(void)
{
  // WiFi.begin();
  WiFi.beginSmartConfig();
  delay(1000);
  wifi_connect_ok = autoConfig(); // 连接自动判断函数

  if (wifi_connect_ok == 0)
  {
    // esp_wifi_restore();       // 删除保存的WiFi信息

    blufi_init(espId_buff); // 蓝牙配网函数
    Serial.printf("SSID:none\r\n");
    wifi_connect_ok = 0;
  }
  // esp_wifi_restore();       // 删除保存的WiFi信息
  // WiFi.disconnect(true); // 参数为 true 表示清除保存的 WiFi 凭据
}
// 连接判断函数
bool autoConfig(void)
{
  wifi_config_t myconfig = {0};
  esp_wifi_get_config(WIFI_IF_STA, &myconfig);
  if (strlen((char *)myconfig.sta.ssid) > 0)
  // if (WiFi.status() == WL_CONNECTED)
  {
    esp_wifi_connect();
    for (int i = 0; i < 6; i++)
    {
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.printf("SSID:%s\r\n", myconfig.sta.ssid);
        Serial.println(WiFi.localIP());
        return 1;
      }
      else
      {
        Serial.print(".");
        Serial.println(WiFi.status());
        delay(1000);
      }
    }
    return 0;
  }
  return 0;
}
/*//WiFi扫描函数
void wifi_scan()
{
  int n = WiFi.scanNetworks();
  ApInfo.count =n;
  Serial.println("scan done");
  if (n == 0)
  {
    Serial.println("no networks found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      ApInfo.rssi = WiFi.RSSI(i);
      ApInfo.mac = WiFi.BSSIDstr(i);
      Serial.printf("%d RSSI ",i);
      Serial.print(WiFi.RSSI(i));
      Serial.print(" ");
      Serial.println(WiFi.BSSIDstr(i));
      delay(10);
    }
  }
}*/
// 获取芯片ID 函数
void getChipidID(void)
{
  char buff[40] = "";

  WiFi.macAddress(macAddr); // MAC地址会储存在这个macAddr数组里面
  esp_read_mac(btAddr, ESP_MAC_BT);

  for (int i = 0; i < 6; i++)
  {
    sprintf(buff, "%02X", btAddr[i]);
    strcat(espId_buff, buff);
  }
  sprintf(TOPIC, "%sset", espId_buff);

  Serial.printf("ESP32 version = %s\r\n", ota_version);
  Serial.printf("ESP32 TOPIC = %s\r\n", TOPIC);
  Serial.printf("ESP32 chipid = %s\r\n", espId_buff);
  Serial.printf("WiFi MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  Serial.printf("BLE MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", btAddr[0], btAddr[1], btAddr[2], btAddr[3], btAddr[4], btAddr[5]);
  strcat(client_id, espId_buff);

  Serial.printf("ESP32 client_id = %s\r\n", client_id);
}
// 删除保存的wifi信息，这里的删除是删除存储在flash的信息。删除后wifi读不到上次连接的记录，需重新配网
void restoreWiFi(void)
{
  delay(500);
  esp_wifi_restore();    // 删除保存的wifi信息
  WiFi.disconnect(true); // 参数为 true 表示清除保存的 WiFi 凭据
  Serial.println("wifi message clean...");
  ESP.restart();
  delay(10);
}
// MQTT连接服务器，失败5S重连
void reconnect()
{
  client.setServer(mqtt_server, port); // 设定MQTT服务器与使用的端口，1883是默认的MQTT端口
  while (!client.connected())
  {
    // Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_id, "song", "song123"))
    {
      // Serial.println("connected");
      mqtt_flag = 1; // mqtt订阅主题成功
    }
    else
    {
      mqtt_flag = 0; // mqtt订阅主题失败
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      Serial.printf("failed, WiFi.status=%d\r\n", WiFi.status());
      // Wait 5 seconds before retrying
      delay(5000);
      // MQTT_CONNECTION_TIMEOUT     -4
      // MQTT_CONNECTION_LOST        -3
      // MQTT_CONNECT_FAILED         -2
      // MQTT_DISCONNECTED           -1
      // MQTT_CONNECTED               0
      // MQTT_CONNECT_BAD_PROTOCOL    1
      // MQTT_CONNECT_BAD_CLIENT_ID   2
      // MQTT_CONNECT_UNAVAILABLE     3
      // MQTT_CONNECT_BAD_CREDENTIALS 4
      // MQTT_CONNECT_UNAUTHORIZED    5
    }
  }
}
// 重定义MQTT发布主题的内容
void my_client_publish(char *cardidBuff, char *temperatureBuff)
{
  // bool mqtt_publish_state = 0;
  // // Serial.printf("current buff  cardidBuff = %s\r\n",cardidBuff);
  // strcat(buff0, buff1);
  // strcat(buff0, cardidBuff);
  // // strcat(buff0,buff3);
  // // strcat(buff0,buff4);
  // strcat(buff0, buff5);
  // strcat(buff0, espId_buff);
  // strcat(buff0, buff7);
  // strcat(buff0, temperatureBuff);
  // strcat(buff0, buff9);
  // Serial.printf("current buff = %s\r\n", buff0);

  // mqtt_publish_state = client.publish("bluetoothMachineRecordTopic", buff0);
  // if (mqtt_publish_state == true)
  //   Serial.printf("MQTT publish success!\r\n");
  // else
  //   Serial.printf("MQTT publish failed!\r\n");

  // memset(buff0, 0, sizeof(buff0));
  // memset(cardidBuff, 0, sizeof(cardidBuff));
  // memset(temperatureBuff, 0, sizeof(temperatureBuff));
  // Serial.printf("******************************\r\n");
}
// MQTT接收中断回调函数
void Mqtt_getcallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Get Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  jsonerror = deserializeJson(jsonBuffer, payload);     // 反序列化JSON
  if (!jsonerror && jsonBuffer.containsKey("openFlag")) // 判断是否有"openFlag"这一字段,如果有//开屏标志
  {
    if (!jsonBuffer["openFlag"].isNull())
    {
      screenstate = jsonBuffer["openFlag"]; // 解析JSON 读取字符串//0：关闭 1：开启
      // 输出结果：打印解析后的值
      Serial.printf("screenstate : %d", screenstate);
    }
  }
  if (!jsonerror && jsonBuffer.containsKey("weather")) // 判断是否有"weather"这一字段,如果有
  {
    if (!jsonBuffer["weather"].isNull())
    {
      const char *jsonweather = jsonBuffer["weather"]; // 天气
      strcpy(jsonweath, jsonweather);                  // 字符串从 jsonweather 复制到 jsonweath。
      // Serial.println(jsonweath);
    }
  }
  if (!jsonerror && jsonBuffer.containsKey("unbindMachine")) // 判断是否有"unbindMachine"这一字段,如果有
  {
    if (!jsonBuffer["unbindMachine"].isNull())
    {
      const char *jsonstr3 = jsonBuffer["unbindMachine"]; // 解绑
      if (strcmp(espId_buff, jsonstr3) == 0)
      {
        screenstate = 1;
      }
      // Serial.println(jsonstr3);
    }
  }
  if (!jsonerror && jsonBuffer.containsKey("OTAstate")) // 判断是否有"OTAstate"这一字段,如果有//OTA升级标志
  {
    if (!jsonBuffer["OTAstate"].isNull())
    {
      ota_state = jsonBuffer["OTAstate"]; // 读取字符串//0：关闭 1：开启
      // // 输出结果：打印解析后的值
      // Serial.println("OTAstate");
      // Serial.println(ota_state);
    }
  }
  if (!jsonerror && jsonBuffer.containsKey("OTAversion")) // 判断是否有"OTAversion"这一字段,如果有//OTA升级版本
  {
    if (!jsonBuffer["OTAversion"].isNull())
    {
      const char *jsonstr1 = jsonBuffer["OTAversion"]; // 读取字符串//0：关闭 1：开启
      // // 输出结果：打印解析后的值
      // Serial.println(jsonstr1);
    }
  }
  if (!jsonerror && jsonBuffer.containsKey("TempMax")) // 判断是否有"TempMax"这一字段,如果有//OTA升级标志
  {
    if (!jsonBuffer["TempMax"].isNull())
    {
      MQTT_weatemp_max = jsonBuffer["TempMax"]; // 读取字符串//0：关闭 1：开启
      // // 输出结果：打印解析后的值
      // Serial.println("MQTT_weatemp_max");
      // Serial.println(MQTT_weatemp_max);
    }
  }
  if (!jsonerror && jsonBuffer.containsKey("TempMin")) // 判断是否有"TempMin"这一字段,如果有//OTA升级标志
  {
    if (!jsonBuffer["TempMin"].isNull())
    {
      MQTT_weatemp_min = jsonBuffer["TempMin"]; // 读取字符串//0：关闭 1：开启
      // // 输出结果：打印解析后的值
      // Serial.println("MQTT_weatemp_min");
      // Serial.println(MQTT_weatemp_min);
    }
  }
  //{"openFlag":0,"weather":"大风","OTAstate":"0","nowTime":"2023-11-30 14:15:10"}
  // memset(payload, 0, sizeof(payload));// 在解析后清除 payload 中的数据
  // free(payload);//报错：block already marked as free
}
// 外部K1中断回调函数
void Touch_callBack(void)
{
  Screen_chage = 1;
  if (Screen_page < 2)
  {
    Screen_page++;
  }
  else
  {
    Screen_page = 0;
  }
}
// 外部复位中断回调函数
void Rest_callBack(void)
{
  wifi_pdstate = 1; // WiFi重置标志位
}

// 屏幕初始化
void screen_init()
{
  digitalWrite(screen, LOW); // 屏幕guanbi
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.loadFont(font_56);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("REALTEMP", 15, 90); // wendu
  tft.unloadFont();
  delay(200);
  digitalWrite(screen, HIGH); // 屏幕判断
  Screen_chage = 1;
}
int color_num = 0;
// 屏幕测试函数
void screen_test()
{
  int AQI_NUM = 12;
  if (Screen_chage == 1) // 页面改变
  {
    Screen_chage = 0;
    if (Screen_Rotation != MQTT_Rotation)
    {
      if (Screen_Rotation == 0)
      {
        tft.setRotation(1);
      }
      if (Screen_Rotation == 1)
      {
        tft.setRotation(3);
      }
      Screen_Rotation = MQTT_Rotation;
    }
    tft.fillScreen(TFT_BLACK);
    // Screen_wifi=-1;

    if (Screen_page == 0)
    {
      Screen_Aqi = -1;
      Screen_power = -1;
      Screen_weatemp_max = -254;
      Screen_weatemp_min = -254;
      memset(screenweath, 0, strlen(screenweath));
    }
    if (Screen_page == 1)
    {
      Screen_temp = -255;
      Screen_hum = -255;
    }
    if (Screen_page == 2)
    {
      Screen_pm25 = -1;
      Screen_pm10 = -1;
      Screen_voc = -1;
      Screen_nox = -1;
    }
  }
  if (Screen_wifi != wifi_connect_ok) // WiFi图标显示
  {
    if (wifi_connect_ok)
    {
      tft.drawBitmap(279, 14, icon_wifi2, 17, 16, TFT_BLACK); // WiFi断开图标
      tft.drawBitmap(279, 14, icon_wifi1, 17, 16, 0xC618);    // WiFi连接图标
    }
    else
    {
      tft.drawBitmap(279, 14, icon_wifi1, 17, 16, TFT_BLACK); // WiFi图标
      tft.drawBitmap(279, 14, icon_wifi2, 17, 16, 0xC618);    // WiFi断开图标
    }
  }

  if (Screen_page == 0)
  {
    tft.drawBitmap(24, 14, icon_light1, 17, 16, 0xC618);
    if ((int)Screen_power != (int)power)
    {
      tft.loadFont(font_16);
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d W", (int)Screen_power);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(strbuff1, 44, 14); // wendu

      sprintf(strbuff1, "%d W", (int)power);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(strbuff1, 44, 14); // wendu
      tft.unloadFont();
      Screen_power = power;
    }

    tft.fillRoundRect(136, 19, 15, 7, 3, 0xbdf7); // 圆角矩形x,y,w,h,r
    tft.fillCircle(164, 22, 3, 0x632C);           // 灰点0x632C
    tft.fillCircle(180, 22, 3, 0x632C);           // 灰点

    if (Aqi_PM25_value <= 50) // 优
    {
      tft.fillRoundRect(40, 86, 37, 6, 3, TFT_GREEN); // 圆角矩形x,y,w,h,r
    }
    else if (Aqi_PM25_value <= 100) // 良
    {
      tft.fillRoundRect(40, 86, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    }
    else // 差
    {
      tft.fillRoundRect(40, 86, 37, 6, 3, TFT_RED); // 圆角矩形x,y,w,h,r
    }

    if (Screen_Aqi != Aqi_PM25_value)
    {
      tft.loadFont(font_90);
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d", Screen_Aqi);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(strbuff1, 24, 125);

      sprintf(strbuff1, "%d", Aqi_PM25_value);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(strbuff1, 24, 125);
      Screen_Aqi = Aqi_PM25_value;
      tft.unloadFont();
    }

    tft.loadFont(font_23);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("AQI", 40, 58);

    if (strcmp(screenweath, jsonweath) != 0)
    {
      int strnum = strcmp(screenweath, jsonweath);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawRightString(screenweath, 285, 58, 1); // 天气

      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawRightString(jsonweath, 285, 58, 1); // 天气
      strcpy(screenweath, jsonweath);
    }
    tft.unloadFont();

    tft.fillRect(260, 140, 25, 3, 0x632C); // 温度间隔线

    if (MQTT_weatemp_max != -254)
    {
      if (Screen_weatemp_max != MQTT_weatemp_max)
      {
        tft.loadFont(font_32);
        memset(strbuff1, 0, strlen(strbuff1));
        sprintf(strbuff1, "%d°", Screen_weatemp_max);
        tft.setTextColor(TFT_BLACK, TFT_BLACK);
        tft.drawRightString(strbuff1, 290, 106, 1); // wendu

        sprintf(strbuff1, "%d°", MQTT_weatemp_max);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawRightString(strbuff1, 290, 106, 1); // wendu
        tft.unloadFont();
        Screen_weatemp_max = MQTT_weatemp_max;
      }
    }
    if (MQTT_weatemp_min != -254)
    {
      if (Screen_weatemp_min != MQTT_weatemp_min)
      {
        tft.loadFont(font_32);
        memset(strbuff1, 0, strlen(strbuff1));
        sprintf(strbuff1, "%d°", Screen_weatemp_min);
        tft.setTextColor(TFT_BLACK, TFT_BLACK);
        tft.drawRightString(strbuff1, 290, 166, 1); // wendu

        sprintf(strbuff1, "%d°", MQTT_weatemp_min);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawRightString(strbuff1, 290, 166, 1); // wendu
        tft.unloadFont();
        Screen_weatemp_min = MQTT_weatemp_min;
      }
    }
    // tft.loadFont(font_32);
    // tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // sprintf(strbuff1, "%2d", 8);
    // tft.drawRightString(strbuff1, 290, 106, 1);
    // tft.drawString("°", 295, 106);
    // tft.drawRightString("15", 290, 166, 1);
    // tft.drawString("°", 295, 166);
    // tft.unloadFont();
  }
  if (Screen_page == 1)
  {
    // 页眉
    tft.fillCircle(146, 22, 3, 0x632C);           // 灰点
    tft.fillRoundRect(156, 19, 15, 7, 3, 0xbdf7); // 圆角矩形x,y,w,h,r
    tft.fillCircle(180, 22, 3, 0x632C);           // 灰点0x632C

    tft.loadFont(font_23);
    tft.setTextColor(0xB596, TFT_BLACK);
    tft.drawString("温度", 75, 77);
    tft.drawString("湿度", 210, 77);
    tft.unloadFont();

    if (Screen_temp != ambientTemperature)
    {
      if ((int)Screen_temp != (int)ambientTemperature)
      {
        tft.loadFont(font_64);
        memset(strbuff1, 0, strlen(strbuff1));
        sprintf(strbuff1, "%d", (int)Screen_temp);
        tft.setTextColor(TFT_BLACK, TFT_BLACK);
        tft.drawRightString(strbuff1, 115, 109, 1); // wendu

        sprintf(strbuff1, "%d", (int)ambientTemperature);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawRightString(strbuff1, 115, 109, 1); // wendu
        tft.unloadFont();
      }
      if (Screen_temp != ambientTemperature)
      {
        tft.loadFont(font_32);
        memset(strbuff1, 0, strlen(strbuff1));
        sprintf(strbuff1, ".%d", (int)(Screen_temp * 10) % 10);
        tft.setTextColor(TFT_BLACK, TFT_BLACK);
        tft.drawString(strbuff1, 115, 109 + 24, 1); // wendu

        sprintf(strbuff1, ".%d", (int)(ambientTemperature * 10) % 10);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(strbuff1, 115, 109 + 24, 1); // wendu
        tft.unloadFont();
      }
      Screen_temp = ambientTemperature;
    }

    if (Screen_hum != (int)ambientHumidity)
    {
      tft.loadFont(font_64);
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d", Screen_hum);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawCentreString(strbuff1, 230, 109, 1); // shidu

      sprintf(strbuff1, "%d", (int)ambientHumidity);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawCentreString(strbuff1, 230, 109, 1); // shidu
      tft.unloadFont();

      Screen_hum = (int)ambientHumidity;
    }
    tft.loadFont(font_32);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("%", 270, 109 + 24, 1); // shidu百分比
    tft.unloadFont();
  }
  if (Screen_page == 2)
  {
    {
      // 页眉
      tft.fillCircle(146, 22, 3, 0x632C);           // 灰点
      tft.fillCircle(164, 22, 3, 0x632C);           // 灰点0x632C
      tft.fillRoundRect(180, 19, 15, 7, 3, 0xbdf7); // 圆角矩形x,y,w,h,r
    }

    if (massConcentrationPm2p5 <= 35) // 优
    {
      tft.fillRoundRect(30, 98, 37, 6, 3, TFT_GREEN); // 圆角矩形x,y,w,h,r
    }
    else if (massConcentrationPm2p5 <= 75) // 良
    {
      tft.fillRoundRect(30, 98, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    }
    else // 差
    {
      tft.fillRoundRect(30, 98, 37, 6, 3, TFT_RED); // 圆角矩形x,y,w,h,r
    }

    if (massConcentrationPm10p0 <= 50) // 优
    {
      tft.fillRoundRect(30, 176, 37, 6, 3, TFT_GREEN); // 圆角矩形x,y,w,h,r
    }
    else if (massConcentrationPm10p0 <= 100) // 良
    {
      tft.fillRoundRect(30, 176, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    }
    else // 差
    {
      tft.fillRoundRect(30, 176, 37, 6, 3, TFT_RED); // 圆角矩形x,y,w,h,r
    }

    if (vocIndex <= 50) // 优
    {
      tft.fillRoundRect(185, 98, 37, 6, 3, TFT_GREEN); // 圆角矩形x,y,w,h,r
    }
    else if (vocIndex <= 100) // 良
    {
      tft.fillRoundRect(185, 98, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    }
    else // 差
    {
      tft.fillRoundRect(185, 98, 37, 6, 3, TFT_RED); // 圆角矩形x,y,w,h,r
    }

    if (noxIndex <= 50) // 优
    {
      tft.fillRoundRect(185, 176, 37, 6, 3, TFT_GREEN); // 圆角矩形x,y,w,h,r
    }
    else if (noxIndex <= 100) // 良
    {
      tft.fillRoundRect(185, 176, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    }
    else // 差
    {
      tft.fillRoundRect(185, 176, 37, 6, 3, TFT_RED); // 圆角矩形x,y,w,h,r
    }
    // tft.fillRoundRect(30, 98, 37, 6, 3, 0x5623);      // 圆角矩形x,y,w,h,r
    // tft.fillRoundRect(30, 176, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    // tft.fillRoundRect(185, 98, 37, 6, 3, TFT_ORANGE); // 圆角矩形x,y,w,h,r
    // tft.fillRoundRect(185, 176, 37, 6, 3, 0x5623);    // 圆角矩形x,y,w,h,r
    tft.loadFont(font_16);
    tft.setTextColor(0xB596, TFT_BLACK);
    tft.drawString("PM2.5", 30, 71); // PM2.5
    tft.drawString("PM10", 30, 149); // PM10
    tft.drawString("VOCs", 185, 71); // VOCs
    tft.drawString("NOx", 185, 149); // NOx
    tft.unloadFont();
    tft.loadFont(font_45);
    if (Screen_pm25 != (int)massConcentrationPm2p5)
    {
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d", Screen_pm25);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(strbuff1, 80, 74); // PM2.5

      sprintf(strbuff1, "%d", (int)massConcentrationPm2p5);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(strbuff1, 80, 74); // PM2.5
      Screen_pm25 = (int)massConcentrationPm2p5;
    }
    if (Screen_pm10 != (int)massConcentrationPm10p0)
    {
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d", Screen_pm10);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(strbuff1, 80, 149); // PM10

      sprintf(strbuff1, "%d", (int)massConcentrationPm10p0);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(strbuff1, 80, 149); // PM10
      Screen_pm10 = (int)massConcentrationPm10p0;
    }
    if (Screen_voc != (int)vocIndex)
    {
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d", Screen_voc);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(strbuff1, 228, 74); // VOCs

      sprintf(strbuff1, "%d", (int)vocIndex);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(strbuff1, 228, 74); // VOCs
      Screen_voc = (int)vocIndex;
    }
    if (Screen_nox != (int)noxIndex)
    {
      memset(strbuff1, 0, strlen(strbuff1));
      sprintf(strbuff1, "%d", Screen_nox);
      tft.setTextColor(TFT_BLACK, TFT_BLACK);
      tft.drawString(strbuff1, 228, 149); // NOx

      sprintf(strbuff1, "%d", (int)noxIndex);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(strbuff1, 228, 149); // NOx
      Screen_nox = (int)noxIndex;
    }

    tft.unloadFont();
  }
}
// OTA相关函数
void OTA_updateBin()
{
  WiFiClient UpdateClient;
  httpUpdate.onStart(update_started);     // 当升级开始时
  httpUpdate.onEnd(update_finished);      // 当升级结束时
  httpUpdate.onProgress(update_progress); // 当升级中
  httpUpdate.onError(update_error);       // 当升级失败时
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, OTA_url);
  switch (ret)
  {
  case HTTP_UPDATE_FAILED: // 当升级失败
    Serial.println("[update] Update failed.");
    break;
  case HTTP_UPDATE_NO_UPDATES: // 当无升级
    Serial.println("[update] Update no Update.");
    break;
  case HTTP_UPDATE_OK: // 当升级成功
    Serial.println("[update] Update ok.");
    break;
  }
}
// 当升级开始时，打印日志
void update_started()
{
  Serial.println("CALLBACK:  HTTP update process started");
}
// 当升级结束时，打印日志
void update_finished()
{
  Serial.println("CALLBACK:  HTTP update process finished");
}
// 当升级中，打印日志
void update_progress(int cur, int total)
{
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes[%.1f%%]...\n", cur, total, cur * 100.0 / total);
}
// 当升级失败时，打印日志
void update_error(int err)
{
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void setup() // 设备初始化
{
  Serial.begin(115200, SERIAL_8N1);
  pinMode(screen, OUTPUT);
  pinMode(Rest_K1, INPUT);                            // 输入上拉模式
  attachInterrupt(Rest_K1, Rest_callBack, RISING);   // 使能中断
  pinMode(Touch_K1, INPUT);                           // 输入上拉模式
  attachInterrupt(Touch_K1, Touch_callBack, FALLING); // 使能中断
  getChipidID();                                      // 获取芯片ID 函数
  screen_init();

  Sen55_init();
  Wifi_connect(); // wifi连接

  // client.setServer(mqtt_server, port);  // 设定MQTT服务器与使用的端口，1883是默认的MQTT端口
  // client.setCallback(Mqtt_getcallback); // 收到onenet平台的命令
  // // 初始化ADC
  // adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
  // // 校准ADC
  // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 3300, &adc1_chars);

  // HttpsOTA.begin(OTA_url, server_certificate);

  Watchdog.begin(); // 默认定时器0,10秒超时.

  // RTOS任务创建
  xTaskCreate(task1, "task1_task", TASK1_STK_SIZE, NULL, TASK1_TASK_PRIO, NULL);
  xTaskCreate(task2, "task2_task", TASK2_STK_SIZE, NULL, TASK2_TASK_PRIO, NULL);
  xTaskCreate(task3, "task3_task", TASK3_STK_SIZE, NULL, TASK3_TASK_PRIO, NULL);
  xTaskCreate(task4, "task4_task", TASK4_STK_SIZE, NULL, TASK4_TASK_PRIO, NULL);
  xTaskCreate(task5, "task5_task", TASK5_STK_SIZE, NULL, TASK5_TASK_PRIO, NULL);
  xTaskCreate(Screeen_task, "task6_task", TASK6_STK_SIZE, NULL, TASK6_TASK_PRIO, NULL);
  xTaskCreate(OTA_task, "task7_task", TASK7_STK_SIZE, NULL, TASK7_TASK_PRIO, NULL);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

/*RTOS 任务1 : 看门狗*/
void task1(void *pvParameters)
{
  while (true)
  {
    Watchdog.feed(); // 喂狗.
    // Serial.println("Task1 runing: Watchdog feed.(1s)");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待1s
  }
}

/*RTOS 任务2 : 网络连接*/
void task2(void *pvParameters)
{
  while (true)
  {
    if (WiFi.status() == WL_CONNECTED) // 判断网络是否连接
    {
      wifi_connect_ok = 1;
      // Serial.println(WiFi.localIP()); // 测试使用，后期删除
    }
    else
    {
      wifi_connect_ok = 0;
      screenstate = 1; // 断网开屏
    }
    if (wifi_pdstate) // 判断是否清除WiFi密码
    {
      wifi_pdstate = 0;
      wifi_connect_ok = 0;
      restoreWiFi();
    }

    if (wifi_connect_ok == 1) // wifi连接OK时判断MQTT服务器
    {
      if (client.connected() == 1)
      {
        client.loop();
      }
      else
      {
        reconnect();     // mqtt服务器连接--重连
        screenstate = 1; // 断MQTT开屏
      }
    }
    if (mqtt_flag == 1) // 如果MQTT服务器连接成功
    {
      client.subscribe(TOPIC); // 订阅命令下发主题
    }
    if (wifi_connect_ok == 0) // wifi连接失效
    {
      Serial.printf("WiFi.status=%d", WiFi.status());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待0.25s
  }
  // WL_NO_SHIELD= 255：Wi-Fi 模块未初始化。
  // WL_IDLE_STATUS= 0：Wi-Fi 模块正在初始化或未连接。
  // WL_NO_SSID_AVAIL= 1：没有找到指定的 Wi-Fi 网络。
  // WL_SCAN_COMPLETED= 2：Wi-Fi 扫描已完成。
  // WL_CONNECTED= 3：成功连接到指定的 Wi-Fi 网络。
  // WL_CONNECT_FAILED= 4：连接失败。
  // WL_CONNECTION_LOST= 5：连接丢失。
  // WL_DISCONNECTED= 6：已断开连接。
}

/*RTOS 任务3 ： 传感器读取*/
void task3(void *pvParameters)
{
  bool state = 0;
  while (true)
  {
    Sen55_Read();
    if (wifi_connect_ok) // wifi连接OK
    {
      M_send();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待s
  }
}

/*RTOS 任务4 ： 屏幕状态展示*/
void task4(void *pvParameters)
{
  while (true)
  {
    if (screenstate == 1)
    {
      digitalWrite(screen, HIGH);
    }
    if (screenstate == 0)
    {
      digitalWrite(screen, LOW); // 屏幕判断
    }
    vTaskDelay(700 / portTICK_PERIOD_MS); // 等待0.7s
  }
}

/*RTOS 任务5 ： ADC*/
void task5(void *pvParameters)
{
  int analogOriginaltempV = 0;
  int analogOriginaltempI = 0;
  while (true)
  {
    //   // 读取ADC值
    // uint32_t adc_value = adc1_get_raw(ADC1_CHANNEL_3);
    // // 映射到电压
    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_value, &adc1_chars);
    // changeVoltsValue=adc_value*3000/4096;
    // // 打印ADC值和电压值
    // Serial.printf("ADC Raw: %d, Voltage: %d mV\n", adc_value, voltage);

    // if (analogOriginalnum < 5)
    // {
    //   analogOriginalValueI = analogRead(ADC_I); // 读取ADC原始值
    //   analogOriginalValueV = analogRead(ADC_V); // 读取ADC原始值

    //   analogOriginaltempV += analogOriginalValueV;
    //   analogOriginaltempI += analogOriginalValueI;
    //   analogOriginalnum++;
    // }
    // else
    // {
    //   analogOriginalSumV = analogOriginaltempV / analogOriginalnum;
    //   analogOriginalSumI = analogOriginaltempI / analogOriginalnum;
    //   changeVoltsSumV = analogOriginalSumV * 2950 / 4095.0;
    //   changeVoltsSumI = analogOriginalSumI * 2950 / 4095.0;
    //   // 上传A读取的ADC值:
    //   Serial.printf("ADC%d analog value = %d ,%.1f mV\r\n", analogOriginalnum, analogOriginalSumV, changeVoltsSumV);
    //   Serial.printf("ADC%d analog value = %d ,%.1f mV\r\n", analogOriginalnum, analogOriginalSumI, changeVoltsSumI);
    //   analogOriginaltempV = 0;
    //   analogOriginaltempI = 0;
    //   analogOriginalnum = 0;
    // }
    analogOriginalValueI = analogRead(ADC_I); // 读取ADC原始值
    analogOriginalValueV = analogRead(ADC_V); // 读取ADC原始值
    changeVoltsSumV = analogOriginalValueV * 2900 / 4096.0;
    changeVoltsSumI = analogOriginalValueI * 2950 / 4095.0;
    power = (changeVoltsSumV * 49) / 10 * changeVoltsSumI / 1000 / 1000;
    // Serial.printf("VADC analog value = %d ,%.1f mV VIN = %.1f\r\n", analogOriginalValueV, changeVoltsSumV , (changeVoltsSumV*49)/10);
    // Serial.printf("IADC analog value = %d ,%.1f mV\r\n", analogOriginalValueI, changeVoltsSumI);
    // Serial.printf("power value = %d ,%.2f ,%.1f W\r\n", (int)power, power, power);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待1s
  }
}

/*RTOS 任务6 ： 屏幕展示*/
void Screeen_task(void *pvParameters)
{
  while (true)
  {

    screen_test();

    vTaskDelay(700 / portTICK_PERIOD_MS); // 等待0.7s
  }
}

/*RTOS 任务7 : OTA*/
void OTA_task(void *pvParameters)
{
  while (true)
  {
    if (ota_state)
    {
      ota_state = 0;
      screenstate = 0;
      digitalWrite(screen, LOW); // 屏幕关闭
      OTA_updateBin();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待1s
  }
}