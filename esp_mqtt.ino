#include <SoftwareSerial.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#define _baudrate 9600
#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int

//函数声明
Adafruit_AM2320 am2320 = Adafruit_AM2320();
void keepAlive();
void send_mqtt_dht();
void SerialEvent();
void autoopen();
int MODE = 1;
u8 mqtt_msg[200] = {0};
int val = 0; //定义数字变量
float TT = 0;
float HH = 0;
boolean home = 1; //0 outside  1 home
boolean bibi = 0;
boolean redray = 0;
boolean stringcomplete = false;
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
unsigned long currentMillis=0;
unsigned long previousMillis = 0;  //will store last time LED was blinked
const long period = 2000;         // period at which to blink in ms

void setup()
{
    am2320.begin();
    pinMode(13, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(A1, INPUT);
    pinMode(6, INPUT);
    digitalWrite(7, LOW);
    //设置波特率
    Serial.begin(_baudrate);
}

void loop()
{
    currentMillis=millis();
    send_mqtt_dht();
    autoopen();
    SerialEvent();
    
}

void SerialEvent()
{
    while (Serial.available())
    {
        int c = Serial.parseInt();
        //  Serial.print("读取到的整数：");
        //  Serial.println(c);
        if (c == 1111)
        {
            bibi = 1;
            //Serial.println("open");
            digitalWrite(7, HIGH);
        }

        if (c == 1100)
        {
            bibi = 0;
            // Serial.println("close");
            digitalWrite(7, LOW);
        }

        if (c == 2222)
        { //back home
            home = 1;
        }

        if (c == 2200)
        {
            home = 0;
        }
    }
}
void autoopen()
{
    if (home == 0 && redray == 1)
    {
        bibi = 1;
        digitalWrite(7, HIGH);
    }
    if (home ==1)
    {
        bibi = 0;
        digitalWrite(7, LOW);
    }
}
void updateDHT11_mqtt()
{
    //Serial.println("读取传感器数据完成，准备上传");
    String cmd = "";
    u16 len;
    u16 i;
    cmd = "{\"Temperature\":";
    cmd += TT;
    cmd += ",\"Humidity\":";
    cmd += HH;
    cmd += ",\"Red-ray\":";
    cmd += redray;
    cmd += ",\"Bibi\":";
    cmd += bibi;
    cmd += ",\"Home\":";
    cmd += home;
    cmd += ",\"PM\":";
    cmd += dustDensity;
    cmd += "}";
    char mg[100];
    strcpy(mg, cmd.c_str());
    Serial.println(mg);
}
void send_mqtt_dht()
{
    digitalWrite(9, LOW);
    delayMicroseconds(280);

    voMeasured = analogRead(A1);

    delayMicroseconds(40);
    digitalWrite(9, HIGH);
    delayMicroseconds(9680);

    calcVoltage = voMeasured * (5.0 / 1024); //电压
    dustDensity = 0.17 * calcVoltage - 0.1;  //粉尘密度

    if (dustDensity < 0)
    {
        dustDensity = 0.00;
    }

    HH = am2320.readHumidity();
    TT = am2320.readTemperature();
    redray = digitalRead(6);
    delay(1200);
    updateDHT11_mqtt();
}
