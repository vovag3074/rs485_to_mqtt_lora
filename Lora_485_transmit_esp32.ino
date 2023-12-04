/*
 * LoRa E32-TTL-100
 * Start device, reset or write to the Serial to send a message.
 * https://www.mischianti.org/2019/10/15/lora-e32-device-for-arduino-esp32-or-esp8266-specs-and-basic-usage-part-1/
 *
 * E32-TTL-100----- Arduino UNO
 * M0         ----- PIN 11 - расширителя портов
 * M1         ----- PIN 12 - расширителя портов
 * TX         ----- PIN 16 (PullUP)
 * RX         ----- PIN 17 (PullUP)
 * AUX        ----- None
 * VCC        ----- 3.3v/5v
 * GND        ----- GND
 *
 */
// 07-апр-2023 решил добавить поддержку mqtt протокола
//  брокер будет находиться на этой esp32
#define TINY_MQTT_DEBUG 0
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <driver/adc.h>
#include <Wire.h>
#include <SPI.h>
#include <DFRobot_MCP23017.h> // модифицирован для I2C на ножках 27, 26 и адресс = 21
//#include <Adafruit_MCP23X17.h>  // моторолла 23017 -- после последнего обновления не работает
#include "Arduino.h"
#define E32_TTL_500 ;
#define FREQUENCY_868
#include "LoRa_E32.h"
#include <ArduinoJson.h>
#include <TimeLib.h>
#include "TinyMqtt.h"
#include "FS.h"
#include <LittleFS.h>
#include <EEPROM.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <ESPmDNS.h>

DFRobot_MCP23017 mcp;                                         // Это расширитель портов

#define RX2_PIN (17)
#define TX2_PIN (16)
#define SDA_PIN (27)
#define SCL_PIN (26)
#define RX1_PIN (4)  // используется контроллером дисплея
#define TX1_PIN (5)
//-------выводы расширителя портов-----------------
#define RED_LED_PIN2 (mcp.eGPB0)
#define GREEN_LED_PIN2 (mcp.eGPB2)
#define BLUE_LED_PIN2 (mcp.eGPB1)
#define M0_PIN2 (mcp.eGPB3)
#define M1_PIN2 (mcp.eGPB4)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200; // gmt+2 - переопределим для настройки
const int   daylightOffset_sec = 3600; // +1 hour letom
//-------определяем цвета для светодиода
const int BLACK = 0;
const int RED = 1;
const int GREEN = 2;
const int BLUE = 3;
const int YELLOW = 4;
const int VIOLET = 5;

#define FORMAT_LITTLEFS_IF_FAILED true
#define HOSTNAME "Rs486ToMqtt"
#define FORCE_USE_HOTSPOT 0

String ts;
uint16_t wifi_ssid_text, wifi_pass_text;
int millisLabelId;
int sourceSend; // источник сообщения 1=LORA, 2=RS485, 3=MQTT

MqttBroker broker(1883);
MqttClient mqtt_a(&broker);
HardwareSerial MySerial(2);   // Lora
HardwareSerial keySerial(1);  //rs485

LoRa_E32 e32ttl100(TX2_PIN, RX2_PIN, &MySerial, UART_BPS_RATE_9600);  // Config without connect AUX and M0 M1
//Adafruit_MCP23X17 mcp;       

// -------------------------------------

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

String readFile(fs::FS& fs, const char* path) {
  String str = "";
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return str;
  }
  while (file.available()) {
    str = file.readString();
  }
  file.close();

  return str;
}

void writeFile(fs::FS& fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}


void deleteFile(fs::FS& fs, const char* path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}


void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD BIN: "));
  Serial.print(configuration.HEAD, BIN);
  Serial.print(" ");
  Serial.print(configuration.HEAD, DEC);
  Serial.print(" ");
  Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH BIN: "));
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL BIN: "));
  Serial.println(configuration.ADDL, BIN);
  Serial.print(F("Chan BIN: "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit BIN    : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDataRate BIN : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate BIN  : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans BIN       : "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup BIN      : "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup BIN      : "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC BIN         : "));
  Serial.print(configuration.OPTION.fec, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower BIN       : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
}

void initLORA_E32() {
  // модуль LORA
  mcp.pinMode(M0_PIN2, OUTPUT);
  mcp.pinMode(M1_PIN2, OUTPUT);
  // настраиваем модуль LORA
  mcp.digitalWrite(M0_PIN2, HIGH);
  mcp.digitalWrite(M1_PIN2, HIGH);
  // ======================================
  delay(500);
  e32ttl100.begin();
  ResponseStructContainer c;
  c = e32ttl100.getConfiguration();
  Configuration configuration = *(Configuration*)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
  printParameters(configuration);
  configuration.ADDL = 0xFF;
  configuration.ADDH = 0xFF;  // работаем в широковещательном режиме
  configuration.CHAN = 0x10;
  configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
  configuration.OPTION.fec = FEC_1_ON;
  configuration.OPTION.ioDriveMode = IO_D_MODE_OPEN_COLLECTOR;
  configuration.OPTION.transmissionPower = POWER_27;
  configuration.SPED.airDataRate = AIR_DATA_RATE_111_192;
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  ResponseStatus rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);
  printParameters(configuration);
  ResponseStructContainer cMi;
  cMi = e32ttl100.getModuleInformation();
  //ModuleInformation mi = *(ModuleInformation*)cMi.data;
  Serial.println(cMi.status.getResponseDescription());
  Serial.println(cMi.status.code);
  //printModuleInformation(mi);
  c.close();
  cMi.close();
  delay(500);
  // LORA переходит в режим приема-передачи
  mcp.digitalWrite(M0_PIN2, LOW);
  mcp.digitalWrite(M1_PIN2, LOW);
  delay(500);
}

void initLED() {
  mcp.pinMode(RED_LED_PIN2, OUTPUT);
  mcp.pinMode(GREEN_LED_PIN2, OUTPUT);
  mcp.pinMode(BLUE_LED_PIN2, OUTPUT);

  mcp.digitalWrite(RED_LED_PIN2, LOW);
  mcp.digitalWrite(GREEN_LED_PIN2, LOW);
  mcp.digitalWrite(BLUE_LED_PIN2, LOW);
}

void setLED(int myColor) {
  switch (myColor) {
    case 1:  // Красный
      {
        mcp.digitalWrite(RED_LED_PIN2, HIGH);
        mcp.digitalWrite(GREEN_LED_PIN2, LOW);
        mcp.digitalWrite(BLUE_LED_PIN2, LOW);
        break;
      }
    case 2:  // Зеленный
      {
        mcp.digitalWrite(RED_LED_PIN2, LOW);
        mcp.digitalWrite(GREEN_LED_PIN2, HIGH);
        mcp.digitalWrite(BLUE_LED_PIN2, LOW);
        break;
      }
    case 3:  // Синий
      {
        mcp.digitalWrite(RED_LED_PIN2, LOW);
        mcp.digitalWrite(GREEN_LED_PIN2, LOW);
        mcp.digitalWrite(BLUE_LED_PIN2, HIGH);
        break;
      }
    case 4:  // Желтый
      {
        mcp.digitalWrite(RED_LED_PIN2, HIGH);
        mcp.digitalWrite(GREEN_LED_PIN2, HIGH);
        mcp.digitalWrite(BLUE_LED_PIN2, LOW);
        break;
      }
    case 5:  // Фиолетовый
      {
        mcp.digitalWrite(RED_LED_PIN2, HIGH);
        mcp.digitalWrite(GREEN_LED_PIN2, LOW);
        mcp.digitalWrite(BLUE_LED_PIN2, HIGH);
        break;
      }
    default:  // Черный
      {
        //любой нестандартный цвет = черный;
        mcp.digitalWrite(RED_LED_PIN2, LOW);
        mcp.digitalWrite(GREEN_LED_PIN2, LOW);
        mcp.digitalWrite(BLUE_LED_PIN2, LOW);
        break;
      }
  }
}

void setMyTime(int mHour, int mMin, int mSec, int mDay, int mMon, int mYear) {
  setTime(mHour, mMin, mSec, mDay, mMon, mYear);
}

void getMyTime() {
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
}

void printLocalTime()
{
  getMyTime();
}


void getTimeNTP()
{
 
    // читаем время
    Serial.println("Time List from network");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    yield();

}

void onPublishA(const MqttClient*, const Topic& topic, const char* payload, size_t) {
  Serial << "--> read from Mqtt " << topic.c_str() << ", " << payload << endl;
  if (String(payload).length() > 0) 
  {
    setLED(BLUE);
    parse(String(payload));
      keySerial.println(ts);
      Serial.println(ts);
      e32ttl100.sendMessage(String(payload));
      if (sourceSend != 3)
      {
       sourceSend = 3; 
       if (ts.length()>0)
        {
         mqtt_a.publish("scanners/rfid", ts);
        }
      }
      else
      {
        sourceSend = 0;
        ts = "";
      }
      ESPUI.print(millisLabelId, String(payload));
    setLED(BLACK);
  }
}

void readStringFromEEPROM(String& buf, int baseaddress, int size) {
  buf.reserve(size);
  for (int i = baseaddress; i < baseaddress + size; i++) {
    char c = EEPROM.read(i);
    buf += c;
    if (!c) break;
  }
}

void connectWifi() {
  int connect_timeout;
  WiFi.setHostname(HOSTNAME);
  Serial.println("Begin wifi...");

  //Load credentials from EEPROM
  if (!(FORCE_USE_HOTSPOT)) {
    yield();
    EEPROM.begin(100);
    String stored_ssid, stored_pass;
    readStringFromEEPROM(stored_ssid, 0, 32);
    readStringFromEEPROM(stored_pass, 32, 96);
    EEPROM.end();

    //Try to connect with stored credentials, fire up an access point if they don't work.
    WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
    connect_timeout = 28;  //7 seconds
    while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) {
      delay(250);
      Serial.print(".");
      connect_timeout--;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    Serial.println("Wifi started");

    if (!MDNS.begin(HOSTNAME)) {
      Serial.println("Error setting up MDNS responder!");
    }
  } else {
    Serial.println("\nCreating access point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(HOSTNAME);

    connect_timeout = 20;
    do {
      delay(250);
      Serial.print(",");
      connect_timeout--;
    } while (connect_timeout);
  }
}

void enterWifiDetailsCallback(Control* sender, int type) {
  if (type == B_UP) {
    Serial.println("Saving credentials to EPROM...");
    Serial.println(ESPUI.getControl(wifi_ssid_text)->value);
    Serial.println(ESPUI.getControl(wifi_pass_text)->value);
    unsigned int i;
    EEPROM.begin(100);
    for (i = 0; i < ESPUI.getControl(wifi_ssid_text)->value.length(); i++) {
      EEPROM.write(i, ESPUI.getControl(wifi_ssid_text)->value.charAt(i));
      if (i == 30) break;  //Even though we provided a max length, user input should never be trusted
    }
    EEPROM.write(i, '\0');

    for (i = 0; i < ESPUI.getControl(wifi_pass_text)->value.length(); i++) {
      EEPROM.write(i + 32, ESPUI.getControl(wifi_pass_text)->value.charAt(i));
      if (i == 94) break;  //Even though we provided a max length, user input should never be trusted
    }
    EEPROM.write(i + 32, '\0');
    EEPROM.end();
  }
}

void textCallback(Control* sender, int type) {
  //This callback is needed to handle the changed values, even though it doesn't do anything itself.
}

void setupUI() {
  if (WiFi.status() != WL_CONNECTED) {
    auto wifitab = ESPUI.addControl(Tab, "", "Настройка WiFi");
    wifi_ssid_text = ESPUI.addControl(Text, "Сеть", "", Alizarin, wifitab, textCallback);
    //Note that adding a "Max" control to a text control sets the max length
    ESPUI.addControl(Max, "", "32", None, wifi_ssid_text);
    wifi_pass_text = ESPUI.addControl(Text, "Пароль", "", Alizarin, wifitab, textCallback);
    ESPUI.setInputType(wifi_pass_text, "password");
    ESPUI.addControl(Max, "", "64", None, wifi_pass_text);
    ESPUI.addControl(Button, "Save", "Save", Peterriver, wifitab, enterWifiDetailsCallback);
  }
  millisLabelId = ESPUI.label("Последний запрос:", ControlColor::Emerald, "0");
  ESPUI.setPanelWide(millisLabelId, true);
  //Finally, start up the UI.
  //This should only be called once we are connected to WiFi.
  ESPUI.begin("RS486ToMQTT");
}

void setup() {
  Serial.begin(9600);
  keySerial.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
  sourceSend = 0;
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  if (mcp.begin()!= 0)  // 21 адрес
  {
    Serial.println("I2C expander error!");
  } else
  {
   Serial.println("I2C OK");
  }
  initLED();
  setLED(BLUE);
  delay(500);
  setLED(RED);
  delay(500);
  setLED(GREEN);
  delay(500);
  setLED(BLACK);
  delay(500);
  initLORA_E32();
  Serial.println("Hi, I'm going to send message Hello!");
  // Send message
  e32ttl100.sendMessage("{Command:10, ID:\"0\"}\n");

  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  connectWifi();
  setupUI();

  broker.begin();
  Console << "Broker ready : " << WiFi.localIP() << endl;

  mqtt_a.setCallback(onPublishA);
  mqtt_a.subscribe("scanners/rfid");
  responseHello();
  mqtt_a.publish("scanners/rfid", ts);

 getTimeNTP();  
}

void responseHello() {
  //{Command:20,Hour:16,Minute:40,Second:00,Day:03,Month:05,Year:2021}
  ts = "{Command:20,Hour:" + String(hour()) + ",Minute:" + String(minute()) + ",Second:" + String(second()) + ",Day:" + String(day()) + ",Month:" + String(month()) + ",Year:" + String(year()) + "} \n";
}

void parse(String s) {
  if (s == "")
   {return;}
  DynamicJsonDocument doc(200);
  DeserializationError error = deserializeJson(doc, s);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    ts = "";
    return;
  }
  // {Command:10,ID:"3074"}

  int cmd = doc["Command"];

  if (cmd == 20) {
    int mH = doc["Hour"];
    int mM = doc["Minute"];
    int mS = doc["Second"];
    int mD = doc["Day"];
    int mMt = doc["Month"];
    int mY = doc["Year"];
    setMyTime(mH, mM, mS, mD, mMt, mY);
    Serial.println("Set Time OK");
    getMyTime();
    ts = "";
  }
  if (cmd == 5)  // если приходит приветствие, то передаем время
  {
    responseHello();
  }
  if (cmd == 10)  // если спрашиваем о времени, то меняем ответ на
  {
    responseHello();
  }
 }

void loop() {
  broker.loop();
  mqtt_a.loop();

  String c;
  int i, j;
  setLED(BLACK);
  // If something available
  if (e32ttl100.available() > 1) {
    // read the String message
    ResponseContainer rc = e32ttl100.receiveMessage();
    setLED(BLUE);
    // Is something goes wrong print error
    if (rc.status.code != 1) {
      rc.status.getResponseDescription();
    } else {
      // Print the data received
      // считаем, что запрос о времени может прийти только по радиоканалу
      c = rc.data;
      i = c.indexOf("}");
      if (i == -1) {
        ts = ts + c;
      } else {
        ts = ts + c;
        Serial << "--> read from e32 Lora" << endl;
        parse(ts);
        keySerial.println(ts);
        Serial.println(ts);
        mqtt_a.publish("scanners/rfid", ts);
        ts = "";
      }
    }
  } else if (keySerial.available()) {
    setLED(GREEN);
    sourceSend = 2;
      String input = keySerial.readString();
      i = c.indexOf("}");
      if (i != -1)
      {
      Serial << "--> read from rs432" << endl;
      parse(input);
      keySerial.println(ts);
      Serial.println(ts);
      e32ttl100.sendMessage(input);
      mqtt_a.publish("scanners/rfid", ts);
      ts = "";
       sourceSend = 0;
      }
  } else if (Serial.available()) {
    setLED(RED);
    sourceSend = 1;
      String input = Serial.readString();
      Serial << "--> read from USB" << endl;
      parse(input);
      keySerial.println(ts);
      Serial.println(ts);
      e32ttl100.sendMessage(input);
      mqtt_a.publish("scanners/rfid", ts);
      ts = "";
  }
}
