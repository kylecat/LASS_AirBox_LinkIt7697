/*
   Name:    AirBox_LASS_online_Ver1.0.ino
   Created: 2020/2/11 下午 02:56:07
   Author:  Liu
*/
#include <LWiFi.h>
#include <WiFiUdp.h>
#include <LRTC.h>
#include "UTC_converter.h"

bool _LoopCount;

/*_____ 數字轉換funciton______*/
String padding(int _d)
{
  String _str;
  if (_d < 10) _str = "0" + String(_d);
  else _str = String(_d);
  return _str;
}

bool compare(int _a, int _b)
{
  bool _result = true;
  if (_a != _b)
  {
    _result = false;
    Serial.println("錯誤: " + String(_a) + "-" + String(_b));
  }
  return _result;
}


/*_____ 通訊用funciton______*/
const char* ssid = "KyleiPhoneXR";                                 //  your network SSID (name)
const char* pass = "11115555";                                     // your network password
int keyIndex = 0;                                                   // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;                               // Wifi 狀態

// 連線到wifi
void connectWifi()
{
  while (status != WL_CONNECTED) {

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    if (_LoopCount > 3)   // 設定跳出while的條件
    {
      _LoopCount = 0;
      break;
    }
    _LoopCount++;
  }
}
// 把目前wifi的狀態印出來
void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


/*_____ 取得網路時間用 ______*/
unsigned int localPort = 2390;                                // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28);                   // time.nist.gov NTP server
const char* NTP_server = "time-a.nist.gov";

const int NTP_PACKET_SIZE = 48;                         // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];                 // buffer to hold incoming and outgoing packets

WiFiUDP Udp;                                                      // A UDP instance to let us send and receive packets over UDP
UTC_Converter cUTC(true);

// 取得NTP的封包
unsigned long sendNTPpacket(const char* host)
{
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(host, 123); //NTP requests are to port 123

  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);

  //Serial.println("5");
  Udp.endPacket();

  //Serial.println("6");
  return 0;
}
// 取得NTP的資料同時更新
void getNTP()
{
  Udp.begin(localPort);
  sendNTPpacket(NTP_server); // send an NTP packet to a time server
  delay(5000);

  if (Udp.parsePacket())
  {
    Serial.println("packet received");                  // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    unsigned long secs1900 = calculateNTPTime();
    cUTC.setUTC(secs1900);
    //cUTC.setTimeZone(8);
  }
  else
  {
    Serial.println("No Packet recevied");
  }
}
// 計算出自1900.1.1 00:00:00以來的秒數(NTP都統一用這當基準點)
unsigned long calculateNTPTime()
{
  // the timestamp starts at byte 40 of the received packet and is four bytes,
  // or two words, long. First, esxtract the two words:
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;
  //Serial.print("Seconds since Jan 1 1900 = ");
  //Serial.println(secsSince1900);


  // now convert NTP time into everyday time:
  //Serial.print("Unix time = ");
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:

  //const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  //unsigned long _epoch = secsSince1900 - seventyYears;
  // print Unix time:
  //Serial.println(_epoch);

  return secsSince1900;
}
// 顯示NTP的內容，_new 控制要不要再更新一次NTP
void showNTP(bool _new = false)
{
  if (_new)  getNTP();
  Serial.print(cUTC.year);	Serial.print("/");	Serial.print(cUTC.month);	Serial.print("/");	Serial.print(cUTC.day);
  Serial.print(" ");
  Serial.println(padding(cUTC.hour) + ":" + padding(cUTC.minute) + ":" + padding(cUTC.second));

  Serial.print(cUTC.WeeksOfYear);
  Serial.print(" Weeks and ");
  Serial.print(cUTC.DaysOfYear);
  Serial.print(" Days in this year, ");
  Serial.print("Day of the week: ");
  Serial.println(cUTC.DaysOfWeek);
}
// 檢察NTP跟系統時間，_reset 控制要不要更新系統時間
bool checkNTP(bool _reset = false)
{
  bool _result = true;

  getNTP();
  LRTC.get();

  _result = _result && compare(cUTC.year, LRTC.year());
  _result = _result && compare(cUTC.month, LRTC.month());
  _result = _result && compare(cUTC.day, LRTC.day());
  _result = _result && compare(cUTC.hour, LRTC.hour());
  _result = _result && compare(cUTC.minute, LRTC.minute());
  _result = _result && compare(cUTC.second, LRTC.second());

  if (_reset && !_result) {
    Serial.println("重設系統時間");
    LRTC.set(cUTC.year, cUTC.month, cUTC.day, cUTC.hour, cUTC.minute, cUTC.second);
    showNTP();
  }
}


/*_____ PMS5003T function ______*/
#include <SoftwareSerial.h>
SoftwareSerial myMerial(3, 2);

long pmat10 = 0;
long pmat25 = 0;
long pmat100 = 0;
long Temp = 0;
long Humid = 0;
char buf[50];
// 讀取資料並更新變數
void retrievepm25()
{
  int count = 0;
  unsigned char c;
  unsigned char high;

  while (myMerial.available())
  {
    c = myMerial.read();
    if ((count == 0 && c != 0x42) || (count == 1 && c != 0x4d)) {
      break;
    }
    if (count > 27) {
      break;
    }
    else if (count == 10 || count == 12 || count == 14 || count == 24 || count == 26) {
      high = c;
    }
    else if (count == 11) {
      pmat10 = 256 * high + c;
    }
    else if (count == 13) {
      pmat25 = 256 * high + c;
    }
    else if (count == 15) {
      pmat100 = 256 * high + c;
    }
    else if (count == 25) {
      Temp = (256 * high + c) / 10;
    }
    else if (count == 27) {
      Humid = (256 * high + c) / 10;
    }
    count++;
  }
  while (myMerial.available()) myMerial.read();
}


/*_____ OLED function ______*/
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);   // ESP32 Thing, HW I2C with pin remapping
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void showData()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "LinkIt AirBox");
  u8g2.drawStr(0, 25, ("PM2.5 : " + String(pmat25) + " ug/m3").c_str());
  u8g2.drawStr(0, 37, ("PM10 : " + String(pmat100) + " ug/m3").c_str());
  u8g2.drawStr(0, 49, ("Temp : " + String(Temp) + " *C").c_str());
  u8g2.drawStr(0, 61, ("Humid : " + String(Humid) + " %RH").c_str());
  u8g2.sendBuffer();
}

/*_____ MQTT function ______*/
#include <PubSubClient.h>

//#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_SERVER"gpssensor.ddns.net"
#define MQTT_SERVER_PORT 1883
#define MQTT_CLIENT_ID "Air_123"
#define MQTT_SUB_TOPIC "LASS/Test/PM25/NTCU"

bool MQTTconnect;

WiFiClient  wifiClient;
PubSubClient mqtt(MQTT_SERVER, MQTT_SERVER_PORT, callback, wifiClient);

void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect(char *_topic, String _payload) {
  mqtt.disconnect();
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(MQTT_CLIENT_ID))
    {
      Serial.println("connected");
      mqtt.publish(_topic, _payload.c_str());
      _LoopCount=0;
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    if (_LoopCount % 2 == 0) connectWifi();
    if (_LoopCount > 10) break;
    _LoopCount++;
  }
}

/*_____ MQTT 資料格式 ______
  | ver_format = 3					// 版本格式
  | FAKE_GPS = 1						// 手動輸入GPS
  | app = PM25		          // app版本
  | ver_app = 0.7.13.a   		//	app版本編號
  | device_id = FT3_061_C		// 設備ID
  | date = 2019 - 09 - 15		// 上傳日期
  | time = 04:34 : 50				// 上傳時間
  | device = LinkIt7697			// 設備名稱
  | s_d0 = 7                // PM10.0
  | s_d1 = 7								// PM2.5
  | s_d2 = 6								// PM1.0
  | s_t0 = 22							  // 溫度
  | s_h0 = 63						    // 濕度
  | gps_lon = 123.544053		// 座標：經度
  | gps_lat = 25.781129			// 座標：緯度
*/
#define DEVICE_ID "NTCU_"
#define GPS_lon     "123.544053"
#define GPS_lat      "25.781129"

String PackedData(long _pm100, long _pm25, long _pm10, long _t, long _h)
{
  return "|s_d0=" + String(_pm100) + "|s_d0=" + String(_pm25) + "|s_d2=" + String(_pm10) + "|s_t0=" + String(_t) + "|s_h0=" + String(_h);
}

String PackedDate(int _y, int _M, int _d, int _h, int _m, int _s)
{
  return "|date=" + String(_y) + "-" + padding(_M) + "-" + padding(_d) + "|time=" + padding(_h) + ":" + padding(_m) + ":" + padding(_s);
}

String PackedMQTTpayload(String _datetime, String _data)
{
  String _payload = "|ver_format=3|FAKE_GPS=1|app=PM25|ver_app=0.7.13.a|device_id=" + String(DEVICE_ID) + String(MQTT_CLIENT_ID);

  _payload += _datetime;
  _payload += _data;
  _payload = _payload + "|gps_lon=" + GPS_lon + "|gps_lat=" + GPS_lat + "|gps_fix=1|gps_num=9|gps_alt=2";

  return _payload;
}


void setup()
{
  Serial.begin(9600);                            // 設定 Serial的brand

  myMerial.begin(9600);

  u8g2.begin();

  connectWifi();

  LRTC.begin();
  checkNTP(true);

  reconnect(MQTT_SUB_TOPIC, "Hello MQTT");

  Serial.println("Setup Done");
}


/*_____ Loop用的設定______*/
bool _wasUpload = false;

void loop()
{
  String _msg;
  String _datetime;
  String _data;

  retrievepm25();
  showData();

  LRTC.get();
  if (LRTC.minute() % 5 == 0)
  {
    if (_wasUpload)
    {

      _data = PackedData(pmat100, pmat25, pmat10, Temp, Humid);
      _datetime = PackedDate(LRTC.year(), LRTC.month(), LRTC.day(), LRTC.hour(), LRTC.minute(), LRTC.second());
      _msg = PackedMQTTpayload(_datetime, _data);
      Serial.println(_msg.length());

      reconnect(MQTT_SUB_TOPIC, _msg);
    }
    _wasUpload = false;
  }
  else
  {
    _wasUpload = true;
  }

  if (LRTC.hour() == 12 && LRTC.minute() == 0 && LRTC.second() < 30)
  {
    checkNTP(true);
  }
}
