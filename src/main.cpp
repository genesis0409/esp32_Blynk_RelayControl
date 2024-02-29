// ****************************************************************************************************

// 4 Channel Relay Control

// ****************************************************************************************************
// All LEDs are connected Relay 'NO' Connector (Normally Open, not NC)
// This Relay's feature: GPIO: HIGH -> OFF // GPIO: LOW -> ON
// ****************************************************************************************************

#include <Arduino.h>
#include <ArduinoJson.h>

#include "soc/soc.h"          // Disable brownout problems
#include "soc/rtc_cntl_reg.h" // Disable brownout problems
#include "driver/rtc_io.h"

#include <U8x8lib.h>
// #include "TYPE1SC.h"

#include "config_Blynk.h"
#include <BlynkSimpleEsp32.h> // wifi header...

#include <ModbusRTUMaster.h>

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

#define SERIAL_BR 115200

/* TCP/IP, HTTP setting*/
char IPAddr[32];
int _PORT = BLYNK_DEFAULT_PORT;
char sckInfo[128];
char recvBuffer[700];
int recvSize;

// Blynk setting *********************************************
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;
BlynkTimer timer; // 함수 주기적 실행 위한 타이머

#define SCAN_RATE 10000

unsigned long currentMillis = 0;
unsigned long previousMillis = 0; // Stores last time using Reference time

#define RELAY_SIG1 23
#define RELAY_SIG2 15
#define RELAY_SIG3 5
#define RELAY_SIG4 4

// OFF: 1, HIGH
#define LED_OFF 1
#define LED_ON 0
bool led_Relay1 = LED_OFF;
bool led_Relay2 = LED_OFF;
bool led_Relay3 = LED_OFF;
bool led_Relay4 = LED_OFF;

// C++ code
//
void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  Serial.begin(SERIAL_BR);

  pinMode(RELAY_SIG1, OUTPUT);
  pinMode(RELAY_SIG2, OUTPUT);
  pinMode(RELAY_SIG3, OUTPUT);
  pinMode(RELAY_SIG4, OUTPUT);

  digitalWrite(RELAY_SIG1, LED_OFF);
  digitalWrite(RELAY_SIG2, LED_OFF);
  digitalWrite(RELAY_SIG3, LED_OFF);
  digitalWrite(RELAY_SIG4, LED_OFF);

  // Begin Blynk
  Blynk.begin(auth, ssid, pass);
}

void loop()
{
  Blynk.run();
}

// not Blynk, just test blink leds
void autoRelayBlink()
{
  digitalWrite(RELAY_SIG1, LOW);  // ON
  delay(1000);                    // Wait for 1000 millisecond(s)
  digitalWrite(RELAY_SIG1, HIGH); // OFF
  digitalWrite(RELAY_SIG2, LOW);  // ON
  delay(1000);                    // Wait for 1000 millisecond(s)
  digitalWrite(RELAY_SIG2, HIGH); // OFF
  digitalWrite(RELAY_SIG3, LOW);  // ON
  delay(1000);                    // Wait for 1000 millisecond(s)
  digitalWrite(RELAY_SIG3, HIGH); // OFF
  digitalWrite(RELAY_SIG4, LOW);  // ON
  delay(1000);                    // Wait for 1000 millisecond(s)
  digitalWrite(RELAY_SIG4, HIGH); // OFF
}
BLYNK_WRITE(V0)
{
  bool pinValue = param.asInt();
  pinValue = !pinValue; // If receive '0' from Blynk, Inverse it: off / 1 -> 0: on

  digitalWrite(RELAY_SIG1, pinValue);
  String led_status = pinValue ? "OFF" : "ON";
  Serial.printf("(V0)Relay 1: %s\r\n", led_status);
}
BLYNK_WRITE(V1)
{
  bool pinValue = param.asInt();
  pinValue = !pinValue; // If receive '0' from Blynk, Inverse it: off / 1 -> 0: on

  digitalWrite(RELAY_SIG2, pinValue);
  String led_status = pinValue ? "OFF" : "ON";
  Serial.printf("(V1)Relay 2: %s\r\n", led_status);
}
BLYNK_WRITE(V2)
{
  bool pinValue = param.asInt();
  pinValue = !pinValue; // If receive '0' from Blynk, Inverse it: off / 1 -> 0: on

  digitalWrite(RELAY_SIG3, pinValue);
  String led_status = pinValue ? "OFF" : "ON";
  Serial.printf("(V2)Relay 3: %s\r\n", led_status);
}
BLYNK_WRITE(V3)
{
  bool pinValue = param.asInt();
  pinValue = !pinValue; // If receive '0' from Blynk, Inverse it: off / 1 -> 0: on

  digitalWrite(RELAY_SIG4, pinValue);
  String led_status = pinValue ? "OFF" : "ON";
  Serial.printf("(V3)Relay 4: %s\r\n", led_status);
}