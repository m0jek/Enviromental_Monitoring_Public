 /// NODE Alpha ///
const char* fwUrlBase = "http://blynk.lexxmail.net/.fota/"; /// put your server URL where the *.bin & *.version files are saved in your http ( Apache? ) server
const int FW_VERSION = 2020061401; /// year_month_day_counternumber 2019 is the year, 04 is the month, 17 is the day 01 is the in day release
/// based on: Self-updating OTA firmware for ESP8266 :  https://www.bakke.online/index.php/2017/06/02/self-updating-ota-firmware-for-esp8266/   ///

#define Version "0.14"  /// sketch version ///

/// Blynk related setup ///
/*
 * V0,V1,V2 virtual pins used to store persistant data for NextTimeSMS, LastPriority and SMS_SENT to survive restart after DeepSleep
 * 
 * V10 virtual switch for activate or deactivate RUNning LED controlling (boolean) ledBlink.
 * 
 * V14 WidgetTerminal terminal V14 
 *
 * V20 virtual switch in push button mode, for on demand OTA
 * 
 * V21 RUNning virtual LED
 *
 * V26 virtual switch in push button mode, to activate HTTP file OTA on demand via boolean HTTP_OTA variable that activate in the loop V26
 * 
 */

// Your WiFi credentials.
// Set password to "" for open networks.
String ssid = "<SSID>";
String password = "<PASSWORD>";

// DEEP SLEEP LENGTH in seconds
//const int LENGTH_OF_SLEEP = 900;  // 15 minutes
const int LENGTH_OF_SLEEP = 120; // 2 minutes.
// Blynk Server
String cloudBlynkServer = "<Blynk Server>";
unsigned int BLYNK_SERVER_HARDWARE_PORT = 8080;
// Blynk Auth Code Token
#define NODE_NUMBER      1

#if (NODE_NUMBER == 1)
  #define VPIN_STORE  0  // V0
  String blynk_token      = "<Blynk Token>";
  String NodeName = "Node Alpha";
#elif (NODE_NUMBER == 2)
   #define VPIN_STORE 1  // V1
   String blynk_token      = "<Blynk Token>";
   String NodeName = "Node Beta";
#elif (NODE_NUMBER == 3)
   #define VPIN_STORE 2 // V2
   String blynk_token      = "<Blynk Token";
   String NodeName = "Node Gamma";
#endif

//#define HARDWARE_LED 2 /// GPIO02 /// D4 /// change according where your ESP8266 board LED is connected to 
#define HARDWARE_LED 4 /// GPIO04 ///  D2
#define REDALERT_LED 5 /// GPIO05 /// D1 used to indicate some error condition, e.g not connecting to Blynk server
// Specify data and clock connections and instantiate SHT1x object
#define dataPin  D4
#define clockPin D3

/// blynk dependencies ///
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space

#include <ezTime.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SHT1x.h>
#include "Math.h"

/// OTA dependencies ///
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

/// HTTP OTA dependencies ///
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClient.h>
WiFiClient client;

// Attach virtual serial terminal to Virtual Pin V14
WidgetTerminal terminal(V14);

WidgetLED led0(V21);

// Setup to measure VCC from Battery if connected.
ADC_MODE(ADC_VCC);
// the internal vcc pin appears to under read by a fixed amount
int BattAdjust = 311;

// Sensor function.
SHT1x sht1x(dataPin, clockPin);

/// SimpleTimer timer;
BlynkTimer timer;
  
/// switch value to control if the WeMos internal led will be blinking if HIGH or not if LOW ///
boolean ledBlink = true;

/// timestamp variable to keep track of millis() unsigned long ticks = millis() / 1000;  That is for debugging printings ///
unsigned long timestamp;

/// Have we already sent SMS ?
bool SMS_SENT = false;
long int NextTimeSMS;
long int Priority;
long int LastPriority;
long int Priority1 = 7200;     //  2 hours in seconds
long int Priority2 = 21600;    //  6 hours
long int Priority3 = 43200;    // 12 hours
long int Priority4 = 86400;    // 24 hours

/// This function will run every time Blynk connection is established
BLYNK_CONNECTED() {
  ///   if (isFirstConnect) {
    //     Request Blynk server to re-send latest values for all pins
    /// }
    Blynk.syncAll();
}

/// restoring counter from server
BLYNK_WRITE(VPIN_STORE)
{
  // restoring Time value
  NextTimeSMS = param[0].asLong();
  // restoring Priority value
  LastPriority = param[1].asLong();
  // restoring SMS status
  SMS_SENT = param[2].asInt();
  // Debug
  char msgBuffer[100];
  sprintf(msgBuffer, "Reading back Values NextTimeSMS: %d LastPriority: %d SMS Sent? %d \n", NextTimeSMS, LastPriority, SMS_SENT);
  Blynk.virtualWrite(V14, msgBuffer);
  Serial.print(msgBuffer);
}
/// END routine to restore stored values on server

/// for HTTP based file OTA update ///
bool HTTP_OTA = false; // we control this boolean variable through V26 switch that is in switch mode 

/// START routine for manual (on demand) requesting HTTP file OTA. Virtual Button SW V26 is used ///
BLYNK_WRITE(V26)
 {
   if (param.asInt()) {
   HTTP_OTA = true; 
   } 
   else {
   HTTP_OTA = false;
   }
 }
/// END routine for manual (on demend) requesting HTTP file OTA. Virtual Button SW V15 is used ///

/// start routine for V10 controlling ledBlink boolean global variable ///
 BLYNK_WRITE(V10)
  { 
    if (param.asInt()) {
      ledBlink = true;
      //digitalWrite(HARDWARE_LED, HIGH); /// switch off the internal LED
    } 
    else {
      ledBlink = false;
      //digitalWrite(HARDWARE_LED, LOW); /// switch on the internal LED
      }
  }
/// end of routine for V10 controlling ledBlink boolean global variable ///

/// start routine to print local time.
void PrintMyTime()
{
  Timezone myTZ;
  // get TZ from EEPROM location 0 and if not set set it
  if (!myTZ.setCache(0)) myTZ.setLocation("Europe/London");
  // If one needs to clear the saved TZ, someTZ.clearCache()
  // need to had something later to do this.
  Serial.println("Local Time: " + myTZ.dateTime());
  long int unixepoch = now();
  Serial.println(String("Time in seconds since Midnight, Jan 1 1970: ") + unixepoch);
  Blynk.virtualWrite(V14, "\nLocal Time: " + myTZ.dateTime() + "\n");
}
/// end of routine to print time

/// ESP-8266 DeepSleep Function
void DeepSleep() {
  // Time to deep sleep (in seconds):
  const int sleepTimeS = LENGTH_OF_SLEEP;
  char msgBuffer[100];
  sprintf(msgBuffer, "%s Going to sleep for %d seconds\n", NodeName.c_str(), LENGTH_OF_SLEEP);
  Blynk.virtualWrite(V14, msgBuffer);
  Serial.print(msgBuffer);
  // disconnect from services before deepSleep
  Blynk.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(500);
  ESP.deepSleep(sleepTimeS*1000000, WAKE_RF_DEFAULT);
}
/// end of ESP-8266 DeeepSleep routine

/// DeepSleepNoConnect routine to save battery if there is no WiFI or Blynk server
void DeepSleepNoConnect() {
  // Time to deep sleep (in seconds):
  const int sleepTimeS = LENGTH_OF_SLEEP * 2;
  char msgBuffer[100];
  sprintf(msgBuffer, "%s Going to sleep for %d seconds, no connection.\n", NodeName.c_str(), LENGTH_OF_SLEEP);
  Serial.print(msgBuffer);
  // disconnect from services before deepSleep
  Blynk.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(500);
  ESP.deepSleep(sleepTimeS*1000000, WAKE_RF_DEFAULT);
}
/// end of DeepSleepNoConnect routine.

/// checkStatus routine that checks batteries and sends alerts.
void checkStatus()
{
  // read VCC and convert
  double Batt;
  Batt = (ESP.getVcc() + BattAdjust) / 1000.0;

  // If battery voltage Good then clear counters if not already cleared
  if ((Batt >= 2.900) && (LastPriority > 0)) Blynk.virtualWrite(VPIN_STORE, 0, 0, false);
  
  Blynk.virtualWrite(V7, Batt);
  Serial.println(String("Battery voltage: ") + Batt + ("V"));

  // debug
  Blynk.virtualWrite(V14, "Checking Battery Voltage state\n");
  Serial.println("Checking Battery Voltage state");
  
  // get current time for SMS checks
  long int UNIXTimeNow = now();
  
  // warn of low batt
  if ((Batt < 2.900) && (Batt > 2.601))
  {
    char MsgBuffer[100];
    sprintf(MsgBuffer, "%s Sensor battery low:  %.3f volts", NodeName.c_str(), Batt);
    Blynk.virtualWrite(V14, MsgBuffer);
    // check if SMS has been sent
    if ( SMS_SENT == false ) {
      Priority = Priority4;
      Blynk.sms(MsgBuffer);
      Blynk.notify(MsgBuffer);
      NextTimeSMS = UNIXTimeNow + Priority;
      LastPriority = Priority;
      SMS_SENT = true;
      Blynk.virtualWrite(VPIN_STORE, NextTimeSMS, LastPriority, SMS_SENT);
    } else {
      // if SMS already sent then check whne last sent and reset
       Blynk.virtualWrite(V14, "TXT messge already sent ...\n");
       if ( (UNIXTimeNow - NextTimeSMS) >= 0 ) {
        SMS_SENT = false;
        Blynk.virtualWrite(VPIN_STORE, 0, 0, SMS_SENT);
        Serial.println("Reseting SMS Sent as time passed");
        Blynk.virtualWrite(V14, "Reseting SMS Sent as time passed\n");
       }    
    } // end of if else
  }
  else if (Batt <= 2.600)
  {
    if (LastPriority > Priority1) SMS_SENT = false;
    char MsgBuffer[100];
    sprintf(MsgBuffer, "%s Sensor BATTERY CRITICAL:  %.3f volts", NodeName.c_str(), Batt);
    Blynk.virtualWrite(V14, MsgBuffer);
    // check if SMS has been sent
    if ( SMS_SENT == false ) {
      Priority = Priority1;
      Blynk.sms(MsgBuffer);
      Blynk.notify(MsgBuffer);
      NextTimeSMS = UNIXTimeNow + Priority;
      LastPriority = Priority;
      SMS_SENT = true;
      Blynk.virtualWrite(VPIN_STORE, NextTimeSMS, LastPriority, SMS_SENT);
    } else {
      // if SMS already sent then check whne last sent and reset
       Blynk.virtualWrite(V14, "TXT messge already sent ...\n");
       if ( (UNIXTimeNow - NextTimeSMS) >= 0 ) {
        SMS_SENT = false;
        Blynk.virtualWrite(VPIN_STORE, 0, 0, SMS_SENT);
        Serial.println("Reseting SMS Sent as time passed");
        Blynk.virtualWrite(V14, "Reseting SMS Sent as time passed\n");
       }    
    } // end of if else
  }
} 
/// end of checkStatus routine

/// Read Sensors and send data to Blynk Server
void sendSensor()
{
  float temp_c;
  float humidity;
  
  // debugging info
  Serial.println("Reading sensors ...\n");
  Blynk.virtualWrite(V14, "\nReading the sensors ...\n");
  
  humidity = sht1x.readHumidity();          // Read humidity (percent)
  temp_c = sht1x.readTemperatureC();       // Read temperature as Celsius
  
   // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temp_c)) {
     Serial.println("Failed to read from sensor!");
     Blynk.virtualWrite(V14, "Failed to read from sensor!");
     return;
  }
  
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, humidity);
  Blynk.virtualWrite(V6, temp_c);

  PrintMyTime();

  // for debugging, send to serial
  Serial.println(String("Relative Humidity: ") + humidity + (" %"));
  Serial.println(String("Temperature: ") + temp_c + ("º C"));
  Serial.println(String("------------------------------------"));
  // debug
  Blynk.virtualWrite(V14, "End of Reading sensors\n");

}
/// END of routine that reads and sends data from sensors

/// START of SimpleTimer timer activating function blinkLedWidget ///
void blinkLedWidget()
{
  if (led0.getValue()) {
    led0.off();
    ///BLYNK_LOG("LED0: off");
  } else {
    led0.on();
    ///BLYNK_LOG("LED0: on");
  }
/// put the routine here to blink WeMos LED (D4), using the "control software switch" ledBlink ///
  if (ledBlink) 
  {int test = digitalRead(HARDWARE_LED); /// Could also be : pinMode(BUILTIN_LED, OUTPUT); OR digitalRead(BUILTIN_LED);  /// onboard LED as output
  digitalWrite(HARDWARE_LED, !test);
  }
}
/// END of SimpleTimer timer activating function blinkLedWidget ///

/// Start of main function that performs HTTP OTA /// 
void checkForUpdates() {
  uint8_t mac[6];
  char macAddr[14];
  // reset HTTP OTA buttin
  Blynk.virtualWrite(V26, false);
  WiFi.macAddress( mac );
  ///sprintf(macAddr,"%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); /// small letters at MAC address
  sprintf(macAddr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); /// capital letters at MAC address
  ///terminal.println(macAddr);
  Blynk.virtualWrite(V14, macAddr);
  String fwURL = String( fwUrlBase );
  fwURL.concat( macAddr );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );
  Serial.println( "Checking for firmware updates." );
  Serial.print( "MAC address: " );
  Serial.println( macAddr );
  Blynk.virtualWrite(V14, "\nChecking for firmware updates.\nMAC address: ");
  Blynk.virtualWrite(V14, macAddr);
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );
  Blynk.virtualWrite(V14, "\nFirmware version URL: ");
  Blynk.virtualWrite(V14, fwVersionURL);
  HTTPClient httpClient;
  httpClient.begin( client, fwVersionURL );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();
    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );
    Blynk.virtualWrite(V14, "\nCurrent firmware version: ");
    Blynk.virtualWrite(V14, FW_VERSION);  
    Blynk.virtualWrite(V14, "\nAvailable firmware version: ");
    Blynk.virtualWrite(V14, newFWVersion);
    int newVersion = newFWVersion.toInt();
    if( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );
    Blynk.virtualWrite(V14, "\nPreparing to update"); 
      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      t_httpUpdate_return ret = ESPhttpUpdate.update( client, fwImageURL );
      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          char currentString[64];
          sprintf(currentString, "\nHTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          Blynk.virtualWrite(V14, currentString);
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          Blynk.virtualWrite(V14, "\nHTTP_UPDATE_NO_UPDATES\n");
          break;
      }
    }
    else {
      Serial.println( "Already on latest version" );
      Blynk.virtualWrite(V14, "\nAlready on latest version\n");
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
    char currentString[64];
    sprintf(currentString, "\nFirmware version check failed, got HTTP response code : %d\n",httpCode);
    Blynk.virtualWrite(V14, currentString);
  }
  httpClient.end();
}
/// End of main function that performs HTTP OTA /// 

/// start routine for manual (on demand) requesting OTA The Virtual Button SW V20 is used ///
  BLYNK_WRITE(V20)
  {
    if (param.asInt()) {
    ArduinoOTA.handle();
    }
  }
/// end of routine for manual (on demand) requesting OTA The Virtual Button SW V20 is used ///

/// START OF SETUP ///
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  /// on board LED ///
  pinMode(HARDWARE_LED, OUTPUT);
  /// alert LED
  pinMode(REDALERT_LED, OUTPUT);
  digitalWrite(REDALERT_LED, LOW);

  WiFi.begin(ssid.c_str(), password.c_str());
  if (WiFi.status() != WL_CONNECTED) {
  Serial.println("Connecting to Wifi AP...");
  for ( int i = 0; i < 300; i++) {                  ///       try to connect to WiFi for max 30s
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("it is connected after %d seconds",(i/10));
        break; }
      yield();
      delay(100);
    }
  }
  if(WiFi.status() == WL_CONNECTED) { 
    //if you get here you have connected to the WiFi
    Serial.println("\nconnected...yesss! :)");
  } else {
    Serial.println("TimeOut! Not Connected even after 10 Seconds trying...\n *** Something wrong happened. It did not connected... ***\n going to sleep!");
    digitalWrite(REDALERT_LED, HIGH);
    yield();
    delay(1000);
    DeepSleepNoConnect();
  } 
   
  //if you get here you have connected to the WiFi
  //Serial.println("connected...yesss! :)");

  /// we are connected, lets init time
  setDebug(INFO); // set ezTime debug

  waitForSync();

  Serial.println();
  Serial.println("UTC: " + UTC.dateTime());

  PrintMyTime();
   
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  unsigned long maxMillis=millis() + 50000;
  Blynk.config(blynk_token.c_str(), cloudBlynkServer.c_str(), BLYNK_SERVER_HARDWARE_PORT);
    Serial.println(" Trying to connected to Blynk... ");
    Serial.printf(" Enter millis is: %d\n",millis());
    /// do-while loop 
    do {
        // Wait until connected or 10 seconds passed without connection ( maxMillis )
      if(Blynk.connect() == true) {
        Serial.println(" Blynk is connected !!! ... ");
        Serial.printf(" Break millis is: %d\n",millis());
        break;
        }
    yield();
    delay(200);
    } while ((Blynk.connect() == false) && ( millis() < maxMillis));
    Serial.printf(" Exit millis is: %d\n",millis());
    if(Blynk.connect() == false) {
      Serial.println(" Not Connected to Blynk. Something wrong happens... ");
      digitalWrite(REDALERT_LED, HIGH);
      yield();
      delay(1000);
      DeepSleepNoConnect();
    }
    if(Blynk.connect() == true) {
      Serial.println(" Blynk is connected!!! ... ");
      digitalWrite(REDALERT_LED, LOW);
    }
  
  pinMode(HARDWARE_LED, OUTPUT);  // initialize onboard LED as output  
  digitalWrite(HARDWARE_LED, LOW); // dim the LED 
  Serial.println("Firmware Version: " + FW_VERSION);
  Blynk.virtualWrite(V14, "\nFirmware Version: ");
  Blynk.virtualWrite(V14, FW_VERSION);

}
/// END OF SETUP ///

/// START OF LOOP ///

void loop() {
  if ((WiFi.status() == WL_CONNECTED))
  {
    /// lets see if we put this here, if it works.
  ArduinoOTA.handle();
  if(HTTP_OTA) {
    /// shall I stop timers and Blynk related in order to ensure succesful WEB OTA ??? ///
    checkForUpdates();
    } /// We need this in order to do WEB file OTA update ///   
  Blynk.run();
  timer.run();
  blinkLedWidget();
  sendSensor();
  checkStatus();
  events();
  blinkLedWidget();
  delay(1); // short delay to allow for any events to settle
  DeepSleep();
  }
  else
  {
  Serial.println("Not connected to WiFI, going to sleep ...");
  events();
  delay(1); // short delay to allow for any events to settle
  DeepSleep();
  }
}

/// END OF LOOP ///
