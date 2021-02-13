#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include <MR_GPIO.h>
#include <MR_SPI.h>



uint16_t apTimeout = 1; //AP Timeout to automatically load saved config data
const IPAddress outIp(10,0,0,231);        // remote IP (not needed for receive)
const unsigned int outPort = 10111;          // remote port (not needed for receive)
const unsigned int localPort = 10101;        // local port to listen for UDP packets (here's where we send the packets)

//AP Variables
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())
String ssid = "MORAD_" + String(ESP_getChipId(), HEX);
//String ssid = "Motivation Radio" ;
const char* password = "MORAD";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h> 
#define USE_AVAILABLE_PAGES     true
              
#include <ESP_WiFiManager.h>  //https://github.com/khoih-prog/ESP_WiFiManager

#define ESP_DRD_USE_EEPROM      false
#define ESP_DRD_USE_SPIFFS      true    //false

#include <WiFiUdp.h>

MR_SPI mr_spi;
MR_GPIO mr_gpio;

// I2C pins
#define SDA 21
#define SCL 22

//DisplayStuff
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Gate output message handlers - TouchOSC push buttons
void gateoutA(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_gpio.GATEout(0,(bool)val);
}
void gateoutB(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_gpio.GATEout(1,(bool)val);
}
void gateoutC(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_gpio.GATEout(2,(bool)val);
}
void gateoutD(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_gpio.GATEout(3,(bool)val);
}

void CVoutA(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_spi.CVout(0,(unsigned)(val*(DAC_RANGE-1)));
}
void CVoutB(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_spi.CVout(1,(unsigned)(val*(DAC_RANGE-1)));
}
void CVoutC(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_spi.CVout(2,(unsigned)(val*(DAC_RANGE-1)));
}
void CVoutD(OSCMessage &msg) {
  float val = msg.getFloat(0); // values 0 to 1
  mr_spi.CVout(3,(unsigned)(val*(DAC_RANGE-1)));
} 


void heartBeatPrint(void) {
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connected");        
  else
    Serial.println("Disconnected");        
  
  if (num == 80) 
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0) 
  {
    Serial.print(" ");
  }
} 

TaskHandle_t status_check;

void check_status(void * pvParameters){
  for (;;){
  static ulong checkstatus_timeout = 0;

  //KH
  #define HEARTBEAT_INTERVAL    10000L
  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((millis() > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = millis() + HEARTBEAT_INTERVAL;
  }
  vTaskDelay(10);}
}

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

OSCErrorCode errorcode;

SemaphoreHandle_t sampleGate;

TaskHandle_t osc_udp_rx;

void OSC_UDP_RX(void * pvParameters) {
  for (;;) {
    
    //Serial.println("OSC RX running");
    OSCMessage msg;
    int size = Udp.parsePacket();
    if (size > 0) {    
      while (size--) {
        msg.fill(Udp.read());
      }
      if (!msg.hasError()) {
        msg.dispatch("/GateA", gateoutA);
        msg.dispatch("/GateB", gateoutB);
        msg.dispatch("/GateC", gateoutC);
        msg.dispatch("/GateD", gateoutD);
        msg.dispatch("/CVoutA", CVoutA);
        msg.dispatch("/CVoutB", CVoutB);
        msg.dispatch("/CVoutC", CVoutC);
        msg.dispatch("/CVoutD", CVoutD);
      } 
      else {
        errorcode = msg.getError();
        Serial.print("error: ");
        Serial.println(errorcode);
      }
    }
    /*static unsigned long prevMillis = millis();
    if (millis() - prevMillis > 1000)
		{
			unsigned long remainingStack = uxTaskGetStackHighWaterMark(NULL);
			Serial.printf("OSC RX Free stack:%lu\n", remainingStack);
			prevMillis = millis();
		}*/
    vTaskDelay( 1 );
  }
}

TaskHandle_t osc_udp_tx;

volatile bool gateIn[4] = {0,0,0,0};
volatile bool gateInFlag[4] = {0,0,0,0};

volatile unsigned short inCV[4] = {0,0,0,0};
volatile float inCVfloat[4] = {0,0,0,0};

TaskHandle_t cvSample = NULL;

QueueHandle_t queueCV;

void OSC_UDP_TX(void *pvParameters){
  for (;;){
    short receiveCV[4];
    xQueueReceive(queueCV, &receiveCV, portMAX_DELAY);
    //Serial.println("OSC TX running");
    OSCBundle bundl;
    if (gateInFlag[0] == 1){
      bundl.add("/CV0").add(receiveCV[0]);
      bundl.add("/Gate0").add((int)gateIn[0]);
      gateInFlag[0] = 0;
      }
    if (gateInFlag[1] == 1){
      bundl.add("/CV1").add(receiveCV[1]);
      bundl.add("/Gate1").add((int)gateIn[1]);
      gateInFlag[1] = 0;
      }
    if (gateInFlag[2] == 1){
      bundl.add("/CV2").add(receiveCV[2]);
      bundl.add("/Gate2").add((int)gateIn[2]);
      gateInFlag[2] = 0;
      }
    if (gateInFlag[3] == 1){
      bundl.add("/CV3").add(receiveCV[3]);
      bundl.add("/Gate3").add((int)gateIn[3]);
      gateInFlag[3] = 0;
      }
    /*static unsigned long prevMillis = millis();
    if (millis() - prevMillis > 1000)
		{
			unsigned long remainingStack = uxTaskGetStackHighWaterMark(NULL);
			Serial.printf("OSC TX Free stack:%lu\n", remainingStack);
			prevMillis = millis();
		}*/
    if (bundl.size() > 0){
      Udp.beginPacket(outIp, outPort);
      bundl.send(Udp); // send the bytes to the SLIP stream
      Udp.endPacket(); // mark the end of the OSC Packet
      bundl.empty();   // free space occupied by message
      vTaskDelay(1);
      }
    else{
      vTaskDelay(1);
    }
  }
}

/*
void OSC_UDP_TX(void * pvParameters) {   //2-5-2021 figure out how to do this right
  for (;;) {
    OSCBundle bundl;
    short receiveCV[4];
    xQueueReceive(queueCV, &receiveCV, portMAX_DELAY);
    int i;
    for (i=0; i<NUM_CHANNELS; i++) {
      if (gateInFlag[i] == 1) {
        bundl.add("/CV" + (String)i ).add((float)inCV[0]/DAC_RANGE);
        bundl.add("/Gate").add(i).add(gateIn[i]);
        gateInFlag[i] = !gateInFlag[i];
      }
      else if (gateInFlag[i] == 1) {
        bundl.add("/Gate").add(i).add(gateIn[i]);
        gateInFlag[i] = !gateInFlag[i];
      }
      Udp.beginPacket(outIp, outPort);
      bundl.send(Udp); // send the bytes to the SLIP stream
      Udp.endPacket(); // mark the end of the OSC Packet
      bundl.empty();   // free space occupied by message  
    }

    vTaskDelay(1);
  }
}
*/

void IRAM_ATTR gateSample0(){  
  gateIn[0] = !digitalRead(GATEin_0);
  gateInFlag[0] = 1;
}

void IRAM_ATTR gateSample1(){
  gateIn[1] = !digitalRead(GATEin_1);
  gateInFlag[1] = 1;
}

void IRAM_ATTR gateSample2(){
  gateIn[2] = !digitalRead(GATEin_2);
  gateInFlag[2] = 1;
}

void IRAM_ATTR gateSample3(){
  gateIn[3] = !digitalRead(GATEin_3);
  gateInFlag[3] = 1;
}



void SampleCV(void * pvParameters) {
  for (;;) {
    //Serial.println("CV Sample running");
    int i;
    unsigned short sendCV[4] = {0,0,0,0};
    for (i = 0; i < NUM_CHANNELS; i++) {
      inCV[i] = {mr_spi.CVin(i)};
      inCVfloat[i] = inCV[i]/DAC_RANGE;
      sendCV[i] = {mr_spi.CVin(i)};
      //Serial.print("size of sendCV ");
      //Serial.println(sizeof(sendCV));
    }
    xQueueSend(queueCV, &sendCV, portMAX_DELAY);
    
    /*static unsigned long prevMillis = millis();
    if (millis() - prevMillis > 1000)
		{
			unsigned long remainingStack = uxTaskGetStackHighWaterMark(NULL);
			Serial.printf("Sample CV Free stack:%lu\n", remainingStack);
			prevMillis = millis();
		}*/
    vTaskDelay( 1 ); 
  }
} 

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  // Options are: 240 (default), 160, 80, 40, 20 and 10 MHz
	//setCpuFrequencyMhz(10);
	int cpuSpeed = getCpuFrequencyMhz();
	Serial.println("Running at " + String(cpuSpeed) + "MHz");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
  Serial.println(F("SSD1306 allocation failed"));
  for(;;); } // Don't proceed, loop forever
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.clearDisplay();
  display.println("MORAD");
  display.display();
  vTaskDelay(500 / portTICK_PERIOD_MS);

  mr_gpio.Init();
  mr_spi.init();

//AP Config Stuff
  unsigned long startedAt = millis();
  
  Serial.println("\nStarting");
  // Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  // ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("MOADWiFiOSC");
  
  ESP_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192,168,2,114), IPAddress(192,168,2,1), IPAddress(255,255,255,0), 
                                        IPAddress(192,168,2,1), IPAddress(8,8,8,8));
  // We can't use WiFi.SSID() in ESP32as it's only valid after connected. 
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();
  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);
  
  //Check if there is stored WiFi router/password credentials.
  //If not found, device will remain in configuration mode until switched off via webserver.
  //Display AP Creds
  Serial.print("Opening configuration portal.");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); 
  display.println("WiFiMgr Access Point");
  display.println("");
  display.print("SSID:");
  display.println(ssid);
  display.print("PSWD:");
  display.println(password);
  display.display();



  if (Router_SSID != "")
  {
    ESP_wifiManager.setConfigPortalTimeout(apTimeout); //If no access point name has been previously entered disable timeout.
    Serial.print(" Timeout ");
    Serial.print(apTimeout);
    Serial.println(" seconds");
  }
  else
    Serial.println("No timeout");

  // SSID to uppercase 
  ssid.toUpperCase();  

  //it starts an access point 
  //and goes into a blocking loop awaiting configuration
  if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password)) 
    Serial.println("Not connected to WiFi but continuing anyway.");
  else 
    Serial.println("WiFi connected...yeey :)");
 
  // For some unknown reason webserver can only be started once per boot up 
  // so webserver can not be used again in the sketch.
  #define WIFI_CONNECT_TIMEOUT        30000L
  #define WHILE_LOOP_DELAY            200L
  #define WHILE_LOOP_STEPS            (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))
  
  startedAt = millis();
  
  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) ){   
    WiFi.mode(WIFI_STA);
    WiFi.persistent (true);
    // We start by connecting to a WiFi network
    
    Serial.print("Connecting to ");
    Serial.println(Router_SSID);
    
    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());
    
    int i = 0;
    while((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
    {
      delay(WHILE_LOOP_DELAY);
    }    
  }

  Serial.print("After waiting ");
  Serial.print((millis()- startedAt) / 1000);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED) //Update Display To Show Connection
  { 
    Udp.begin(localPort);

    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("Starting UDP");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Local port: ");
    Serial.println(localPort);    
    Serial.print("Transmit IP: ");
    Serial.println(outIp);
    Serial.print("Out port: ");
    Serial.println(outPort);

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Connected!");
    display.println("  WiFiOSC");
    display.display();
    vTaskDelay (500 / portTICK_PERIOD_MS);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    //display.println("Connected! WiFiOSC");
    display.print("rXIP: ");
    display.println(WiFi.localIP());
    display.print("rX:   ");
    display.println(localPort);
    display.print("tXIP: ");
    display.println(outIp);
    display.print("tX:   ");    
    display.println(outPort);

    display.display(); 
  }
  else
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));

  sampleGate = xSemaphoreCreateMutex();

  queueCV = xQueueCreate( 1, 8);

  attachInterrupt(GATEin_0, gateSample0, CHANGE);
  attachInterrupt(GATEin_1, gateSample1, CHANGE);
  attachInterrupt(GATEin_2, gateSample2, CHANGE);
  attachInterrupt(GATEin_3, gateSample3, CHANGE);

  xTaskCreatePinnedToCore(
      OSC_UDP_RX,  /* Task function. */
      "osc udp rx",   /* name of task. */
      2000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &osc_udp_rx, /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
      OSC_UDP_TX,   /* Task function. */
      "osc udp tx", /* name of task. */
      2000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      &osc_udp_tx,  /* Task handle to keep track of created task */
      1);           /* pin task to core 1 */

  xTaskCreatePinnedToCore(
      SampleCV,   /* Task function. */
      "SampleCV", /* name of task. */
      1650,      /* Stack size of task */
      NULL,       /* parameter of the task */
      3,          /* priority of the task */
      &cvSample,  /* Task handle to keep track of created task */
      1);         /* pin task to core 1 */

  xTaskCreatePinnedToCore(
      check_status,   /* Task function. */
      "status check", /* name of task. */
      1024,          /* Stack size of task */
      NULL,           /* parameter of the task */
      6,              /* priority of the task */
      &status_check,  /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */
}

void loop() {
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  //delay(10);
  

/*
  //test test
  mr_spi.CVout(2, 1000);
  vTaskDelay( 250 );
  mr_spi.CVout(2, 2000);
  vTaskDelay( 250 );
  mr_spi.CVout(2, 3000);
  vTaskDelay( 250 );
  mr_spi.CVout(2, 4000);
  vTaskDelay( 250 );
  mr_spi.CVout(2, 3000);
  vTaskDelay( 250 );
  mr_spi.CVout(2, 2000);
  vTaskDelay( 250 );  
*/


  
  vTaskDelete( NULL );
  //vTaskDelay(portMAX_DELAY);

  /*
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); 
  display.print("CV_A:");
  display.println((float)inCV[0]/DAC_RANGE);
  display.print("CV_B:");
  display.println((float)inCV[1]/DAC_RANGE);
  display.print("CV_C:");
  display.println((float)inCV[2]/DAC_RANGE);
  display.print("CV_D:");
  display.println((float)inCV[3]/DAC_RANGE);
  display.display();
  */
  //vTaskDelay( 1000 );
}