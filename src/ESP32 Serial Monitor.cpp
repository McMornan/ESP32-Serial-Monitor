#define IR_PIN  21
#define DISPLAY_ON (HIGH)
#define DISPLAY_OFF (LOW)

#include "FS.h" // for touchscreen calibration data
#define CALIBRATION_FILE "/TouchCalData2"
#define REPEAT_CAL false

#include <SPI.h>

#define USE_DMA_TO_TFT
#include "TFT_eSPI.h"
#include "Free_Fonts.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TelnetSpy.h>
#include <Wire.h>

#if not defined(ST7796_DRIVER)
  #error "wrong display driver defined!"
#endif

const char* Aurora_topic = "openhab/Aurora/POWER";
const char* BlackKnight_topic = "zigbee2mqtt/ZigbeeSteckdose15"; //state


/**************************** FOR OTA **************************************************/
#define SENSORNAME "ControllerPanel Buttons" //change this to whatever you want to call your device

#define OTApassword "123" //the password you will need to enter to upload remotely via the ArduinoIDE
int OTAport = 8266;

/****************************************FOR JSON***************************************/
const int BUFFER_SIZE = 512;// NOT SMALLER or sketch will crash
#define MQTT_MAX_PACKET_SIZE 1024

/************ WIFI and MQTT Information (CHANGE THESE FOR YOUR SETUP) ******************/
const char* ssid = "yourwlanid"; //type your WIFI information inside the quotes
const char* password = "secretpassword";
const char* mqtt_server = "mqttserverip";
const char* mqtt_username = "";
const char* mqtt_password = "";
const int mqtt_port = 1883;
TFT_eSPI tft = TFT_eSPI();

#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
// Create 6 keys for the configuration menu
char keyLabel[12][15] = {"1", "2", "3","landscape","portrait","exit","9600","19200","38400","57600","115200","230400"};
uint16_t keyColor[12] = {TFT_BLACK, TFT_BLACK, TFT_BLACK,TFT_BLUE, TFT_BLUE, TFT_RED,TFT_DARKGREY,TFT_DARKGREY,TFT_DARKGREY,TFT_DARKGREY,TFT_DARKGREY,TFT_DARKGREY};

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button key[12];

WiFiClient espClient;
PubSubClient client(espClient);

bool ConnectionEstablished; // Flag for successfully handled connection
TelnetSpy LOG;

int fontsize = 1;
int serialspeed = 1;


// The scrolling area must be a integral multiple of TEXT_HEIGHT
int TEXT_HEIGHT=16; // Height of text to be printed and scrolled
#define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
#define TOP_FIXED_AREA 0 // Number of lines in top fixed area (lines counted from top of screen)
int YMAX = 480; // Bottom of screen area
int XMAX = 320;

// The initial y coordinate of the top of the scrolling area
uint16_t yStart = TOP_FIXED_AREA;
// yArea must be a integral multiple of TEXT_HEIGHT
uint16_t yArea = YMAX - TOP_FIXED_AREA-BOT_FIXED_AREA;
// The initial y coordinate of the top of the bottom text line
uint16_t yPos = TOP_FIXED_AREA;
uint16_t fontOffset = 0;

// Keep track of the drawing x coordinate
uint16_t xPos = 0;

// For the byte we read from the serial port
byte data = 0;

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

/********************************** START SETUP WIFI*****************************************/
void setup_wifi() {

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(FM12);
  tft.setCursor(20, 20);

  delay(10);
  // We start by connecting to a WiFi network
  LOG.println();
  LOG.print("Connecting to ");
  LOG.println(ssid);

  tft.println();
  tft.print("Connecting to ");
  tft.println(ssid);

  LOG.print("WIFI status = ");
  LOG.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  LOG.print("WIFI status = ");

  WiFi.mode(WIFI_STA);
  delay(1000);
  WiFi.begin(ssid, password);

  int nCount = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    tft.print(".");

    nCount ++;

    if (nCount >= 10) ESP.restart();

    delay(1000);
  }

  LOG.println("");
  LOG.println("WiFi connected");
  LOG.println("IP address: ");
  LOG.println(WiFi.localIP());

  tft.println("");
  tft.println("WiFi connected");
  tft.println("IP address: ");
  tft.println(WiFi.localIP());
}

/********************************** MQTT callback*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
  }
  return result;
}

/********************************** START RECONNECT*****************************************/
void reconnect() {

  if (WiFi.status() != WL_CONNECTED) return;

  int nCount = 0;

  // Loop until we're reconnected
  LOG.print("Attempting MQTT connection...");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
    LOG.print(".");
    delay(5000);

    String ClientId = "ControllerPanelButtons__";
    unsigned char mac[6];
    WiFi.macAddress(mac);
    ClientId += macToStr(mac);

    // Attempt to connect
    if (client.connect(ClientId.c_str())) {
      LOG.println("connected");
      //client.subscribe(Aurora_topic);
      
    } else {
      LOG.print("failed, rc=");
      LOG.print(client.state());
      LOG.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }

    nCount ++;
    if ( nCount > 10) ESP.restart();
  }
}

void setupScrollArea(uint16_t TFA, uint16_t BFA) {
  tft.writecommand(0x33); // Vertical scroll definition
  tft.writedata(TFA >> 8);
  tft.writedata(TFA);
  tft.writedata((YMAX - TFA - BFA) >> 8);
  tft.writedata(YMAX - TFA - BFA);
  tft.writedata(BFA >> 8);
  tft.writedata(BFA);
}

void scrollAddress(uint16_t vsp) {
  tft.writecommand(0x37); // Vertical scrolling pointer
  tft.writedata(vsp>>8);
  tft.writedata(vsp);
}

void setup(void) {
  LOG.begin(230400);
  Serial2.begin(9600);
  
  pinMode(TFT_BL, OUTPUT); // switch Display LED on
  digitalWrite(TFT_BL, DISPLAY_ON);

  pinMode(21, INPUT_PULLUP); // switch Display LED on

  tft.init();

  tft.setRotation(0); /* Landscape orientation */
  touch_calibrate();

  

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  //OTA SETUP
  ArduinoOTA.setPort(OTAport);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(SENSORNAME);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() {
    LOG.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    LOG.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    LOG.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      LOG.println("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR) {
      LOG.println("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR) {
      LOG.println("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR) {
      LOG.println("Receive Failed");
    }
    else if (error == OTA_END_ERROR) {
      LOG.println("End Failed");
    }
    delay(1000);
    ESP.restart();
  });
  ArduinoOTA.begin();

  LOG.setPort(24);
  LOG.setWelcomeMsg("Serial Logger\n\n");
  LOG.setDebugOutput(false);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(4,0);
  tft.setTextFont(1);
  TEXT_HEIGHT = tft.fontHeight(1)+1;
  tft.print("ready...9600 baud");

  setupScrollArea(TOP_FIXED_AREA, BOT_FIXED_AREA);

  if(digitalRead(21) == LOW) {
    tft.setFreeFont(FMB24);
    tft.setCursor(20,80);
    tft.setTextColor(TFT_RED);
    tft.print("ATTENTION: \nPAUSE \nTRIGGERED!");

    while(digitalRead(21) == LOW);

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20,80);
    tft.setTextColor(TFT_GREEN);
    tft.println("CONTINUE...");

    tft.setTextColor(TFT_WHITE);
    tft.setTextFont(1);  
  }
  
}

int configMenu(int orientation)
{
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  scrollAddress(0); // reset scroll shift

  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(FM18);
  tft.setCursor(50,35);
  tft.println("fontsize");

  tft.setCursor(50,125);
  tft.println("screen orientation");

  tft.setCursor(50,205);
  tft.println("serial speed");

  tft.setFreeFont(FM12);
  key[0].initButton(&tft, 70, 70, 62,40, TFT_WHITE, keyColor[0], TFT_WHITE, keyLabel[0], 1);
  key[0].drawButton();

  key[1].initButton(&tft, 150, 70, 62,40, TFT_WHITE, keyColor[1], TFT_WHITE, keyLabel[1], 1);
  key[1].drawButton();

  key[2].initButton(&tft, 230, 70, 62,40, TFT_WHITE, keyColor[2], TFT_WHITE, keyLabel[2], 1);
  key[2].drawButton();

  key[3].initButton(&tft, 130, 155, 140,40, TFT_WHITE, keyColor[3], TFT_WHITE, keyLabel[3], 1);
  key[3].drawButton();

  key[4].initButton(&tft, 290, 155, 140,40, TFT_WHITE, keyColor[4], TFT_WHITE, keyLabel[4], 1);
  key[4].drawButton();

  key[5].initButton(&tft, 410, 290, 120,40, TFT_WHITE, keyColor[5], TFT_WHITE, keyLabel[5], 1);
  key[5].drawButton();

  tft.setFreeFont(FM9);
  key[6].initButton(&tft, 45, 235, 65,40, TFT_WHITE, keyColor[6], TFT_WHITE, keyLabel[6], 1);
  key[6].drawButton();

  key[7].initButton(&tft, 116, 235, 74,40, TFT_WHITE, keyColor[7], TFT_WHITE, keyLabel[7], 1);
  key[7].drawButton();

  key[8].initButton(&tft, 191, 235, 74,40, TFT_WHITE, keyColor[8], TFT_WHITE, keyLabel[8], 1);
  key[8].drawButton();

  key[9].initButton(&tft, 268, 235, 74,40, TFT_WHITE, keyColor[9], TFT_WHITE, keyLabel[9], 1);
  key[9].drawButton();

  key[10].initButton(&tft, 343, 235, 75,40, TFT_WHITE, keyColor[10], TFT_WHITE, keyLabel[10], 1);
  key[10].drawButton();

  key[11].initButton(&tft, 419, 235, 75,40, TFT_WHITE, keyColor[11], TFT_WHITE, keyLabel[11], 1);
  key[11].drawButton();

  uint16_t t_x = 0, t_y = 0; // To store the touch coordinates

  long timeout = millis();
  

  while((millis() - timeout) < 10000) {
    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = tft.getTouch(&t_x, &t_y);

    // / Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t b = 0; b < 12; b++) {
      if (pressed && key[b].contains(t_x, t_y)) {
        key[b].press(true);  // tell the button it is pressed
      } else {
        key[b].press(false);  // tell the button it is NOT pressed
      }
    }

    // Check if any key has changed state
    for (uint8_t b = 0; b < 12; b++) {
      if (key[b].justReleased()) key[b].drawButton();     // draw normal

      if (key[b].justPressed()) {
        key[b].drawButton(true);  // draw invert

        if (b < 3) {
          fontsize = b +1;
          return orientation;
        }
        if (b == 3) {
          orientation = 1; 
          return orientation;
        }
        if (b == 4) {
          orientation = 0;  
          return orientation;
        }
        if (b == 5) {
          return orientation;
        }
        if (b > 5) {
          serialspeed = b - 5;
          return orientation;
        }
        delay(10);
      }
    }
  }
return orientation;
}

int scroll_line() {
  int yTemp = yStart;

  if (tft.getRotation() == 1) {
    yStart += TEXT_HEIGHT;
    if (yStart >= (YMAX - BOT_FIXED_AREA)) {
      yStart = TOP_FIXED_AREA + fontOffset;
      tft.fillScreen(TFT_BLACK);
    }
    yTemp = yStart;
  } else {
    for (int i = 0; i < TEXT_HEIGHT; i++) {
      yStart++;
      if (yStart == (YMAX - BOT_FIXED_AREA)) {
        yStart = TOP_FIXED_AREA + fontOffset;
      }
      scrollAddress(yStart);
    }
    tft.fillRect(0,yTemp,XMAX,TEXT_HEIGHT, TFT_BLACK);
  }
  return yTemp;
}

void loop(void) {
  if (WiFi.status() != WL_CONNECTED) {
    delay(1);
    LOG.print("WIFI Disconnected. Attempting reconnection.");
    setup_wifi();
    return;
  }

  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  ArduinoOTA.handle();
  LOG.handle();

  uint16_t x,y;

  if( tft.getTouch(&x,&y)) {
    int orientation = tft.getRotation();
    int newOrientation = configMenu(orientation);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(4,fontOffset);

    switch(newOrientation) {
      case 0: // hardware scroll in portrait mode
        tft.setRotation(newOrientation);
        XMAX=320;
        YMAX=480;
        yStart = yPos = TOP_FIXED_AREA;
        setupScrollArea(TOP_FIXED_AREA, BOT_FIXED_AREA);
        break;

      case 1: // screen blanking in landscape mode
        tft.setRotation(newOrientation);
        XMAX=480;
        YMAX=320;
        yStart = yPos = TOP_FIXED_AREA;
        break;
    }
    switch(fontsize) {
      case 1: tft.setTextFont(1);TEXT_HEIGHT = tft.fontHeight(1)+1;fontOffset = 0;break;
      case 2: tft.setFreeFont(FM9);TEXT_HEIGHT = 15;fontOffset = 10;break;
      case 3: tft.setFreeFont(FM12);TEXT_HEIGHT = 18;fontOffset = 18;break;
      case 4: tft.setFreeFont(FM24);TEXT_HEIGHT = 30;fontOffset = 32;break;
    }
    
    int baudrate = 115200;

    switch(serialspeed) {
      case 1: baudrate = 9600;break;
      case 2: baudrate = 19200;break;
      case 3: baudrate = 38400;break;
      case 4: baudrate = 57600;break;
      case 5: baudrate = 115200;break;
      case 6: baudrate = 230400;break;
    }

    Serial2.end();
    Serial2.begin(baudrate);

    tft.print("ready...");
    tft.print(baudrate);
    tft.println(" baud");
    delay(500);
  }

  while (Serial2.available()) {
    data = Serial2.read();

    if(digitalRead(21)) { // only show read data when pause switch has not been triggered
      // If it is a CR or we are near end of line then scroll one line
      if (data == '\r' || xPos>(XMAX - 10)) {
        xPos = 0;
        yPos = scroll_line(); 
        LOG.println();
      }
      if (data > 31 && data < 128) {
        xPos += tft.drawChar(data,xPos,yPos);

        char str[2];
        str[0] = data;
        str[1] = 0;
        LOG.print(str);
      }
    }
  }
}
