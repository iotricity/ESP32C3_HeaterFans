#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFiMulti.h>
#include <ESPping.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <FastLED.h>

// Pin definitions for LilyGo T01-C3 (ESP01 alike)
// https://lilygo.cc/products/t-01c3
#define LEDPIN1        8      // LED Data/Clk pin
#define LEDPIN2       21      // LED Data/Clk pin --> GPIO U0TXD
#define LEDPIN3       20      // LED Data/Clk pin --> GPIO U0RXD
#define ONEWPIN        9      // OneWire pin
#define PWMPIN         2      // Fan PWM pin

// PWM Settings
#define PWMFREQ       25000   // PWM Frequency (22.5kHz ~ 25kHz should do)
#define RPMMIN        20      // Low RPM
#define RPMMAX       224      // High RPM

// LED lighting settings
#define LED_TYPE      WS2812
#define COLOR_ORDER   GRB
#define NUM_LEDS      16       // Light Wings 140mm = 20, Light Wings LX 120mm = 16
#define BRIGHTNESS    96
#define SPARKLES_NUM        50                    // Number of sparkles
#define SPARKLES_CHANGE     3                     // Randomness of sparkles
#define SPARKLES_CLR        200                   // Randomness of sparkles

// Temperature compensation (if needed)
#define TEMPCALIB     -1.0
#define TEMPTRESHHLD  30.0
OneWire oneWire(ONEWPIN);
DS18B20 sensor(&oneWire);

// Network/internet
#define WIFICONNTM    7500          // Auto-connect time-out
IPAddress pingIP(8, 8, 8, 8); // The remote ip to ping, Google DNS
WiFiMulti wifiMulti;
WiFiServer server(80);

// Initialize LED array
CRGB leds[NUM_LEDS * 3];
int sparkles[SPARKLES_NUM][2];

CRGB scale[] = {
  CRGB(0, 0, 255),
  CRGB(0, 120, 210),
  CRGB(0, 210, 120),
  CRGB(0, 255, 0),
  CRGB(93, 242, 0),
  CRGB(132, 228, 0),
  CRGB(157, 214, 0),
  CRGB(176, 200,0),
  CRGB(192, 185, 0),
  CRGB(206, 170, 0),
  CRGB(219, 153, 0),
  CRGB(230, 135, 0),
  CRGB(241, 113, 0),
  CRGB(248, 89, 0),
  CRGB(253, 60, 0),
  CRGB(255, 0, 0)
};

// Global for temperature measurement
float acttemp = -128.0;

// Globals for WiFi status
boolean hasWifi = false, hasInet = false;
float pingInet = 0.0;



//---------- Setup ----------------------------------------------------
void setup(void) {
  static int ctr = 0;

  pinMode(PWMPIN, OUTPUT);

  // Set DS18B20 resolution to 12-bits
  sensor.setResolution(12);

  WiFi.mode(WIFI_STA);
  // Add one or more access point SSID's and passwords here
  wifiMulti.addAP("yourssid", "somepassword");
  //wifiMulti.addAP("AnotherSSID", "somepassword");
  //wifiMulti.addAP("AnotherSSID", "somepassword");

  server.begin();

  sensor.begin();

  // Initialize the LED strip
  delay(1000); // safety startup delay for led strips
  FastLED.addLeds<LED_TYPE, LEDPIN1, COLOR_ORDER>(leds, 0, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, LEDPIN2, COLOR_ORDER>(leds, NUM_LEDS, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, LEDPIN3, COLOR_ORDER>(leds, 2 * NUM_LEDS, NUM_LEDS);
  FastLED.setCorrection(TypicalLEDStrip);
         
  FastLED.setBrightness(BRIGHTNESS);

  fill_solid(leds, NUM_LEDS * 3, CHSV(0, 255, 64));

  leds[1] = CRGB::Blue;
  for(int i = 0; i< SPARKLES_NUM; i++) {
    sparkles[i][0] = -1;      // Sparkle brightness, -1 means available for new position
    sparkles[i][1] = 0;       // Sparkle position
  }

  FastLED.show();
  
  // Initialize PWM for fan control, 8-bits resolutions
  ledcAttach(PWMPIN, PWMFREQ, 8);
  ledcWrite(PWMPIN, 0); // Turn of fan
}


//---------- Main loop -------------------------------------------------
void loop() {
  static unsigned long timeWifiUpdate, timeTempUpdate;
  unsigned long dlyLoop, dlyDelay, dlyFastLED;
  static int ledtemp = 0;

  // Check wifi connection every 5 seconds
  if(millis() - timeWifiUpdate >= 5000) {
    timeWifiUpdate = millis();
    // If the connection to the last or strongest hotspot is lost, it will connect to the next network on the list
    hasInet = false;
    if(wifiMulti.run(WIFICONNTM) == WL_CONNECTED) {
      hasWifi = true;
      hasInet = Ping.ping("www.google.com" /* pingIP */, 1);
      pingInet = Ping.averageTime();
    } else {
      hasWifi = false;
    }
  }

  // Check temperature sensor every 10 seconds
  if(millis() - timeTempUpdate >= 10000 || acttemp == -128.0) {
    timeTempUpdate = millis();
    sensor.requestTemperatures();
    //delay(2);
    acttemp = (sensor.getTempC() + TEMPCALIB);
    if(acttemp < 15.0) {
      acttemp = 15.0;
    }
    if(acttemp > 60.0) {
      acttemp = 60.0;
    }
    ledtemp = map((long) acttemp, 15, 60, 0, NUM_LEDS - 1);
    if(acttemp >= TEMPTRESHHLD) {
      ledcWrite(PWMPIN, RPMMAX);
    } else {
      ledcWrite(PWMPIN, RPMMIN);
    }
  }

  // Check internet client for HTTP request, only if internet connection is active
  if(hasInet) {
    WiFiClient client = server.available();   // listen for incoming clients
    if(client) {                             // if you get a client,
      String currentLine = "";                // make a String to hold incoming data from the client
      while(client.connected()) {            // loop while the client's connected
        if(client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          if(c == '\n') {                    // if the byte is a newline character

            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if(currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();

              // the content of the HTTP response follows the header:
              client.print("Current temp.: ");
              client.print(acttemp);
              client.print("<br>&nbsp;<br>");

              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if(c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
        }
      }
      // close the connection:
      client.stop();
    }
  }

  if(millis() - dlyFastLED >= 20) {
    dlyFastLED = millis();
    run_sparkles(scale[ledtemp]);
    //run_heartbeat();    // Add some fancy heart beat for the first LED on the first fan
    FastLED.show();
  }

}


void run_heartbeat() {
  // Show a red heartbeat at the last LED
  static int heartbeat_cntr = 0;
  static int heartbeat_fade = 100;
  static long heartbeat_delay = 0;

  if(heartbeat_cntr++ >= 350) {
    heartbeat_cntr = 0;
  }

  // Make the LED fade to dark red
  if(heartbeat_cntr == 0 || heartbeat_cntr == 110) {
    if(hasInet) {
      leds[1] = CRGB::Red;
    } else {
      leds[1] = CRGB::White;
    }
    heartbeat_fade = 100;
  } else {
    heartbeat_fade--;
    if(heartbeat_fade < 4) {
      heartbeat_fade = 4;
    }
  }
  leds[1] = leds[1].fadeToBlackBy(1); 
}


void run_sparkles(CRGB bkgdcolor) {
  // Add some white sparkles for that christmas feeling
  static int maxsparkles;

  maxsparkles = 15;

  for(int i = 0; i < maxsparkles; i++) {
    if(sparkles[i][0] < 0 && random(2000) < SPARKLES_CHANGE) {
      sparkles[i][0] = random(32, 128);    // Choose random brightness
      sparkles[i][1] = random(NUM_LEDS * 3);    // Choose position
    }
    if(sparkles[i][0] > 0) {
      leds[sparkles[i][1]] = CRGB(sparkles[i][0], sparkles[i][0], sparkles[i][0]); // CHSV(SPARKLES_CLR, 255, sparkles[i][0]);     // Add sparkle as white with brightness
      sparkles[i][0] -= 2;
      // Once fade out, set color back to background color
      if(sparkles[i][0] <= 2) {
        leds[sparkles[i][1]] = bkgdcolor; //CHSV(0, 255, 64); // set to red
        leds[sparkles[i][1]] = leds[sparkles[i][1]].fadeToBlackBy(224);
        sparkles[i][0] = -1;
      }
    }
  }
}

