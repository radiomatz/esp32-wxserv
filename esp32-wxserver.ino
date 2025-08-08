#include <WiFi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Wire.h>
#include "BlueDot_BME280.h"

#include <ArduinoMqttClient.h>
#include "time.h"
#include "esp_sntp.h"


// The NAME of your wifi:
const char *ssid = "MYBOX XXXX YYYYY";
// Password of your wifi:
const char *password = "PPPPPPPPPPPPPPPPPPPP";
// The Mac Address of your wifi:
const uint8_t bssid[6] = { 0xfe, 0xfd, 0xfc, 0xfb, 0xfb, 0xfa };
// Dont forget to change your callsign below in loop()

const int LED = 2;

WiFiServer server(1432);


BlueDot_BME280 bme280 = BlueDot_BME280();
int i_bme280 = 1;
float pressure = 1013;
float humidity = 50;
float temp = 69.0;
float tempc = 15.0;

struct tm timeinfo;
bool time_ok = false;

// Change here to your prefered TimeServer
const char *ntpServer1 = "192.168.178.1";
const char *ntpServer2 = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
const char broker[] = "192.168.178.20";
int port = 1883;
const char topic[] = "weather/sensor";
const char topicd[] = "weather/date";
volatile long nrloops;
char stemp[128];



void setup_bme() {

  if (i_bme280 == 0)
    return;

  //*********************************************************************
  //*************BASIC SETUP - READ BEFORE GOING ON!*********************

  //Choose between the SPI and I2C Communication protocols
  //Or leave the I2C Communication as default
  //0:        Set to 0 for I2C (default value)
  //1:        Set to 1 for Software SPI
  //2:        Set to 2 for Hardware SPI
  bme280.parameter.communication = 0;  //Choose communication protocol

  //*********************************************************************
  //*************BASIC SETUP - READ BEFORE GOING ON!*********************
  //Set the I2C address of your breakout board
  //Or ignore this, if you're using SPI Communication
  //0x76:       Alternative I2C Address (SDO pin connected to GND)
  //0x77:       Default I2C Address (SDO pin unconnected)
  bme280.parameter.I2CAddress = 0x76;  //Choose I2C Address

  //*********************************************************************
  //*************BASIC SETUP - READ BEFORE GOING ON!*********************

  //Set the pins for SPI Communication
  //Or ignore this, if you're using I2C Communication instead
  bme280.parameter.SPI_cs = 0;    //Are you using either Software SPI or Hardware SPI? Then you need to define the Chip Select Pin.
  bme280.parameter.SPI_mosi = 0;  //If you are using Software SPI, then you need to define the MOSI line. For Hardware SPI you can leave this line commented.
  bme280.parameter.SPI_miso = 0;  //If you are using Software SPI, then you need to define the MISO line. Just comment this out for Hardware SPI.
  bme280.parameter.SPI_sck = 0;   //If you are using Software SPI, then you need to define the SCK line. Same as before. For Hardware SPI, just comment this out.

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  //Now choose on which mode your device will run
  //On doubt, just leave on normal mode, that's the default value
  //0b00:     In sleep mode no measurements are performed, but power consumption is at a minimum
  //0b01:     In forced mode a single measured is performed and the device returns automatically to sleep mode
  //0b11:     In normal mode the sensor measures continually (default value)
  bme280.parameter.sensorMode = 0b11;  //Choose sensor mode

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  //Great! Now set up the internal IIR Filter
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
  //In short, a high factor value means less noise, but measurements are also less responsive
  //You can play with these values and check the results!
  //In doubt just leave on default
  //0b000:      factor 0 (filter off)
  //0b001:      factor 2
  //0b010:      factor 4
  //0b011:      factor 8
  //0b100:      factor 16 (default value)
  bme280.parameter.IIRfilter = 0b100;  //Setup for IIR Filter

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  //Next you'll define the oversampling factor for the humidity measurements
  //Again, higher values mean less noise, but slower responses
  //If you don't want to measure humidity, set the oversampling to zero

  //0b000:      factor 0 (Disable humidity measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  bme280.parameter.humidOversampling = 0b101;  //Setup Humidity Oversampling

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  //Now define the oversampling factor for the temperature measurements
  //You know now, higher values lead to less noise but slower measurements
  //0b000:      factor 0 (Disable temperature measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  bme280.parameter.tempOversampling = 0b101;  //Setup Temperature Ovesampling

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  //Finally, define the oversampling factor for the pressure measurements
  //For altitude measurements a higher factor provides more stable values
  //On doubt, just leave it on default
  //0b000:      factor 0 (Disable pressure measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  bme280.parameter.pressOversampling = 0b101;  //Setup Pressure Oversampling

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  //For precise altitude measurements please put in the current pressure corrected for the sea level
  //On doubt, just leave the standard pressure as default (1013.25 hPa)
  bme280.parameter.pressureSeaLevel = 1013.25;  //default value of 1013.25 hPa

  //Now write here the current average temperature outside (yes, the outside temperature!)
  //You can either use the value in Celsius or in Fahrenheit, but only one of them (comment out the other value)
  //In order to calculate the altitude, this temperature is converted by the library into Kelvin
  //For slightly less precise altitude measurements, just leave the standard temperature as default (15°C)
  //Remember, leave one of the values here commented, and change the other one!
  //If both values are left commented, the default temperature of 15°C will be used
  //But if both values are left uncommented, then the value in Celsius will be used
  bme280.parameter.tempOutsideCelsius = 15;  //default value of 15°C
  //bme280.parameter.tempOutsideFahrenheit = 59;           //default value of 59°F
}



void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("No time available (yet)");
    return;
  }
  time_ok = 1;
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (gets called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}


void connect_mqtt() {
  if ( !mqttClient.connect(broker, port) ) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  } else {
    Serial.println("You're connected to the MQTT broker!");
  }
}



void setup() {

  nrloops = 0;

  Serial.begin(57600);
  do {
    delay(500);
  } while ( !Serial );

  Serial.println();
  Serial.println(__FILE__);

  pinMode(LED, OUTPUT);  // set the LED pin mode

  delay(10);

  // WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password, 0, bssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // BME280
  setup_bme();
  Wire.begin(21, 22);
  uint8_t bme_id = bme280.init();
  if (bme_id != 0x60) {
    i_bme280 = 0;
    Serial.print(F("Ops! BME280 could not be found! "));
    Serial.println(bme_id, HEX);
  } else {
    Serial.print(F("BME280 detected! "));
    Serial.println(bme_id, HEX);
  }

  // MQTT
  connect_mqtt();

  // NTP
  sntp_set_time_sync_notification_cb(timeavailable);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);


  // TCP SERVER
  server.begin();
}



void loop() {
  WiFiClient client = server.available();  // listen for incoming clients
  if (client) {
    if (client.connected()) {
      client.printf("[0] N0CALL-13>WIDE1-1,TCPIP:=4822.17N/01043.09E_c...s...g...t%03.0fh%02.0fb%0.0fxBME ESP32 %.1fC %.1f%% %.1fhPa\n", temp, humidity, pressure * 10.0, tempc, humidity, pressure);
      client.stop();
   }
  }


  // if no client, read BME280
  if (!i_bme280 == 0) {

    // Serial.print(F("Temperature in Celsius:\t\t"));
    temp = bme280.readTempF();
    tempc = bme280.readTempC();
    // Serial.println(tempc);

    // Serial.print(F("Humidity in %:\t\t\t"));
    humidity = bme280.readHumidity();
    // Serial.println(humidity);

    // Serial.print(F("Pressure in hPa:\t\t"));
    pressure = bme280.readPressure() / pow(1.0 - 500.0 / 44330.0, 5.255);
    // Serial.println(pressure);

    //Serial.println();
  }

  if ( !mqttClient.connected() )
    connect_mqtt();

  if ( mqttClient.connected() ) {  
    mqttClient.poll();
    getLocalTime(&timeinfo);
  }

  if ( mqttClient.connected() && time_ok && nrloops <= 0 ) {  // only all 10 mins

    nrloops = 10 * 60 * 2;
    strftime(stemp, 80, "%F %T", &timeinfo);

    mqttClient.beginMessage(topic, true);
    mqttClient.printf("%.1f °C - %.0f %% - %.1f hPa", tempc, humidity, pressure);
    mqttClient.endMessage();
    
    mqttClient.beginMessage(topicd, true);
    mqttClient.print(stemp);
    mqttClient.endMessage();

    Serial.print("MQTT done - ");
    Serial.print("Date: ");
    Serial.println(stemp);
  }

  nrloops = nrloops - 1;

  delay(500);
}
