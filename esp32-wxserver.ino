#include <WiFi.h>
#include <Wire.h>
#include "BlueDot_BME280.h"
#include <ArduinoMqttClient.h>
#include "time.h"
#include "esp_sntp.h"

const char *ssid = "MYBOX XXXX YYYYY";                           // The NAME of your wifi router
const char *password = "PPPPPPPPPPPPPPPPPPPP";                   // Password of your wifi router
const uint8_t bssid[6] = { 0xfe, 0xfd, 0xfc, 0xfb, 0xfb, 0xfa }; // The Mac Address of your wifi router

#define SLEEP_AT_NIGHT true // are you also one of them switching WiFi off over night at your router ?
#define SLEEP_FROM 2300     // hhmm
#define SLEEP_TO    659     // hhmm or hmm 

const char *callsign = "N0CALL";// your callsign without suffix (see suffixes @ aprs-doku)
const char *pos_ns = "4848.48N"; // DDMM.SSN or DDMM.SSS ; seconds as decimal fraction of minute
const char *pos_ew = "01010.10E"; // DDDMM.SSE or DDDMM.SS W ; seconds as decimal fraction of minute

#define PORT 1432 // the PortNumber of this host to connect to (tcp)

const int LED = 2;

WiFiServer server(PORT);

BlueDot_BME280 bme280 = BlueDot_BME280();
int i_bme280 = 1;
float pressure = 1013;
float humidity = 50;
float temp = 69.0;   // farenheit
float tempc = 15.0;  // celsius

struct tm timeinfo;
bool time_ok = false;
const char *ntpServer1 = "192.168.178.1";
const char *ntpServer2 = "europe.pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 0;

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
  if (!getLocalTime(&timeinfo)) {
    if (Serial) Serial.println("No time available (yet)");
    return;
  }

  if (Serial) Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (gets called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
  time_ok = true;
  if (Serial) Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}


void connect_mqtt() {
  if (Serial) Serial.println("connecting to MQTT");
  if (!mqttClient.connect(broker, port)) {
    if (Serial) Serial.print("MQTT connection failed! Error code = ");
    if (Serial) Serial.println(mqttClient.connectError());
  } else {
    if (Serial) Serial.println("You're connected to the MQTT broker!");
  }
}


void blink(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, true);
    delay(150);
    digitalWrite(LED, false);
    delay(150);
  }
  delay(150);
}



void setup() {

  nrloops = 0;

  pinMode(LED, OUTPUT);
  digitalWrite(LED, true);

  Serial.begin(115200);
  delay(2000);

  if (Serial) {
    Serial.println();
    Serial.println(__FILE__);
  }

  // WiFi network
  if (Serial) Serial.println();
  if (Serial) Serial.println();
  if (Serial) Serial.print("Connecting to ");
  if (Serial) Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password, 0, bssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    if (Serial) Serial.print(".");
  }

  if (Serial) Serial.println("");
  if (Serial) Serial.println("WiFi connected.");
  if (Serial) Serial.print("IP address: ");
  if (Serial) Serial.println(WiFi.localIP());

  // BME280
  setup_bme();
  Wire.begin(21, 22);
  uint8_t bme_id = bme280.init();
  if (bme_id != 0x60) {
    i_bme280 = 0;
    if (Serial) Serial.print(F("Ops! BME280 could not be found! "));
    if (Serial) Serial.println(bme_id, HEX);
  } else {
    if (Serial) Serial.print(F("BME280 detected! "));
    if (Serial) Serial.println(bme_id, HEX);
  }

  // NTP
  sntp_set_time_sync_notification_cb(timeavailable);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  while ( !time_ok )
    delay(100);

  // MQTT
  connect_mqtt();

  // TCP SERVER
  server.begin(PORT);

  digitalWrite(LED, false);
}


void loop() {

  if ( time_ok ) {

    if ( SLEEP_AT_NIGHT ) {
      int hhmm = timeinfo.tm_hour * 100 + timeinfo.tm_min;
      if (SLEEP_AT_NIGHT && (hhmm >= SLEEP_FROM || hhmm < SLEEP_TO)) {
        if (Serial) Serial.println("Time now is: " + String(hhmm));
        if (Serial) Serial.println("Sleeping due to WiFi Shutdown over night");
        delay(3600000);  // 1h
      }
    } 

    if (!WiFi.isConnected()) {
      if (Serial) Serial.println("reconnecting WiFi");
      time_ok = false;
      WiFi.reconnect();
      server.begin(PORT);
    }

    if ( WiFi.isConnected() ) {
      WiFiClient client = server.available();  // listen for incoming clients
      if (client) {
        if (Serial) Serial.println("got client request from: " + client.remoteIP().toString() + ":" + String(client.remotePort()));
        if ( client.connected() ) {
          client.printf("[0] %s-13>WIDE1-1,TCPIP:=%s/%s_c...s...g...t%03.0fh%02.0fb%0.0fxBME ESP32 %.1fC %.1f%% %.1fhPa\n",
                        callsign, pos_ns, pos_ew, temp, humidity, pressure * 10.0, tempc, humidity, pressure);
          client.stop();
        }
      }

      // if no client, read BME280
      if (!i_bme280 == 0) {
        temp = bme280.readTempF();
        tempc = bme280.readTempC();
        humidity = bme280.readHumidity();
        pressure = bme280.readPressure() / pow(1.0 - 500.0 / 44330.0, 5.255);
      }

      if ( !mqttClient.connected() )
        connect_mqtt();

      if ( mqttClient.connected() ) {
        mqttClient.poll();
        getLocalTime(&timeinfo);
      }

      if ( mqttClient.connected() && nrloops <= 0) {  // only all 10 mins

        nrloops = 10 * 60 * 2;
        strftime(stemp, 80, "%F %T", &timeinfo);

        mqttClient.beginMessage(topic, true);
        mqttClient.printf("%.1f °C - %.0f %% - %.1f hPa", tempc, humidity, pressure);
        mqttClient.endMessage();

        mqttClient.beginMessage(topicd, true);
        mqttClient.print(stemp);
        mqttClient.endMessage();

        if (Serial) Serial.print("MQTT done - ");
        if (Serial) Serial.print("Date: ");
        if (Serial) Serial.println(stemp);
      }

      nrloops = nrloops - 1;
    }
  }
  delay(100);
}
