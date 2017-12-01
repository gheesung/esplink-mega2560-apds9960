/*
  esplink-mqtt.ino - ESP-Link and MQTT 

  Copyright (C) 2017  Ghee Sung

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Simple example to demo the esp-link MQTT client using APDS sensor
 * to send the data to MQTT 
 */
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
#include <ArduinoJson.h>

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
ELClient esp(&Serial3, &Serial);
// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins
void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
    } else {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
    }
  }
}

bool connected;

// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println("MQTT connected!");
  mqtt.subscribe("brownstone/esp-link/mcu/gpio/cmnd");
  mqtt.subscribe("brownstone/esp-link/mcu/cmnd");
  mqtt.subscribe("brownstone/esp-link/status");
  //mqtt.subscribe("/esp-link/2", 1);
  mqtt.publish("brownstone/esp-link/mcu/status", "MCU connected");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println("MQTT disconnected");
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  Serial.print("dataaaaa=");
  String data = res->popString();
  Serial.println(data);
}

void mqttPublished(void* response) {
  Serial.println("MQTT published");
}


void setup() {

  Serial.begin(115200);
  Serial3.begin(115200);
  delay(2500);// delay to let the ESP to be ready for syncing
  Serial.println("EL-Client starting!");
  esp._debugEn = false;
  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  //esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  bool ok;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) Serial.println("EL-Client sync failed!");
  } while(!ok);
  Serial.println("EL-Client synced!");
  
  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();

  //Serial.println("ARDUINO: setup mqtt lwt");
  //mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");

  Serial.println("EL-MQTT ready");

  // Initialize Serial port
  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - ColorSensor"));
  Serial.println(F("--------------------------------"));
  
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
  
}

// Variables used in the loop
static int count;
static uint32_t last;
static int detected = 0;
DynamicJsonBuffer jsonBuffer;;
JsonObject& root = jsonBuffer.createObject();

void loop() {
  esp.Process();

  if (connected && (millis()-last) > 30000) {
    // Read the light levels (ambient, red, green, blue)
    if (  !apds.readAmbientLight(ambient_light) ||
          !apds.readRedLight(red_light) ||
          !apds.readGreenLight(green_light) ||
          !apds.readBlueLight(blue_light) ) {
      Serial.println("Error reading light values");
    } else {
      Serial.print("Ambient: ");
      Serial.print(ambient_light);
      Serial.print(" Red: ");
      Serial.print(red_light);
      Serial.print(" Green: ");
      Serial.print(green_light);
      Serial.print(" Blue: ");
      Serial.println(blue_light);
      root["sensor"] = "adps9960"; 
      root["ambient"] = ambient_light;     
      root["red"] = red_light;     
      root["green"] = green_light;     
      root["blue"] = blue_light;     
      String output;
      root.printTo(output);
      mqtt.publish("brownstone/esp-link/color/status", output.c_str());
    }

    last = millis();
  }
}
