// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char SSID[]     = "capt_ph03nix";    // Network SSID (name)
const char PASS[]     = "password";    // Network password (use for WPA, or use as key for WEP)


float tdsMeter;
float tempSensor;
bool wifiConnected;

void initProperties(){

  ArduinoCloud.addProperty(tdsMeter, READ, 3 * SECONDS, NULL);
  ArduinoCloud.addProperty(tempSensor, READ, 3 * SECONDS, NULL);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
