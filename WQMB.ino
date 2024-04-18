
#include <WiFiS3.h>
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/// LED Matrix Display

ArduinoLEDMatrix matrix;

const uint32_t HEART[][4] = {
  { 0x1983fc7f,
    0xe7fe3fc1,
    0xf80f0060,
    200 },
  { 0x1983f,
    0xc3fc1f80,
    0xf0060000,
    500 },
  { 0x1983fc7f,
    0xe7fe3fc1,
    0xf80f0060,
    500 },
  { 0x1983f,
    0xc3fc1f80,
    0xf0060000,
    500 },
  { 0x1983fc7f,
    0xe7fe3fc1,
    0xf80f0060,
    500 }
};

/// SSD1306 AKA OLED DISPLAY

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

/// DS18B20

#define ONE_WIRE_BUS 2  // DS18B20 data wire is connected to input 2

DeviceAddress thermometerAddress;  // custom array type to hold 64 bit device address

OneWire oneWire(ONE_WIRE_BUS);           // create a oneWire instance to communicate with temperature IC
DallasTemperature tempSensor(&oneWire);  // pass the oneWire reference to Dallas Temperature

/// TDS METER

#define TdsSensorPin A0
#define VREF 5.0           // analog reference voltage(Volt) of the ADC
#define SCOUNT 30          // sum of sample point
int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;
float tempData;

void setup() {
  Serial.begin(9600);
  Serial.println("DS18B20 Temperature IC Test");
  Serial.println("Locating devices...");
  tempSensor.begin();
  pinMode(TdsSensorPin, INPUT);
  matrix.loadSequence(HEART);
  matrix.begin();
  matrix.play(true);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();

  display.setTextSize(5);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("WQMB");

  display.display();
  delay(2000);

  if (!tempSensor.getAddress(thermometerAddress, 0))
    Serial.println("Unable to find Device.");
  else {
    Serial.print("Device Address: ");
    printAddress(thermometerAddress);
    Serial.println();
  }
  tempSensor.setResolution(thermometerAddress, 9);  // set the temperature resolution (9-12)



}

void loop() {

  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(100);

  tempSensor.requestTemperatures();                      // request temperature sample from sensor on the one wire bus
  displayTemp(tempSensor.getTempC(thermometerAddress));  // show temperature on OLED display

  delay(500);

  const char text[] = "  WQMB  ";
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text);
  matrix.endText(SCROLL_LEFT);

  matrix.endDraw();

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 500U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

  }

  displayData();
}
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
void displayTemp(float temperatureReading) {  // temperature comes in as a float with 2 decimal places

  // show temperature °C
  Serial.print(temperatureReading);  // serial debug output
  Serial.print("°");
  Serial.print("C  ");

  // show temperature °F
  Serial.print(DallasTemperature::toFahrenheit(temperatureReading));  // serial debug output
  Serial.print("°");
  Serial.println("F");

  tempData=temperatureReading;

}

void displayData(){

  /// Displaying on to the OLED DISPLAY

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("   WQMB");
  display.display();
  delay(2000);


  /// TDS VALUE

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("TDS :");
  display.print(tdsValue);
  display.print("ppm");
  display.display();

  delay(3000);

  /// Temperature

  display.clearDisplay();
  display.setTextSize(2);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println("Temp :");
  display.print(tempData);
  display.write(248);
  display.print("C");
  display.display();

  delay(3000);

  /// Water Quality

  if((tempData<39)&&(tdsValue<60)){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("");
    display.println("Water is");
    display.print("GOOD");
    display.display();
  }
  else{
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("");
    display.println("Water is");
    display.print("BAD");
    display.display();
  }
  delay(2000);
}


// print device address from the address array
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}