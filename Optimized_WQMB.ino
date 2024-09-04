#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// TDS METER

#define TdsSensorPin A0
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25,tempData;

// SSD1306

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

// DS18B20

#define ONE_WIRE_BUS 2  // DS18B20 data wire is connected to input 2

OneWire oneWire(ONE_WIRE_BUS);           // create a oneWire instance to communicate with temperature IC
DallasTemperature sensors(&oneWire);  // pass the oneWire reference to Dallas Temperature


void setup()
{
  Serial.begin(9600);
  pinMode(TdsSensorPin,INPUT);

  sensors.begin();

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();

  display.setTextSize(5);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("WQMB");
  display.display();
  Serial.println("WQMB");
  delay(2000);
  
}

void loop()
{
  getReading();
  oledDisplay();
}


void getReading(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT) 
        analogBufferIndex = 0;
   }   
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U)
  {
    printTimepoint = millis();
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
  }
  sensors.requestTemperatures();
  Serial.print("Temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(sensors.getTempCByIndex(0));
  Serial.print("°");
  Serial.println("C  ");
  delay(1000);
  tempData=sensors.getTempCByIndex(0);
  temperature=tempData;
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
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

void oledDisplay(){
  
  /// DISPLAYING TO OLED

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("DATA");
  //display.print("TDS:");
  display.print(tdsValue);
  display.println("ppm");

  display.cp437(true);
  //display.print("TEMP:");
  display.print(tempData);
  display.write(248);
  display.print("C");
  display.display();

  delay(2000);

  if((tdsValue<60)&&(tempData<200)){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Water is:");
    display.print("GOOD");
    display.display();
  }
  else{
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Water is:");
    display.print("BAD");
    display.display();
  }
  delay(1000);
}
