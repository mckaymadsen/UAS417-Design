/*  Filename:       sensor_CA.ino
    Author:         McKay Madsen
    Date:           1/20/19
    Version:        2.0
    Description:
      Code reads in the analog voltage values from 2 pins. This data corresponds to the front sensor
      and the top sensor. THis value is converted into a distance (in cm) using a formula show below.
      If the distance threshold is met, a flag is sent to the connected device via serial.
      Also reads data from the BMP 388 using the adafruit library
    Notes:
      ADC Library Documentation here:
      https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1

*/

#include <ADC.h>  //library from pedvide
#include <Wire.h> //I2C 
#include <Adafruit_Sensor.h>  //adafruit libraries
#include "Adafruit_BMP3XX.h"  //adafruit libraries

#define SEALEVELPRESSURE_HPA (1026.2) //as of 1/31/29

Adafruit_BMP3XX bmp; // I2C

const long interval = 5000;           // interval at which to send BMP data
const int a_pin_front = A2;
const int a_pin_top = A3;

ADC *adc = new ADC();

int object_flag = 0;
double voltage_front, distance_front;
double voltage_top, distance_top;

void setup() {
  //Open connection to Serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Begin setup\n");

  /* BMP SETUP START */
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  /* BMP SETUP END */
  

  /* CA SENSOR SETUP START*/
  pinMode(a_pin_front, INPUT);
  pinMode(a_pin_top, INPUT);

  //setup ADC 1
  adc->setAveraging(4, ADC_0); // set number of averages
  adc->setResolution(10, ADC_0); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);

  //set up ADC 2
  adc->setAveraging(4, ADC_1); // set number of averages
  adc->setResolution(10, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_1);
  /* CA SENSOR SETUP END */
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, 1); //show teensey is on

  Serial.println("End setup\n");
}



void read_CA_sensors() {
  /*  Used to read in the analog voltage output that is being sent by the XL-MaxSonar device.
      Scale factor is ((Vcc/1024 per 2cm.  A 5V supply yields ~4.9mV/2cm for 10.68 meter sensors.
      5V yields ~4.9mV/2cm., and 3.3V yields ~3.2mV/2cm. There

      https://billwaa.wordpress.com/2014/03/11/arduino-ultrasonic-range-finder-xl-maxsonar/
  */

  voltage_front = adc->analogRead(a_pin_front, ADC_0) * 0.0049 / 2;
  distance_front = (voltage_front / 0.0049) * 2 ;

  //top sensor
  voltage_top = adc->analogRead(a_pin_top, ADC_1) * 0.0032 / 2;
  distance_top = (voltage_top / 0.0032) * 2;

  adc->printError();
}

void check_flag() {
  if (distance_front < 600 || distance_top < 600) {
    object_flag = 1;
    Serial.write(object_flag);
  }
  else {
    object_flag = 0;
    //Serial.write(object_flag);
  }
}


unsigned long previousMillis = 0;        // will store last time LED was updated
void print_data_BMP_388() {
  //BMP388
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    //TEMP OFFSET = 1
    Serial.println();
    Serial.print("Temperature = ");
    Serial.print(bmp.temperature + 1);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    Serial.println();
  }
}

void print_data_CA_sensors() { //debug
  //debug function
  //CA Sensors
  Serial.print("\tdistance_front: ");
  Serial.print(distance_front);
  Serial.print("\tsensor_top: ");
  Serial.print(distance_top);
  Serial.println();
}

void loop() {

  read_CA_sensors();
  //check_flag();
  print_data_BMP_388();
  print_data_CA_sensors(); //debug
  delay(100);
}

