PRODUCT_ID(12447);
PRODUCT_VERSION(4);

// This #include statement was automatically added by the Particle IDE.
#include <SparkFunMAX17043.h>

/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10
#define BME280_ENABLE D2

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

double tempF = 0.0;
double tempC = 0.0;
double humidity = 0.0;
double pressure = 0.0;
double altitude = 0.0;

float voltage = 0.0;
float chargeStatus = 0.0;

String deviceName = "default";

// Open a serial terminal and see the device name printed out
void handler(const char *topic, const char *data)
{
  deviceName = String::format("%s", data);
  String deviceNameData = String::format("topic=%s data=%s", topic, deviceName.c_str());
  Particle.publish("device-name-data", deviceNameData, PRIVATE);
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("BME280 test"));

  Particle.subscribe("particle/device/name", handler);
  Particle.publish("particle/device/name");

  pinMode(BME280_ENABLE, OUTPUT);

  Particle.variable("tempF", tempF);
  Particle.variable("tempC", tempC);
  Particle.variable("humidity", humidity);
  Particle.variable("pressure", pressure);

  // Set up the MAX17043 LiPo fuel gauge:
  lipo.begin(); // Initialize the MAX17043 LiPo fuel gauge

  // Quick start restarts the MAX17043 in hopes of getting a more accurate
  // guess for the SOC.
  lipo.quickStart();
}

void loop()
{

  if (deviceName != "default")
  {
    printValues();
    printVoltage();
    System.sleep(SLEEP_MODE_DEEP, 60);
  }
  else
  {
    delay(10000);
  }
}

void printVoltage()
{
  // lipo.getVoltage() returns a voltage value (e.g. 3.93)
  voltage = lipo.getVoltage();
  // lipo.getSOC() returns the estimated state of charge (e.g. 79%)
  chargeStatus = lipo.getSOC();
  Serial.print("Voltage = ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("Charge State = ");
  Serial.print(chargeStatus);
  Serial.println(" %");

  String data = String::format("{ \"tags\" : {\"location\": \"%s\"},\"values\": {\"voltage\": %.2f, \"chargeStatus\": %.2f}}", deviceName.c_str(), voltage, chargeStatus);
  Particle.publish("influx-voltage-data", data, PRIVATE);
}

void printValues()
{
  digitalWrite(BME280_ENABLE, HIGH);
  delay(1000);
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  tempC = bme.readTemperature();
  tempF = tempC * (9 / 5) + 32;
  pressure = bme.readPressure() / 100.0F;
  humidity = bme.readHumidity();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.print("Temperature = ");
  Serial.print(tempC);
  Serial.println(" *C");

  Serial.print("Temperature = ");
  Serial.print(tempF);
  Serial.println(" *F");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();

  String data = String::format("{ \"tags\" : {\"location\": \"%s\"},\"values\": {\"tempC\": %.2f, \"tempF\": %.2f, \"pressure\": %.1f,\"humidity\": %.1f,\"altitude\": %.1f}}", deviceName.c_str(), tempC, tempF, pressure, humidity, altitude);
  Particle.publish("influx-temperature-data", data, PRIVATE);
  digitalWrite(BME280_ENABLE, LOW);
}