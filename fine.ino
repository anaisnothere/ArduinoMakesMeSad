//UV, CO2, TVOC, temp, pressure, altitude, humidity, gyrox, gyroy, gyroz, accelx, accely, accelz, magx, magy, magz,luminosity, data0, data1, distance, time(milis)

#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SparkFunTSL2561.h>
#include <Servo.h>


#define CCS811_ADDR 0x5B //Default I2C Address
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints
#define PIN_NOT_WAKE 5
//#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#define DECLINATION 5.88 // Declination (degrees) in Eforie Sud, Constanta, RO
//#define DECLINATION 5.96 // Declination (degrees) in Targu Ocna, Bacau, RO
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;
LSM9DS1 imu;
SFE_TSL2561 light;

static unsigned long lastPrint = 0; // Keep track of print time
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds
unsigned long time;

Servo servo1;
Servo servo2;
int reading = 0;
int pos = 10;
int trigger;
float time_s, time_s2, time_s3;

void setup()
{

//  servo1.attach(3);
//  servo1.write(180);
  servo1.attach(2);
  pinMode(4, INPUT);
  Wire.begin(); // join i2c bus
  

  Serial.begin(9600);
  Serial2.begin(19200);
  Serial3.begin(9600);
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  light.begin();




  unsigned char ID;
  light.getID(ID);
  gain = 0;
  unsigned char time = 2;
  light.setTiming(gain, time, ms);
  light.setPowerUp();

  myCCS811.begin();
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;

  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  myBME280.begin();

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  imu.begin();



}

void loop()
{
  trigger = pulseIn(4, HIGH, 25000);
  servo1.write(175);
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  time_s = millis();
  time_s2 = millis();
  time_s3 = millis();
  
 


  Serial3.print(uvIntensity);
  Serial3.print(",");

  myCCS811.dataAvailable();
  myCCS811.readAlgorithmResults();

  Serial3.print(myCCS811.getCO2());
  Serial3.print(",");
  Serial3.print(myCCS811.getTVOC());
  Serial3.print(",");
  Serial3.print(myBME280.readTempC(), 2);
  Serial3.print(",");
  Serial3.print(myBME280.readFloatPressure(), 2);
  Serial3.print(",");
  Serial3.print(myBME280.readFloatAltitudeMeters(), 2);
  Serial3.print(",");
  Serial3.print(myBME280.readFloatHumidity(), 2);
  Serial3.print(",");

  float BMEtempC = myBME280.readTempC();
  float BMEhumid = myBME280.readFloatHumidity();

  Serial3.print(BMEtempC);
  Serial3.print(",");
  Serial3.print(BMEhumid);
  Serial3.print(",");

  myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);

  imu.gyroAvailable();
  imu.readGyro();
  Serial3.print(imu.calcGyro(imu.gx), 2);
  Serial3.print(",");
  Serial3.print(imu.calcGyro(imu.gy), 2);
  Serial3.print(",");
  Serial3.print(imu.calcGyro(imu.gz), 2);
  Serial3.print(",");

  imu.accelAvailable();
  imu.readAccel();
  Serial3.print(imu.calcAccel(imu.ax), 2);
  Serial3.print(",");
  Serial3.print(imu.calcAccel(imu.ay), 2);
  Serial3.print(",");
  Serial3.print(imu.calcAccel(imu.az), 2);
  Serial3.print(",");

  imu.magAvailable();
  imu.readMag();
  Serial3.print(imu.calcMag(imu.mx), 2);
  Serial3.print(",");
  Serial3.print(imu.calcMag(imu.my), 2);
  Serial3.print(",");
  Serial3.print(imu.calcMag(imu.mz), 2);
  Serial3.print(",");

  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);

  unsigned int data0, data1;
  light.getData(data0, data1);
  double lux;    // Resulting lux value
  boolean good;  // True if neither sensor is saturated
  good = light.getLux(gain, ms, data0, data1, lux);
  Serial3.print(lux);
  Serial3.print(",");
  lidar_se3();
  Serial3.print(time_s3);
  Serial3.print(",");
  Serial3.println();
  





  Serial2.print("{TIMEPLOT:UV|data|UV|T|");
  Serial2.print(uvIntensity);
  Serial2.println("}");

  myCCS811.dataAvailable();
  myCCS811.readAlgorithmResults();

  Serial2.print("{TIMEPLOT:co2|data|co2|T|");
  Serial2.print(myCCS811.getCO2());
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:TVOC|data|TVOC|T|");
  Serial2.print(myCCS811.getTVOC());
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:temp|data|temp|T|");
  Serial2.print(myBME280.readTempC(), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:pressure|data|pressure|T|");
  Serial2.print(myBME280.readFloatPressure(), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:altitude|data|altitude|T|");
  Serial2.print(myBME280.readFloatAltitudeMeters(), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:humidity|data|humidity|T|");
  Serial2.print(myBME280.readFloatHumidity(), 2);
  Serial2.println("}");

  //float BMEtempC = myBME280.readTempC();
  //float BMEhumid = myBME280.readFloatHumidity();

  myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);

  imu.gyroAvailable();
  imu.readGyro();
  Serial2.print("{TIMEPLOT:gyrox|data|gyrox|T|");
  Serial2.print(imu.calcGyro(imu.gx), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:gyroy|data|gyroy|T|");
  Serial2.print(imu.calcGyro(imu.gy), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:gyroz|data|gyroz|T|");
  Serial2.print(imu.calcGyro(imu.gz), 2);
  Serial2.println("}");

  imu.accelAvailable();
  imu.readAccel();
  Serial2.print("{TIMEPLOT:accelx|data|accelx|T|");
  Serial2.print(imu.calcAccel(imu.ax), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:accely|data|accely|T|");
  Serial2.print(imu.calcAccel(imu.ay), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:accelz|data|accelz|T|");
  Serial2.print(imu.calcAccel(imu.az), 2);
  Serial2.println("}");

  imu.magAvailable();
  imu.readMag();
  Serial2.print("{TIMEPLOT:magx|data|magx|T|");
  Serial2.print(imu.calcMag(imu.mx), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:magy|data|magy|T|");
  Serial2.print(imu.calcMag(imu.my), 2);
  Serial2.println("}");
  Serial2.print("{TIMEPLOT:magz|data|magz|T|");
  Serial2.print(imu.calcMag(imu.mz), 2);
  Serial2.println("}");

  //unsigned int data0, data1;
  light.getData(data0, data1);
  //  double lux;    // Resulting lux value
  //  boolean good;  // True if neither sensor is saturated
  good = light.getLux(gain, ms, data0, data1, lux);
  Serial2.print("{TIMEPLOT:luminosity|data|luminosity|T|");
  Serial2.print(lux);
  Serial2.println("}");
  Serial2.println(time);
  delay(300);





  Serial.print(uvIntensity);
  Serial.print(",");

  myCCS811.dataAvailable();
  myCCS811.readAlgorithmResults();

  Serial.print(myCCS811.getCO2());
  Serial.print(",");
  Serial.print(myCCS811.getTVOC());
  Serial.print(",");
  Serial.print(myBME280.readTempC(), 2);
  Serial.print(",");
  Serial.print(myBME280.readFloatPressure(), 2);
  Serial.print(",");
  Serial.print(myBME280.readFloatAltitudeMeters(), 2);
  Serial.print(",");
  Serial.print(myBME280.readFloatHumidity(), 2);
  Serial.print(",");

  //float BMEtempC = myBME280.readTempC();
  //float BMEhumid = myBME280.readFloatHumidity();

  Serial.print(BMEtempC);
  Serial.print(",");
  Serial.print(BMEhumid);
  Serial.print(",");

  myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);

  imu.gyroAvailable();
  imu.readGyro();
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(",");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(",");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.print(",");

  imu.accelAvailable();
  imu.readAccel();
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(",");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(",");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.print(",");

  imu.magAvailable();
  imu.readMag();
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.print(",");

  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);

  //  unsigned int data0, data1;
  light.getData(data0, data1);
  //  double lux;    // Resulting lux value
  //  boolean good;  // True if neither sensor is saturated
  good = light.getLux(gain, ms, data0, data1, lux);
  Serial.print(lux);
  Serial.print(",");
  lidar();
  Serial.print(time_s);
  Serial.print(",");
  Serial.println();

  if (trigger > 1200 ) {
     
     
    while ( pos != 10) {
      servo1.write(pos);
      Serial.print(",,,,,,,,,,,,,,,,,,,");;
      Serial3.print(",,,,,,,,,,,,,,,,,,,");
      lidar();
      Serial.println(pos);
      lidar_se3();
      Serial3.println(pos);
      pos--;
      //delay(20);
    }
    while ( pos != 170) {
      servo1.write(pos);
      Serial.print(",,,,,,,,,,,,,,,,,,,");
      Serial3.print(",,,,,,,,,,,,,,,,,,,");
      lidar();
      Serial.println(pos);
      lidar_se3();
      Serial3.println(pos);
      pos++;
      //delay(20);
    }
  }

}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  //  Serial3.print(pitch, 2);
  //  Serial3.print(",");
  //  Serial3.print(roll, 2);
  //  Serial3.print(",");
  //  Serial3.print(heading, 2);
  //  Serial3.print(",");
}


int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void lidar() {
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)
  Wire.endTransmission(); // stop transmitting
  delay(20); // Wait 20ms for transmit
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting
  delay(20); // Wait 20ms for transmit
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if (2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print(reading); // print the reading
    Serial.print(",");
  }
}


void lidar_se3() {
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)
  Wire.endTransmission(); // stop transmitting
  delay(20); // Wait 20ms for transmit
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting
  delay(20); // Wait 20ms for transmit
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if (2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial3.print(reading); // print the reading
    Serial3.print(",");
  }
}

