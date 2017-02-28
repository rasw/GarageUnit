// ***************************************************************************
// i2c wiring
// Vin = +3.3V
// GND = Gnd
// SCK = SCL last i/o pin
// SDI = SDA second in from last i/o pin
// ***************************************************************************

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>    // library I2C address = 0x77h

#define BLUELED 12
#define GREENLED 10
#define REDLED 11

int LightSensorPin = A5;
float LightSensor = 0;
float temperature = 0;
float pressure = 0;
float humidity = 0;
float temperatureCalibration = 0.966;   // Temperature calibration correction value 0.915 drops by -1 oC
float pressureCalibration = 1.00701;    // Matched to office baro Pressure calibration correction value 1.00701
float humidityCalibration = 1.0;        // Humidity calibration correction value

long checkSum = 0;                      // zigbee checksum
byte BME280_TemperatureByteArray[4];    // holds the 4 byte IEEE754 convertion for zigbee frame
byte BME280_HumidityByteArray[4];
byte BME280_BaroByteArray[4];
byte LightLevelByteArray[4];
int PowerUpMessagesCount = 0;           // Sends 30 messages every 10 seconds, then every 10 minutes
bool PostPowerUp = false;
byte TransmitStatus[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
byte success = false;                   // true = success
int x = 0;                              // arrary index
    

Adafruit_BME280 bme;                     // I2C
//Adafruit_BME280 bme(BME_CS);           // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(BLUELED, OUTPUT);  
  pinMode(GREENLED, OUTPUT); 
  pinMode(REDLED, OUTPUT); 
  pinMode(LED_BUILTIN, OUTPUT);  

  if (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    while (1) {FlashFlashREDLED();}
  }
   LampTest();
}

void loop() {
   
    LightSensor = GetLightReading(); // read the value from the light sensor and map to 0-100%:
    digitalWrite(LED_BUILTIN, HIGH);
    ReadSensors();
    digitalWrite(LED_BUILTIN, LOW);
    SendDataFrame();
    digitalWrite(LED_BUILTIN, HIGH); 
    ProcessReply();
    digitalWrite(LED_BUILTIN, LOW); 
    
    if(PowerUpMessagesCount < 31)
    {
      if(!PostPowerUp)
      {
        delay(10000);  // delay 10 seconds (30 messages at power up)
        FlashRedLED(100);
        delay(100);
        FlashBlueLED(100);
      }
    }
    else
    {
        delay(300000);  // delay 5 mins
        PostPowerUp = true;
        if(LightSensor < 50)
        {
          FlashBlueLED(10);
        }
        else
        {
          FlashBlueLED(100);
        }
    } 
    
    PowerUpMessagesCount++;
}

void SendDataFrame()
{
 // To calculate the check sum you add all bytes of the packet excluding the frame delimiter 7E and the length (the 2nd and 3rd bytes).
 // To calcualte length total byte count minus 3 ( first 3 bytes and checksum byte not included)
 // MAKE SURE THE Flow control is diabled on the XBEE !!
  long checkSum = 0x00;
  //unsigned long checkSum = 0x00;
   
  Serial1.write((byte)0x7E);    //start byte
  Serial1.write((byte)0x00);    // high part of lenght, alway zero
  Serial1.write((byte)0x27);    // low part of lenght, number of bytes 0x27 = 39 dec
  Serial1.write((byte)0x10); checkSum += 0x10;    // 0x10 send request
  Serial1.write((byte)0x01); checkSum += 0x01;    // frame id set to zero for no reply, 01 for reply
  
  // ID of recipient, CO-ORDINATOR
  Serial1.write((byte)0x00); //checkSum += 0x00;
  Serial1.write((byte)0x13); checkSum += 0x13;
  Serial1.write((byte)0xA2); checkSum += 0xA2;
  Serial1.write((byte)0x00); checkSum += 0x00;
  Serial1.write((byte)0x41); checkSum += 0x41;
  Serial1.write((byte)0x63); checkSum += 0x63;
  Serial1.write((byte)0x18); checkSum += 0x18;
  Serial1.write((byte)0x45); checkSum += 0x45;

  // 16 bit of recipients or 0xFFFE if unkown
  Serial1.write((byte)0xFF); checkSum += 0xFF;
  Serial1.write((byte)0xFE); checkSum += 0xFE;
  Serial1.write((byte)0x00); //checkSum += 0x00;
  Serial1.write((byte)0x00); //checkSum += 0x00;

  // checkSum = 0x3C4;  //437;  // claculation of checksum from above bytes hex
   
  // Data frame 
  // ID
  Serial1.write((byte)0x24); checkSum += 0x24; // $ 
  Serial1.write((byte)0x30); checkSum += 0x30; // 0 
  Serial1.write((byte)0x30); checkSum += 0x30; // 0
  Serial1.write((byte)0x37); checkSum += 0x37; // 7
  Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  
  // BME280 Temperature  
   memcpy(BME280_TemperatureByteArray, &temperature,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)BME280_TemperatureByteArray[i]);
     checkSum += BME280_TemperatureByteArray[i];
   }
   Serial1.write((byte)0x7C); checkSum += 0x7C; // |
  //---------------------------------------------------------------------------------------
  
  // BME 280 Humidity
   memcpy(BME280_HumidityByteArray, &humidity,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)BME280_HumidityByteArray[i]);
     checkSum += BME280_HumidityByteArray[i];
   }
    Serial1.write((byte)0x7C); checkSum += 0x7C; // |
   //---------------------------------------------------------------------------------------

   // BME 280 Barometer
   memcpy(BME280_BaroByteArray, &pressure,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)BME280_BaroByteArray[i]);
     checkSum += BME280_BaroByteArray[i];
   }
     Serial1.write((byte)0x7C); checkSum += 0x7C; // |
   //---------------------------------------------------------------------------------------

   // Light Level
   memcpy(LightLevelByteArray, &LightSensor,4);
   for(int i = 0; i < 4; i++)
   {
     Serial1.write((byte)LightLevelByteArray[i]);
     checkSum += LightLevelByteArray[i];
   }
    Serial1.write((byte)0x23); checkSum += 0x23; // #
   //---------------------------------------------------------------------------------------

    Serial1.write((byte)0xFF - (checkSum & 0xFF));  // calc and send checksum
    checkSum = 0; // Reset checksum
   
  //  long sum = 0x17 + 0xFF + 0xFF + 0xFF + 0xFE + 0x02 +'D' + '1' + value;
  //  Serial11.write(0xFF - (sum & 0xFF), BYTE);  calc checksum
}  

void ReadSensors()
{
    temperature = bme.readTemperature();
    temperature = (temperature * temperatureCalibration);
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    pressure = bme.readPressure() / 100.0F;
    pressure = (pressure * pressureCalibration);
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");

    humidity = bme.readHumidity();
    humidity = (humidity * humidityCalibration); 
    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Light % = ");
    Serial.println(LightSensor);
    Serial.println("");
}

int GetLightReading()
{
   LightSensor = analogRead(LightSensorPin);
    
    if(LightSensor < 750)
       LightSensor = 750;
    else if (LightSensor > 900)
       LightSensor = 900;
    
    return map(LightSensor, 750, 900, 0, 100);
}

void LampTest()
{
  for(int l = 0; l < 25; l++)
  {
     digitalWrite(REDLED,HIGH);
     delay(120);
     digitalWrite(REDLED,LOW);
     digitalWrite(BLUELED,HIGH);
     delay(120);
     digitalWrite(BLUELED,LOW);
     digitalWrite(GREENLED,HIGH);
     delay(120);
     digitalWrite(GREENLED,LOW);
     delay(120);
  }
}

void FlashFlashREDLED()
{
   for(int q = 0; q < 81;q++)
   {
      digitalWrite(REDLED,HIGH);
      delay(50);
      digitalWrite(REDLED,LOW);
      delay(50);
   }
      digitalWrite(REDLED,LOW);
}

void ProcessReply()
{
    //byte TransmitStatus[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    //byte success = false;  // true = success
    //int x = 0; // arrary index
    delay(1000);
    while(Serial1.available() > 0)
    {
        TransmitStatus[x] = Serial1.read();  // x 0-10 (11 bytes)
        //Serial.write((byte)TransmitStatus[x]);
        x++;
    }
        x = 0;                                // reset index

    if(TransmitStatus[0] == 0x7E)            // check for frame start
    {
        if(TransmitStatus[3] == 0x8B)        // Frame type 8B = reply
        {
            if(TransmitStatus[8] == 0x00)    // 0x00 is Success
            {
                delay(100);
                success = 0x00;  // Success
                
                if(LightSensor < 50)
                {
                  FlashGreenLED(10);
                  delay(120);
                  FlashGreenLED(10);
                }
                else
                {
                  FlashGreenLED(100);
                  delay(120);
                  FlashGreenLED(100);
                }
            }
            else
            {
                success = 0xFF;  // failure
            }
        }
        else
        {
            success = 0xFF;      // failure
        }
      
    if(success != 0x00)
    {
        FlashFlashREDLED();       // red led
    }
  }
    
    for(int z = 0; z < 10; z++)
    {
      TransmitStatus[z] = 0x00;  // clean out the xbee reply frame buffer
    }   
}

void FlashGreenLED( int Brightness)
{
  for(int f = 0; f<200; f++)
  {
    digitalWrite(GREENLED,HIGH);  // if you want use the analogpins as digital than A0-> 14 ,A1->15,..etc
    delayMicroseconds(Brightness);
    digitalWrite(GREENLED,LOW);
    delayMicroseconds(900);
  }
}

void FlashRedLED( int Brightness)
{
  for(int f = 0; f<200; f++)
  {
    digitalWrite(REDLED,HIGH);  // if you want use the analogpins as digital than A0-> 14 ,A1->15,..etc
    delayMicroseconds(Brightness);
    digitalWrite(REDLED,LOW);
    delayMicroseconds(900);
  }
}

void FlashBlueLED(int Brightness)
{
  for(int f = 0; f<200; f++)
  {
    digitalWrite(BLUELED,HIGH);  // if you want use the analogpins as digital than A0-> 14 ,A1->15,..etc
    delayMicroseconds(Brightness);
    digitalWrite(BLUELED,LOW);
    delayMicroseconds(900);
  }
}


