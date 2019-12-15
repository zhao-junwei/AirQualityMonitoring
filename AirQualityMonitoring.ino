#include <SoftwareSerial.h>
#include "U8glib.h"

// setup u8g object, please remove comment from one of the following constructor calls
// IMPORTANT NOTE: The following list is incomplete. The complete list of supported 
// devices with all constructor calls is here: http://code.google.com/p/u8glib/wiki/device
U8GLIB_SSD1306_128X64 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9

// define the data structure for pms5003
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

SoftwareSerial oxySerial(2, 3);
SoftwareSerial pmsSerial(4, 5);

//get oxymethylene:DART WZ-S sensor
float getOxymethylene(void)
{
  unsigned char HeadGegin = 0;
  float oxymethylene = 0.0;
  int index = 0;
  unsigned char buf[50];
  while (oxySerial.available()) //return 01 03 20 *******
  {
    // get the new byte:
    unsigned char inChar = (unsigned char) oxySerial.read();
    //Serial.write(inChar);
    if (!HeadGegin) // packet head
    {
      if (inChar == 0xFF)
      {
        buf[index++] = inChar;
        inChar = (unsigned char) oxySerial.read();
        //Serial.write(inChar);
        if (inChar == 0x17)
        {
          buf[index++] = inChar;
          inChar = (unsigned char) oxySerial.read();
          //Serial.write(inChar);
          if (inChar == 0x04)
          {
            buf[index++] = inChar;
            HeadGegin = 1;
          }
        }
      }

    }
    else //start retrieve data
    {
      buf[index++] = inChar;
    }
  }
  Serial.print("get index:");
  Serial.println(index);

  if (index >= 9) //9 data received
  {
    unsigned long int temp = buf[4] * 256 + buf[5];
    oxymethylene = (float) (temp * 0.0013393); //oxymethylene_f
  }
  else 
  {
   oxymethylene = -1.0;
  }

  //Serial.println(oxymethylene);
  return  oxymethylene;
}


// get pmsdata from pms5003 sensor
boolean readPMSdata(Stream *s, pms5003data *pmsdata)
{
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)pmsdata, (void *)buffer_u16, 30);
 
  if (sum != pmsdata->checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

void setup(void)
{
    Serial.begin(9600);
    oxySerial.begin(9600);
    pmsSerial.begin(9600);
    //Serial.println("Started...");
}

void loop(void)
{
  // read CH2O first
  oxySerial.listen();
  delay(100);
  static float ch2o = -1.0;
  float oxy = getOxymethylene();
  if (oxy > 0.0) 
    ch2o = oxy;
  Serial.print("get oxy:");
  Serial.println(oxy);

  struct pms5003data pmsdata;
  static int pm25 = -1;
  static int pm100 = -1;

  //Switch to read PMS data
  pmsSerial.listen();
  delay(100);
  if (readPMSdata(&pmsSerial, &pmsdata))
  {
    pm25 = pmsdata.pm25_standard;
    pm100 = pmsdata.pm100_standard;
    Serial.println("pm25std, pm100std, pm25env, pm100env");
    Serial.println(pmsdata.pm25_standard);
    Serial.println(pmsdata.pm100_standard);
    Serial.println(pmsdata.pm25_env);
    Serial.println(pmsdata.pm100_env);
  }

  else
  {
    Serial.println("PMS read failed.");
  }

  // picture loop
  u8g.firstPage();  
  do {
        u8g.setFont(u8g_font_unifont);
        
        // output CH2O
        u8g.setPrintPos(0, 20); 
        u8g.print("CH2O:");
        u8g.setPrintPos(60, 20); 
        if (ch2o < 0.0)
        {
          u8g.print("N/A");
        }
        else
        {
          u8g.print(ch2o);
        }

        // output PM2.5
        u8g.setPrintPos(0, 40); 
        u8g.print("PM2.5:");
        u8g.setPrintPos(60, 40); 
        if (pm25 == -1)
        {
          u8g.print("N/A");
        }
        else
        {
          u8g.print(pm25);
        }

        // output PM10
        u8g.setPrintPos(0, 60); 
        u8g.print("PM10:");
        u8g.setPrintPos(60, 60); 
        if (pm100 == -1)
        {
          u8g.print("N/A");
        }
        else
        {
          u8g.print(pm100);
        }

  } while( u8g.nextPage() );
  
  // rebuild the picture after some delay
  delay(1000);
}
