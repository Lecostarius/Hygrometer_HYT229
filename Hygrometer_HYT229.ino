#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Zanshin_BME680.h"

#define OLED_ADDR   0x3C
#define HYT221_ADDR 0x28

#define SCALE_MAX 16384.0
#define TEMP_OFFSET  40.0
#define TEMP_SCALE 165.0
#define HUM_SCALE 100.0

#define HYT_DEBUG 1
// https://forum.arduino.cc/t/hyt-221-from-hygrosens-instruments/54560/5

BME680_Class BME680;  ///< Create an instance of the BME680 class

class HYT221 {
    private:
        int address;
        int rawTemp;
        int rawHum;
        
    public:
        HYT221(int I2Cadr);
        uint8_t begin( void );
        uint8_t read( void );

        int getRawHumidity( void );
        int getRawTemperature( void ) ;
        double getHumidity( void ) ;
        double getTemperature( void );
  
};
HYT221::HYT221(int I2Cadr){
    address = I2Cadr;
}

uint8_t HYT221::begin(void) {
    return 1;
}

uint8_t HYT221::read( void ) {
    Wire.beginTransmission(address);
    Wire.write((byte)0x00);
    Wire.available();
    int Ack = Wire.read(); // receive a byte
    
    // DEBUG
    #if HYT_DEBUG
        Serial.print("ACK: ");
        Serial.println(Ack);
    #endif
    
    Wire.endTransmission();
    
    // DEBUG ////////////////
    //request 4 bytes
    #if HYT_DEBUG
        Wire.requestFrom(address, 4);

        Wire.available();

        int a1 = Wire.read(); // receive a byte
        int a2 = Wire.read(); // receive a byte
        int a3 = Wire.read(); // receive a byte
        int a4 = Wire.read(); // receive a byte
    #endif
    ////////////////////////////////
    
    // delay inteval !!! blocking MCU
    delay(100);

    //request 4 bytes
    Wire.requestFrom(address, 4);

    Wire.available();

    int b1 = Wire.read(); // receive a byte
    int b2 = Wire.read(); // receive a byte
    int b3 = Wire.read(); // receive a byte
    int b4 = Wire.read(); // receive a byte
    
    // DEBUG
    #if HYT_DEBUG
        Serial.print("a1: ");
        Serial.println(a1, BIN);
        Serial.print("b1: ");
        Serial.println(b1, BIN);
        Serial.print("a2: ");
        Serial.println(a2, BIN);
        Serial.print("b2: ");
        Serial.println(b2, BIN);
        Serial.print("a3: ");
        Serial.println(a3, BIN);
        Serial.print("b3: ");
        Serial.println(b3, BIN);
        Serial.print("a4: ");
        Serial.println(a4, BIN);
        Serial.print("b4: ");
        Serial.println(b4, BIN);
    #endif
    

    // combine the bits
    rawHum = ( b1 << 8 | b2 ) & 0x3FFF;

    // Mask away 2 last bits see HYT 221 doc
    rawTemp = b3 << 6 | ( unsigned(b4) >> 2 ) ;
    
    return 1;
}

int HYT221::getRawHumidity( void ) {
    return rawHum;
}

int HYT221::getRawTemperature( void ) {
    return rawTemp;
}

double HYT221::getHumidity( void ) {
    //hum = 100.0 / pow( 2, 14 ) * rawHum;
    return (HUM_SCALE * rawHum) / SCALE_MAX;
}

double HYT221::getTemperature( void ) {
    return ( (TEMP_SCALE * rawTemp) / SCALE_MAX ) - TEMP_OFFSET;
}


// here we go:

Adafruit_SSD1306 display(128, 32, &Wire, -1);
HYT221 hyt(HYT221_ADDR);

uint8_t have_hyt = 0;
uint8_t have_oled = 0;

// BME sensor stuff
int32_t temp, humidity, pressure, gas;

///< Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()

void setup() {
  int ack;
  Serial.begin(115200);
  Serial.println("Lecostarius");
  
  // check whether we have all devices on I2C bus:
  Wire.begin();
  Wire.requestFrom(HYT221_ADDR, 1);
  if (Wire.available()) have_hyt = 1;
  Wire.requestFrom(OLED_ADDR, 1);
  if (Wire.available()) have_oled = 1;

  if (have_oled == 0) {
    Serial.println("Found no OLED on I2C bus, stopping!");
    while (1);;
  }
  
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("Lecostarius");
  display.display();

  hyt.begin();
  
  display.setCursor(0,16);
  display.print("HYT: "); display.println(have_hyt);
  display.display();

  // BME680
  Serial.print(F("Starting I2CDemo example program for BME680\n"));
  Serial.print(F("- Initializing BME680 sensor\n"));
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  
  Serial.print(F("- Setting IIR filter to a value to OFF\n"));
  //BME680.setIIRFilter(IIR4);  // Use enumerated type values
  BME680.setIIRFilter(IIROff); // 
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds
}


void loop(){
  Serial.println("loop!");
  BME680.getSensorData(temp, humidity, pressure, gas); 
  Serial.print("BME680:"); Serial.print(temp); Serial.print(", pressure "); Serial.println(pressure);
   hyt.read();
   Serial.println("finished read");
   double h = hyt.getHumidity();
   double t = hyt.getTemperature();
   int hraw =  hyt.getRawHumidity() ;
   int traw =  hyt.getRawTemperature() ;
   Serial.print(t);
   Serial.print(" C, ");
   Serial.print(traw);
   Serial.print(" , ");
   Serial.print(h);
   Serial.print(" %, ");
   Serial.println(hraw);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(t); display.print(" C / ");
  display.print(temp / 100.0); display.print(" C");
  display.setCursor(0,12);
  display.print(h); display.print(" %rF / ");
  display.print(humidity); display.print(" %rF");
  display.setCursor(0,24);
  display.print(pressure/100.0); display.print(" mbar. Gas: ");
  display.print(gas);
  display.display();
   delay(3000);

}
