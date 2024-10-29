#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>

#define OLED_ADDR   0x3C
#define HYT221_ADDR 0x28

#define SCALE_MAX 16384.0
#define TEMP_OFFSET  40.0
#define TEMP_SCALE 165.0
#define HUM_SCALE 100.0

//#define HYT_DEBUG 1
// https://forum.arduino.cc/t/hyt-221-from-hygrosens-instruments/54560/5

Adafruit_BMP280 bmp;

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
  display.setCursor(0,24);
  if (bmp.begin(0x76),88) {
    display.print("BMP280: OK");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X8,      /* Filtering. use FILTER_X2 or FILTER_X16 */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.println(bmp.sensorID(),16);

  } else {
    display.print("BMP280 NOT FOUND");
  }
  display.display();
  delay(1000);
}


void loop(){
  Serial.println("loop!");
  Serial.println(bmp.sensorID(),16);
  Serial.println(bmp.getStatus());
  Serial.println(bmp.readTemperature());
   hyt.read();
   Serial.println("finished read");
   double h = hyt.getHumidity();
   double t = hyt.getTemperature();
   double p = bmp.readPressure();
   int hraw =  hyt.getRawHumidity() ;
   int traw =  hyt.getRawTemperature() ;
   float alt = bmp.readAltitude();
   Serial.print(t);
   Serial.print(" C, ");
   Serial.print(traw);
   Serial.print(" , ");
   Serial.print(h);
   Serial.print(" %, ");
   Serial.println(hraw);
   Serial.print("pressure: ");
   Serial.println(p);

  display.clearDisplay();
  display.setCursor(0,0);
  display.print(t); display.print(" C / ");
  
  display.setCursor(0,12);
  display.print(h); display.print(" %rF / ");
  display.setCursor(0,24);
  display.print(p );
  display.print(" Pa / ");
  display.print(alt);
  display.print("m");
  
  display.display();
   delay(3000);

}
