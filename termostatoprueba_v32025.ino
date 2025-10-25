 
  #include <WiFi.h>
 #include <Wire.h>
#include <InfluxDbClient.h>
  #include <InfluxDbCloud.h>
  #include "AESLib.h"
  const int electrovalvula = 16;
const int fan1 = 18;
const int fan2 = 26;
const int fan3 = 27;
  int8_t sda=5;
  int8_t scl=22;
// Update these with values suitable for your network.
// BMP280
/*******************************/
float temperature_BMP280;
float pressure;
float tempC; 


// Temperature variable
int32_t _t_fine;


// Trimming parameters
uint16_t _dig_T1;
int16_t _dig_T2;
int16_t _dig_T3;

uint16_t _dig_P1;
int16_t _dig_P2;
int16_t _dig_P3;
int16_t _dig_P4;
int16_t _dig_P5;
int16_t _dig_P6;
int16_t _dig_P7;
int16_t _dig_P8;
int16_t _dig_P9;
/*******************************/
//Datos exteriores
/*******************************/
float Temperatura_impulsion;
float  Temperatura_exterior;
float Humedad_exterior;
float consigna;




//AHT20
/*******************************/
float temperature_AHT20;
float humidity;
bool sensor_started = false;
bool sensor_busy = false;
unsigned long measurementDelayAHT20 = 0;
int cntAHT;
/*******************************/
//influxdB
char temperaturaAHT[8];
char temperaturaBMP[8];
char presion[8];
char humedad[8];
char temperaturaImpulsion[8];
char temperaturaExterior[8];
char HumedadExterior[8];
char temperaturaConsigna[8];
//int contador;

float delta = 0;
float minDelta = 10;
float maxDelta = 0;


//  Heartbeat
unsigned long HeartbeatMillis = 0;
const long Heartbeatinterval = 5000;
//const char* ssid = "Domotica";
//const char* password = "electricitat";
const char* ssid = "MIWIFI_k7dD";
const char* password = "7EyEcNEx";

//const char* mqtt_server = "test.mosquitto.org"; // MQTT broker
WiFiClient espClient;

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

unsigned long lastSend = 0;
unsigned long lastSend2 = 0;  // Para controlar el intervalo de envío
 #define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"
 #define INFLUXDB_TOKEN "zWpIX5zHbT_ONNgZ-oCeCQyBHLQRwD3Gl9rYYJE_3LS0kvPud8CazVzE3DDFd6j-mh1EHA_U_0l4g71DYxUMqg=="
  #define INFLUXDB_ORG "f226e725bae2019c"
  #define INFLUXDB_BUCKET "Datos-CIFP_Politecnic_de_Llevant"
  
  // Time zone info
  #define TZ_INFO "UTC1"
  
  // Declare InfluxDB client instance with preconfigured InfluxCloud certificate
  InfluxDBClient client1(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
  
  // Declare Data point
 Point point("Termostato");

AESLib aesLib;

#define INPUT_BUFFER_LIMIT (128 + 1) // designed for Arduino UNO, not stress-tested anymore (this works with readBuffer[129])

unsigned char cleartext[INPUT_BUFFER_LIMIT] = {0}; // THIS IS INPUT BUFFER (FOR TEXT)
unsigned char ciphertext[2*INPUT_BUFFER_LIMIT] = {0}; // THIS IS OUTPUT BUFFER (FOR BASE64-ENCODED ENCRYPTED DATA)

//unsigned char readBuffer[18] = "username:password";
unsigned char readBuffer[16];

// AES Encryption Key (same as in node-js example)
// General initialization vector (same as in node-js example) (you must use your own IV's in production for full security!!!)


byte aes_key[] = { 0x9B, 0x40, 0x71, 0x1B, 0x90, 0x2D, 0xDB, 0x41, 0xD4, 0xE8, 0x8A, 0xC, 0x9, 0xCB, 0x8F, 0xDA };
byte aes_iv[N_BLOCK] = { 0x71, 0x9C, 0xDB, 0x44, 0x3A, 0x85, 0xF9, 0x1A, 0xF3, 0x7F, 0xAC, 0x0, 0x7E, 0x88, 0x80, 0x1C };
byte enc_iv[N_BLOCK]= { 0xD3, 0x54, 0x4F, 0xCA, 0x18, 0x72, 0x91, 0x13, 0x1B, 0x87, 0x79, 0xB2, 0xEE, 0x73, 0xFC, 0x7B };
byte enc_iv_to[N_BLOCK]  = { 0x72, 0xD7, 0xA4, 0x5F, 0x27, 0xFC, 0xB, 0xDC, 0xCD, 0x77, 0x24, 0xC3, 0xF9, 0xA0, 0x2E, 0xD0 };
byte enc_iv_from[N_BLOCK] = { 0xEB, 0x23, 0x51, 0x99, 0x3, 0xD4, 0x52, 0xA8, 0xA3, 0x5F, 0xBF, 0x6D, 0xD5, 0xFB, 0x69, 0x49 };




#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

TFT_eSPI tft = TFT_eSPI();

// Touchscreen pins
#define XPT2046_IRQ 34   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 35  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 3





// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;


// Stores current button state
bool buttonState = false;

// Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
void printTouchToSerial(int touchX, int touchY, int touchZ) {
  Serial.print("X = ");
  Serial.print(touchX);
  Serial.print(" | Y = ");
  Serial.print(touchY);
  Serial.print(" | Pressure = ");
  Serial.print(touchZ);
  Serial.println();
}

// Draw button frame
void drawFrame() {
 tft.drawRect(0,260,90,340, TFT_BLACK);
  tft.drawRect(90,260,167,340, TFT_BLACK); 
  tft.drawRect(167,260,240,340, TFT_BLACK); 
}

// Draw a red button
void drawONButton() {
  tft.fillRect(0,260,90,80, TFT_CYAN);
 
  drawFrame();
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("ON", 40,290);
  buttonState =true;
 }

// Draw a green button
void drawOFFButton() {
  tft.fillRect(0,260,90,80, TFT_WHITE);
  //tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, TFT_WHITE);
   drawFrame();
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("OFF", 40, 290);
  buttonState =false;
  
}


void escritorio(){
   tft.fillRect(90,260,90,80, TFT_CYAN);
  tft.fillRect(170,260,80,80, TFT_YELLOW);
  drawFrame();
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
    tft.drawString("OFF", 40, 290);
  tft.drawString("+  -", 170, 290);
   tft.setTextColor(TFT_BLACK);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("T=15,00C",120, 50);
   tft.drawString("H=60,39%", 120, 100);
   tft.drawString("T=19,00C", 120, 230);
  tft.setTextColor(TFT_BLUE);
  tft.setTextSize(2);
     tft.drawString("impulsion  exterior", 115, 130);
   tft.drawString("consigna", 120, 190);
   tft.drawString("interior", 120, 20);
     // tft.drawLine(110,0,110,240, TFT_BLACK);
   //tft.drawLine(220,0,220,240, TFT_BLACK);
   tft.drawLine(0,120,240,120, TFT_BLACK);
   tft.drawLine(0,180,240,180, TFT_BLACK);
   //tft.drawLine(110,120,220,120, TFT_BLACK);
  // tft.drawLine(220,90,340,90, TFT_BLACK);
   tft.setTextColor(TFT_BLACK);
 tft.drawString("T=19,00C", 60, 150);
  //tft.drawString("T=20,58C", 60, 200);
 
  tft.drawString("T=10,58C", 175, 150);
 
  tft.drawString("H=70,39%", 175, 170);
 // tft.drawString("T=14,38C", 275, 140);
}

void escribirtemp() {
  char consigStr[8];
   char tempStrAHT[8];
      char humStr[8];
     

 dtostrf(consigna, 1, 2, consigStr);
  dtostrf(temperature_AHT20, 1, 2, tempStrAHT);
      dtostrf(humidity, 1, 2, humStr);
   tft.fillRect(80,30,110,90, TFT_WHITE);
tft.fillRect(80,200,100,50, TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(consigStr,130, 230);
  tft.drawString(tempStrAHT, 130, 50);
  tft.drawString(humStr, 130, 100);
 /* tft.drawString("T=17,58C", 275, 80);
 tft.drawString("T=10,58C", 165, 170);
  tft.drawString("H=70,39%", 165, 200);
  tft.drawString("T=14,38C", 275, 140);*/
}


void comprobarpulsacion(){

// Checks if Touchscreen was touched, and prints X, Y and Pressure (Z) info on the TFT display and Serial Monitor
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 1, 3200, 1, SCREEN_WIDTH);
    y = map(p.y, 1, 2400, 1, SCREEN_HEIGHT);
    z = p.z;

    printTouchToSerial(x, y, z);
       if ((x < 120) && (y <80)) {
       Serial.println("Down button pressed");
        consigna=consigna-0.5;
        escribirtemp();
              delay(200); }
       if ((x > 120) && (x < 240) && (y <80)) {
       Serial.println("Up button pressed");
        consigna=consigna+0.5;
        escribirtemp();
              delay(200); }


    if(buttonState){
      
       if ((x > 240) && (y <80)) {
             Serial.println("OFF");        
          Serial.println("OFF button pressed");
          drawOFFButton();
        delay(200);  }
      }
    
    else {
     
       if ((x > 240) && (y <80)) {
         Serial.println("ON");
          Serial.println("ON button pressed");
          drawONButton();
         delay(200);
        }      }    }}






// Generate IV (once)
void aes_init() {
  aesLib.gen_iv(aes_iv);
  aesLib.set_paddingmode((paddingMode)0);
}

uint16_t encrypt_to_ciphertext(char * msg, uint16_t msgLen, byte iv[]) {
  //Serial.println("Calling encrypt (string)...");
  // aesLib.get_cipher64_length(msgLen);
  int cipherlength = aesLib.encrypt((byte*)msg, msgLen, (byte*)ciphertext, aes_key, sizeof(aes_key), iv);
                   // uint16_t encrypt(byte input[], uint16_t input_length, char * output, byte key[],int bits, byte my_iv[]);
  return cipherlength;
}

uint16_t decrypt_to_cleartext(byte msg[], uint16_t msgLen, byte iv[]) {
 // Serial.print("Calling decrypt...; ");
  uint16_t dec_bytes = aesLib.decrypt(msg, msgLen, (byte*)cleartext, aes_key, sizeof(aes_key), iv);
 // Serial.print("Decrypted bytes: "); Serial.println(dec_bytes);
  return dec_bytes;
}


void BMP280_begin() {
  Wire.beginTransmission(0x77);
  Wire.write(0xD0);  // 0xBE -->  register for chip identification
  Wire.endTransmission();
  Wire.requestFrom(0x77, 1);
  uint8_t chip_ID = Wire.read();
  if (chip_ID == 0x58) {  // 0x58 --> BMP280
    Serial.println("BMP280 found");
  } else {
    Serial.println("BMP280 no encontrado");
  }

  // Generate soft-reset
  Wire.beginTransmission(0x77);
  Wire.write(0xE0);  // 0xE0 --> Reset register
  Wire.write(0xB6);  // 0xB6 --> Reset value for reset register
  Wire.endTransmission();

  // Wait for copy completion NVM data to image registers
  uint8_t stat_Reg = 1;

  while (stat_Reg == 1) {
    Wire.beginTransmission(0x77);
    Wire.write(0XF3);  // 0XF3 --> Status register
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)0x77, (byte)1);
    stat_Reg = Wire.read();
    Serial.println(stat_Reg);
  }

  // See datasheet 4.2.2 Trimming parameter readout


  // Array for storing the read values
  uint16_t values[12];

  // Addresses of the registers with coefficients Data to be read
  uint8_t registers[] = { 0x88, 0x8A, 0x8C, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E };

  for (int i = 0; i < 12; i++) {
    Wire.beginTransmission(0x77);
    Wire.write(registers[i]);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)0x77, (byte)2);
    if (Wire.available() >= 2) {
      values[i] = Wire.read() << 8 | Wire.read();  // compose 16-bit value
    }
  }

  // Reverse the bytes for all 16-bit values (little endian / big endian)
  for (int i = 0; i < 12; i++) {
    values[i] = (values[i] >> 8) | (values[i] << 8);
  }

  // Assigning values to the variables
  _dig_T1 = values[0];
  _dig_T2 = values[1];
  _dig_T3 = values[2];
  _dig_P1 = values[3];
  _dig_P2 = values[4];
  _dig_P3 = values[5];
  _dig_P4 = values[6];
  _dig_P5 = values[7];
  _dig_P6 = values[8];
  _dig_P7 = values[9];
  _dig_P8 = values[10];
  _dig_P9 = values[11];


  // Set in sleep mode to provide write access to the “config” register
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);  // 0XF3 --> Contol register
  Wire.write(0b00);  // 00   --> sleep mode
  Wire.endTransmission();

  
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  uint8_t configValues = ((0b001 << 5) | (0b011 << 2) | 0b11);
  //                       temp           press         mode
  Wire.write(configValues);
  Wire.endTransmission();

  delay(10);

  
  
  Wire.beginTransmission(0x77);
  Wire.write(0xF5);
  configValues = (0b110 << 5) | (0b100 << 2);
  //              standby        filter
  Wire.write(configValues);
  Wire.endTransmission();



  // Wait for first completed conversion
  delay(100);

 
}



void readTemperatureBMP280() {
  int32_t var1, var2, adc_T;

  // Read temperature registers
  Wire.beginTransmission(0x77);
  Wire.write(0xFA);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x77, (byte)3);

  adc_T = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
  adc_T >>= 4;

  // See datasheet 4.2.3 Compensation formulas
  var1 = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

  _t_fine = var1 + var2;

  float T = (((_t_fine * 5) + 128) >> 8);

  temperature_BMP280 = T / 100;
  
}



void readPressureBMP280() {
  int64_t var1;
  int64_t var2;
  int64_t p;
  int32_t adc_P;

  // Read temperature for t_fine
  readTemperatureBMP280();

  // Read pressure registers
  Wire.beginTransmission(0x77);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x77, (byte)3);
  adc_P = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();

  adc_P >>= 4;

  // See datasheet 4.2.3 Compensation formulas
  var1 = ((int64_t)_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_dig_P6;
  var2 = var2 + ((var1 * (int64_t)_dig_P5) << 17);
  var2 = var2 + (((int64_t)_dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_dig_P3) >> 8) + ((var1 * (int64_t)_dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_dig_P1) >> 33;

  if (var1 == 0) {
    return;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_dig_P7) << 4);

  pressure = p / 25600;
}




void AHT20_begin() {
  Wire.beginTransmission(0x38);
  Wire.write(0xBE);  // 0xBE --> init register for AHT2x
  Wire.endTransmission();
}

void startMeasurementAHT20() {
  Wire.beginTransmission(0x38);
  Wire.write(0xAC);  // 0xAC --> start measurement
  Wire.write(0x33);  // 0x33 --> not really documented what it does, but it's called MEASUREMENT_CTRL
  Wire.write(0x00);  // 0x00 --> not really documented what it does, but it's called MEASUREMENT_CTRL_NOP
  Wire.endTransmission();
  measurementDelayAHT20 = millis();
  sensor_started = true;
  sensor_busy = true;
}

void checkbusyAHT20() {
  if (millis() < measurementDelayAHT20) {
    measurementDelayAHT20 = millis();
  }

  if (sensor_started && sensor_busy && ((millis() - measurementDelayAHT20 >= 200))) {
    sensor_started = false;
    sensor_busy = false;
  }


  if (sensor_started && sensor_busy && ((millis() - measurementDelayAHT20 >= 80))) {
    Wire.requestFrom(0x38, 1);
    if (Wire.available()) {
      unsigned char c = Wire.read();
      if (!(c & 0x80)) {
        sensor_busy = false;
      }
    }
  }
}


void getDataAHT20() {
  if (sensor_started && !sensor_busy) {
    Wire.requestFrom(0x38, 7);  // Request 7 bytes of data

    unsigned char str[7] = { 0 };
    int index = 0;

    // Fault detection
    unsigned long timeoutMillis = 200;
    unsigned long startMillis = millis();



    while (Wire.available()) {
      str[index] = Wire.read();  // Receive a byte as character

           index++;

      // Fault detection
      if (millis() - startMillis > timeoutMillis) {
        Serial.println("Timeout while waiting for data from AHT20");
        return;
      }
    }
    if (index == 0 || (str[0] & 0x80)) {
      Serial.println("Failed to get data from AHT20");
      sensor_started = false;
      return;
    }

    // Check CRC
    uint8_t crc = 0xFF;
    for (uint8_t byteIndex = 0; byteIndex < 6; byteIndex++) {
      crc ^= str[byteIndex];
      for (uint8_t bitIndex = 8; bitIndex > 0; --bitIndex) {
        if (crc & 0x80) {
          crc = (crc << 1) ^ 0x31;
        } else {
          crc = (crc << 1);
        }
      }
    }
    if (crc != str[6]) {
      Serial.println("CRC check failed");
      sensor_started = false;
      return;
    }

    // Parse data
    float humi, temp;
    // Extract the raw data for humidity from the bytes
    unsigned long __humi = str[1];  // Byte 1: The first 8 bits of the raw data for humidity
    __humi <<= 8;                   // Move the bits 8 positions to the left
    __humi += str[2];               // Byte 2: Add the next 8 bits
    __humi <<= 4;                   // Move the bits 4 positions to the left
    __humi += str[3] >> 4;          // Byte 3: Add the last 4 bits (shifted to the right)

   
    humi = (float)__humi / 1048576.0;
    humidity = humi * 100.0;
//humidity=humi;
    // Extract the raw data for temperature from the bytes
    unsigned long __temp = str[3] & 0x0f;  // Byte 3: The last 4 bits of the raw data for the temperature
    __temp <<= 8;                          // Move the bits 8 positions to the left
    __temp += str[4];                      // Byte 4: Add the next 8 bits
    __temp <<= 8;                          // Move the bits to the left again by 8 positions
    __temp += str[5];                      // Byte 5: Add the last 8 bits


    
    temp = (float)__temp / 1048576.0 * 200.0 - 50.0;

    temperature_AHT20 = temp;

    sensor_started = false;
  }
}


void leeraht20bmp280() {

  int bandera=10;
while(bandera>0){
  checkbusyAHT20();
  getDataAHT20();
  bandera--;
  if((humidity*temperature_AHT20)>0){bandera=-1;}
  }


  unsigned long currentMillis = millis();
  if (currentMillis - HeartbeatMillis >= Heartbeatinterval) {
    HeartbeatMillis = currentMillis;


    //BMP280
    readTemperatureBMP280();
    Serial.print("Temperatur: ");
    Serial.print(temperature_BMP280);
    Serial.println(" C");

    readPressureBMP280();
    Serial.print("presion: ");
    Serial.print(pressure);
    Serial.println(" hPa");


    // AHT20
    startMeasurementAHT20();

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Temperatur: ");
    Serial.print(temperature_AHT20);
    Serial.println(" C");



    // Calculate the delta between the temperature values AHT20 and BMP280
    delta = (temperature_BMP280 > temperature_AHT20) ? (temperature_BMP280 - temperature_AHT20) : (temperature_AHT20 - temperature_BMP280);

    if (delta < minDelta) {
      minDelta = delta;
    }
    if (delta > maxDelta) {
      maxDelta = delta;
    }
    Serial.print("Temperatur Delta : ");
    Serial.print(delta);
    Serial.print(" | Min Delta: ");
    Serial.print(minDelta);
    Serial.print(" | Max Delta: ");
    Serial.println(maxDelta);
  }
 }





void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}






uint16_t msgLen;
uint16_t encLen;
unsigned char base64decoded[50];
uint16_t decLen;


void AHT20BMP280_begin(){
int bandera=10;

Wire.begin(sda, scl);
  AHT20_begin();
  BMP280_begin();
  startMeasurementAHT20();
  while(bandera>0){
leeraht20bmp280();
delay(100);
//bandera--;
  if((humidity*temperature_AHT20)>0){bandera=-1;}
  }
}



void setup() {
 pinMode(electrovalvula, OUTPUT); // Sets the trigPin as an Output
  pinMode(fan1, OUTPUT); // Sets the trigPin as an Output
  pinMode(fan2, OUTPUT); // Sets the trigPin as an Output
  pinMode(fan3, OUTPUT); // Sets the trigPin as an Output
digitalWrite(fan1, LOW);
digitalWrite(fan2, LOW);
digitalWrite(fan3, HIGH);
digitalWrite(electrovalvula, LOW);
  Serial.print("Arrancamos ");
  Serial.begin(115200);
  setup_wifi();
  //client.setServer(mqtt_server, 1883);
  
  AHT20BMP280_begin();
  
  aes_init(); // generate random IV, should be called only once? causes crash if repeated..
  consigna=15;

// Accurate time is necessary for certificate validation and writing in batches
    // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
    // Syncing progress and the time will be printed to Serial.
    timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
  
  
    // Check server connection
    if (client1.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client1.getServerUrl());
    } else {
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client1.getLastErrorMessage());
    }

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 3: touchscreen.setRotation(3);
  touchscreen.setRotation(3);//antes 1

  // Start the tft display
  tft.init();
  // Set the TFT display rotation in landscape mode
  tft.setRotation(0);//antes 1

  // Clear the screen before writing to it
 
tft.fillScreen(TFT_WHITE);
  // Draw button 
 

  escritorio();
drawOFFButton();

}

void loop() {
  char tempStrAHT[8];
      char humStr[8];
      char tempStrBMP[8];
      char presStr[8];
      
       unsigned long now = millis();
 
   // Enviar datos cada 10 segundos
 
 
 leeraht20bmp280();
 
  comprobarpulsacion();
 

 
 if (now - lastSend > 2000) {
    lastSend = now;
       escribirtemp(); 
     // escritorio();
      dtostrf(temperature_AHT20, 1, 2, tempStrAHT);
      dtostrf(humidity, 1, 2, humStr);
      dtostrf(temperature_BMP280, 1, 2, tempStrBMP);
      dtostrf(pressure, 1, 2, presStr);
      
   
  
point.clearFields();
  
    // Store measured value into point
    point.addField("temperaturaAHT20", tempStrAHT);
   point.addField("temperaturaBMP280", tempStrBMP);
   
  point.addField("humedad", humStr);
   point.addField("presion", presStr);
    
  
     //Print what are we exactly writing
    Serial.print("Writing: ");
    Serial.println(point.toLineProtocol());
  

  
    // Write point
    if (!client1.writePoint(point)) {
      Serial.print("InfluxDB write failed: ");
    Serial.println(client1.getLastErrorMessage());
    }
  
  
 
}


 

}










