// Mazda 6 2.2d Version without Brightness Control
#include "pitches.h"
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "Wire.h"
#include "math.h"
#include <BH1750.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
// stuff for removing bonding
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"

#define GR_CORR 0.3

BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial

#define RES_PWM 10
#define LED_MAX 1023

/*GPIOs*/

//LEFT ID = 2
#define LEFT_RGB 2
#define RGB_LEFT_RED 16
#define RGB_LEFT_BLUE 4
#define RGB_LEFT_GREEN 17

#define RIGHT_RGB 1
#define RGB_RIGHT_RED 32
#define RGB_RIGHT_GREEN 13
#define RGB_RIGHT_BLUE 33

#define DPF_LED 12
#define STATUS_LED 26
#define OIL_LED 27

#define BUZZER_PIN 25

//bonding removal stuff
#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove
#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

ELM327 myELM327;

RF24 radio(15,5); //15 pin not connected to CE, CE is always ON, IO5 is for CSN

struct RGBColor
{
  int R;
  int G;
  int B;
};

RGBColor HeatScale[255];

uint32_t MY_ENGINE_RPM = 0;
uint32_t MY_ENGINE_OIL_TEMP = 0;
uint32_t MY_ENGINE_COOLANT_TEMP = 0;
uint32_t MY_REGEN_COUNT = 0;
uint32_t MY_REGEN_STATE = 0;
uint8_t MY_COOLANT_LEVEL = 2; // 0 - level Low, 1 - Level OK, 2 - Level Unknown

int cyclecounter = 0;

void setSingleLEDValue(uint8_t ID, uint16_t value, float brightness);
void setRGBLEDColor(uint8_t ID, uint16_t R, uint16_t G, uint16_t B, float brightness);
void startmyRF24();
uint8_t getCoolantLevel();
void playToneBuzzer(uint8_t num_beeps, uint32_t note);
void printAllValues();

void removeAllBonded();
bool initBluetooth();
char *bda2str(const uint8_t* bda, char *str, size_t size);
uint8_t errorCount = 0;
uint8_t queryFlag = 0;
uint16_t loopCount = 0;

void setup()
{

  // create heatscale
  int j=0;
  for(j=0;j<255;j++)
  {
    if(j>=0 && j<=90) //from -40C to +50C BLUE
    {
      HeatScale[j].B = 1000;
      HeatScale[j].R = 0;
      HeatScale[j].G = 0; 
    }
    if(j>90 && j<=120) //from +50C to 80C YELLOW
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = 1000;
      HeatScale[j].G = 1000; 
    }
    if(j>120 && j<=145) //from +80C to 105C RED-ORANGE
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = 1000;
      HeatScale[j].G = 500; 
    }
    if(j>145 && j<255) //above 105C RED
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = 1000;
      HeatScale[j].G = 0; 
    }
  }

  //-------------------SETUP LEDs---------------------
  
  uint8_t LEDPins[9]={RGB_RIGHT_RED,RGB_RIGHT_GREEN,RGB_RIGHT_BLUE,RGB_LEFT_RED,RGB_LEFT_GREEN,RGB_LEFT_BLUE,DPF_LED,STATUS_LED,OIL_LED};
  int i = 0;
  for(i=0; i<9; i++)
  {
    ledcSetup(i, 1000, RES_PWM);
    ledcAttachPin(LEDPins[i], i);  
    ledcWrite(i, LED_MAX);
  }
  //-------------------START LEDs blinking----------------------
  for(i=0;i<1020;i++)
  {
    setRGBLEDColor(LEFT_RGB, i, i, i, 1);
    setRGBLEDColor(RIGHT_RGB, i, i, i, 1);
    delay(5);
  }
  setSingleLEDValue(DPF_LED, LED_MAX, 1);
  delay(150);
  setSingleLEDValue(STATUS_LED, LED_MAX, 1);
  delay(150);
  setSingleLEDValue(OIL_LED, LED_MAX, 1);
  delay(150);
  setSingleLEDValue(DPF_LED, 0, 0);
  delay(150);
  setSingleLEDValue(STATUS_LED, 0, 0);
  delay(150);
  setSingleLEDValue(OIL_LED, 0, 0);
  delay(150);

  setRGBLEDColor(LEFT_RGB, 0, 0, 0, 0);
  setRGBLEDColor(RIGHT_RGB, 0, 0, 0, 0);

  //-------------------SETUP LEDs END---------------------
  
  //-------------------Start NRF24L01+ Radio Module---------------------
  DEBUG_PORT.begin(115200);
  DEBUG_PORT.println("Start my RF24 now...");
  startmyRF24();
  DEBUG_PORT.println("RF24 started up...");
  //-------------------Finished starting up NRF24L01+ Radio Module---------------------

  //-------------------Start Bluetooth Connection---------------------
  // 1. Remove all bonded devices.  
  removeAllBonded();

  // 2. Start connection to OBDII device
  ELM_PORT.begin("OBDII", true);
  
  // 2.1 If needed set PIN code
  // ELM_PORT.setPin("0000");  
  if (!ELM_PORT.connect("OBDII"))
  {
    DEBUG_PORT.println("Couldn't connect to OBD scanner - OBD Connection Failed!");
    ESP.restart();  //reboot on error
  }
  // Connected to OBDII device
  // 3. Begin connection with ELM327 chip via BT Serial
  if (!myELM327.begin(ELM_PORT, false, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner - ELM327 Connection Failed!");
    ESP.restart();  //reboot on error
  }
  // Finished connection setup with ELM327 chip via BT Serial.
  Serial.println("Connected to ELM327");
  // Switch off LEDs
  setRGBLEDColor(RIGHT_RGB,0,0,0,0);
  setRGBLEDColor(LEFT_RGB,0,0,0,0);
  setSingleLEDValue(DPF_LED,0,0);
  setSingleLEDValue(STATUS_LED,LED_MAX,1);
  playToneBuzzer(3, NOTE_C3);
}


void loop()
{
  /*-------------------------START OF MAIN LOOP--------------------------*/
  loopCount++;
  loopCount%=25500;
  // Turn off status LED, until received good OBD data.
  setSingleLEDValue(STATUS_LED,0,0);

  if(loopCount%20000 == 0)
  {
    switch (queryFlag)
    {
    case 0: 
      MY_ENGINE_OIL_TEMP = myELM327.oilTemp();
      /* code */
      break;
    case 1:
      MY_ENGINE_COOLANT_TEMP = myELM327.engineCoolantTemp();
      break;
    case 2:
      MY_COOLANT_LEVEL = getCoolantLevel();
      //MY_ENGINE_RPM = myELM327.rpm();
      break;
    case 3:
      MY_REGEN_STATE = myELM327.processPID(0x22,0x0380,1,1,1,0);
      //MY_REGEN_COUNT = myELM327.processPID(0x22,0x0432,1,2,1,0);
      break;
    case 4:
      // will not be run while queryFlag%=4!
      //MY_REGEN_STATE = myELM327.processPID(0x22,0x0380,1,1,1,0);
	  break;
    default:
      break;
    }
    //  Always alert for low coolant!
    while(getCoolantLevel() == 0)
    {
      // stuck in this loop while coolant is LOW!
      playToneBuzzer(10, NOTE_G6);
      // later on add LED alerts
    }
    noTone(BUZZER_PIN);

    if(myELM327.nb_rx_state == ELM_SUCCESS)
    {
      setSingleLEDValue(STATUS_LED,LED_MAX,1);  // show status LED blink, when received value
      setSingleLEDValue(DPF_LED,0,0);           // switch off DPF LED
      queryFlag++;                              
      queryFlag%=4;                             // query increment & modulo division for switch between values to get
      errorCount = 0;                           // errors count is zeroed out, when ELM_SUCCESS occurs
      printAllValues();                         // print out all read values to DEBUG_PORT (Serial)
      setRGBLEDColor(LEFT_RGB,HeatScale[MY_ENGINE_OIL_TEMP+40].R,HeatScale[MY_ENGINE_OIL_TEMP+40].G,HeatScale[MY_ENGINE_OIL_TEMP+40].B,1);
      setRGBLEDColor(RIGHT_RGB,HeatScale[MY_ENGINE_COOLANT_TEMP+40].R,HeatScale[MY_ENGINE_COOLANT_TEMP+40].G,HeatScale[MY_ENGINE_COOLANT_TEMP+40].B,1);
      setSingleLEDValue(STATUS_LED,0,0);        // show status LED blink, when received value
      
      if(MY_REGEN_STATE != 0)                   // alert with LED diesel particle filter (DPF) regeneration state
        setSingleLEDValue(DPF_LED,LED_MAX,1);       
      else
        setSingleLEDValue(DPF_LED,0,0);
        
    }
    else if(myELM327.nb_rx_state != ELM_GETTING_MSG)  // alert with blank STATUS LED for bad received data
    {
      setSingleLEDValue(STATUS_LED,0,0);
      Serial.println("ELM ERROR OCCURRED");
      myELM327.printError();
      errorCount++;                              // increase number of errors occured
    }

  }

  //If too much errors occur without a single good received data.
  if(errorCount>40)
  {
    ESP.restart();
  }
  /*-------------------------END OF MAIN LOOP--------------------------*/
}

void setRGBLEDColor(uint8_t ID, uint16_t R, uint16_t G, uint16_t B, float brightness)
{
  if(brightness>1)
    brightness = 1;
  if(brightness<0)
    brightness = 0;

  if(ID == RIGHT_RGB)
  {
    //RIGHT LED (ENGINE OIL SELECTED)
    ledcWrite(0, (LED_MAX-R*brightness));
    ledcWrite(1, (LED_MAX-G*brightness*GR_CORR));
    ledcWrite(2, (LED_MAX-B*brightness));
  }
  if(ID == LEFT_RGB)
  {
    //LEFT LED (COOLANT SELECTED)
    ledcWrite(3, (LED_MAX-R*brightness));
    ledcWrite(4, (LED_MAX-G*brightness*GR_CORR));
    ledcWrite(5, (LED_MAX-B*brightness));
  }
}

void setSingleLEDValue(uint8_t ID, uint16_t value, float brightness)
{
  value = value*brightness;
  if(value>=0)
    {
    switch (ID)
    {
    case DPF_LED:
      ledcWrite(6,(LED_MAX-value)*(1-brightness));
      break;

    case STATUS_LED:
      ledcWrite(7,(LED_MAX-value)*(1-brightness));
      break;

    case OIL_LED:
      ledcWrite(8,(LED_MAX-value)*(1-brightness));
      break;
    
    default:
      break;
    }
  }
}

float getBackgroundLightLevel()
{
  /*
  while (!lightMeter.measurementReady(true)) 
  {
    yield();
  }
  float lux = lightMeter.readLightLevel();
  lux = lux/54700;
  lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
  return lux;
  */
}

void removeAllBonded()
{
  
  DEBUG_PORT.begin(115200);
  initBluetooth();
  DEBUG_PORT.print("ESP32 bluetooth address: ");
  DEBUG_PORT.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) {
    DEBUG_PORT.print("No bonded device found.");
  } else {
    DEBUG_PORT.print("Bonded device count: "); DEBUG_PORT.println(count);
    if(PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES; 
      DEBUG_PORT.print("Reset bonded device count: "); DEBUG_PORT.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) {
      for(int i = 0; i < count; i++) {
        DEBUG_PORT.print("Found bonded device # "); DEBUG_PORT.print(i); DEBUG_PORT.print(" -> ");
        DEBUG_PORT.print(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(REMOVE_BONDED_DEVICES) {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) {
            DEBUG_PORT.print("Removed bonded device # "); 
          } else {
            DEBUG_PORT.print("Failed to remove bonded device # ");
          }
          DEBUG_PORT.println(i);
        }
      }        
    }
  }
}

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

bool initBluetooth()
{
  if(!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if(esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if(esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

void startmyRF24()
{
uint8_t RF24_ADDR[6] = { "00001"};
bool radionumber = 1; // this radio will use address[1] to transmit
bool role = false; // this is a radio receiver
float payload = 0.0;
while(!radio.begin()) 
  {
    DEBUG_PORT.println(F("NRF24L01 hardware is not responding, retrying..."));
    delay(1000);
  }
radio.openReadingPipe(0, RF24_ADDR);
radio.setPALevel(RF24_PA_LOW);
radio.startListening();
DEBUG_PORT.println("NRF24L01 hardware is listening!");
}

void playToneBuzzer(uint8_t num_beeps, uint32_t note)
{
  int i=0;
  for(i=0;i<num_beeps;i++)
  {
    tone(BUZZER_PIN, note, 100);
    noTone(BUZZER_PIN);
    delay(100);
  }
}

void printAllValues()
{
  DEBUG_PORT.print("Oil temp: "); 
  DEBUG_PORT.println(MY_ENGINE_OIL_TEMP);
  DEBUG_PORT.print("Coolant temp: "); 
  DEBUG_PORT.println(MY_ENGINE_COOLANT_TEMP);
  /*
  Serial.print("RPM: "); 
  Serial.println(MY_ENGINE_RPM); 
  Serial.print("Regeneration count: "); 
  Serial.println(MY_REGEN_COUNT); 
  */
  DEBUG_PORT.print("Regeneration State: "); 
  DEBUG_PORT.println(MY_REGEN_STATE); 
  DEBUG_PORT.print("Coolant Level State: "); 
  switch(MY_COOLANT_LEVEL)
  {
    case 0: 
      DEBUG_PORT.println("Coolant level: LOW");
      break;
    case 1:
      DEBUG_PORT.println("Coolant level: OK");
      break;
    case 2:
      DEBUG_PORT.println("Coolant level: UNKNOWN");
      break;
    default:
      DEBUG_PORT.println("Coolant level: ERROR");
      break;
  }
}

uint8_t getCoolantLevel()
{
  DEBUG_PORT.println("Starting to get coolant level...");
  while (!radio.available()); //stuck while waiting for packet
  DEBUG_PORT.println("Received NRF24L01 packet...");
  uint8_t text = 2;  // first set it as unknown
  radio.read(&text, sizeof(text));
  DEBUG_PORT.print("Coolant Level Received: ");
  DEBUG_PORT.println(text);
  return text;
}