// Mazda 6 2.2d Version without Brightness Control
// NRF24L01 WORKING!!!!
#include "pitches.h"
#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "math.h"
#include <SPI.h>
//#include "printf.h"
#include "RF24.h"
// stuff for removing bonding
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include"esp_gap_bt_api.h"
#include "esp_err.h"
#include "string.h"

#define GR_CORR 0.5

BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
#define DEBUG_PORT Serial

#define TEMP_LOW_L 0  // -40C 
#define TEMP_LOW_U 90 // +50C
#define TEMP_MID_L 91 // +51C
#define TEMP_MID_U 124 // +84C
#define TEMP_OK_L 125 // +85C
#define TEMP_OK_U 150 // +110C

#define RES_PWM 10
#define LED_MAX 1023

#define NRF_CE 21
#define NRF_CS 5

#define OBD_UPDATE_INTERVAL 100
#define COOLANT_LOW_LIMIT 10
#define COOLANT_LEVEL_TIMEOUT 400000
#define ELM_ERROR_MAX 10
#define ELMTIMEOUT 10000
/*GPIOs*/

//LEFT = COOLANT, RIGHT = OIL
#define LEFT_RGB 1
#define RGB_LEFT_RED 32
#define RGB_LEFT_BLUE 33
#define RGB_LEFT_GREEN 13 

#define RIGHT_RGB 2
#define RGB_RIGHT_RED 16
#define RGB_RIGHT_GREEN 17
#define RGB_RIGHT_BLUE 25 

#define DPF_LED 12
#define STATUS1_LED 26
#define CLNTSTATUS_LED 27

#define BUZZER_PIN 4
#define BUZZER_PWM_CHN 10

//bonding removal stuff
#define REMOVE_BONDED_DEVICES 1   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove
#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

ELM327 myELM327;

RF24 radio(21,5); // CE pin at 21, CSN pin at 5

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
uint8_t COOLANT_LOW_CNTR = 0;
uint32_t RADIO_UNAVAILABLE = 0;

int cyclecounter = 0;

void setSingleLEDValue(uint8_t ID, uint16_t value, float brightness);
void setRGBLEDColor(uint8_t ID, uint16_t R, uint16_t G, uint16_t B, float brightness);
void startmyRF24();
void playToneBuzzer(uint8_t num_beeps, uint32_t note);
void printAllValues();

void removeAllBonded();
bool initBluetooth();
char *bda2str(const uint8_t* bda, char *str, size_t size);
uint8_t errorCount = 0;
uint8_t queryFlag = 0;
uint16_t loopCount = 0;
uint32_t prevTime1,prevTime2 = 0;
BTAddress devAddress;

void setup()
{
  // create HeatScale
  int j=0;
  for(j=0;j<255;j++)
  {
    if(j>=TEMP_LOW_L && j<=TEMP_LOW_U) //from -40C to +50C BLUE
    {
      HeatScale[j].B = LED_MAX;
      HeatScale[j].R = 0;
      HeatScale[j].G = 0; 
    }
    if(j>TEMP_LOW_U && j<=TEMP_MID_U) //from +50C to 85C WHITE
    {
      HeatScale[j].B = LED_MAX;
      HeatScale[j].R = LED_MAX;
      HeatScale[j].G = LED_MAX; 
    }
    if(j>TEMP_OK_L && j<=TEMP_OK_U) //from +80C to 110C YELLOW
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = LED_MAX;
      HeatScale[j].G = LED_MAX/2; 
    }
    if(j>TEMP_OK_U && j<255) //above 110C RED
    {
      HeatScale[j].B = 0;
      HeatScale[j].R = LED_MAX;
      HeatScale[j].G = 0; 
    }
  }

  //-------------------SETUP LEDs---------------------
  
  uint8_t LEDPins[9]={RGB_RIGHT_RED,RGB_RIGHT_GREEN,RGB_RIGHT_BLUE,RGB_LEFT_RED,RGB_LEFT_GREEN,RGB_LEFT_BLUE,DPF_LED,STATUS1_LED,CLNTSTATUS_LED};
  int i = 0;
  for(i=0; i<9; i++)
  {
    ledcSetup(i, LED_MAX, RES_PWM);
    ledcAttachPin(LEDPins[i], i);  
    ledcWrite(i, LED_MAX);  //inverted logic!, 1023 = zero light
  }
  
  //-------------------START LEDs PWM & TEST----------------------
  for(i=1;i<255;i++)
  {
    setRGBLEDColor(RIGHT_RGB, HeatScale[i].R, HeatScale[i].G, HeatScale[i].B, 1);
    setRGBLEDColor(LEFT_RGB, HeatScale[i].R, HeatScale[i].G, HeatScale[i].B, 1);
    delay(5);
  }
  setSingleLEDValue(DPF_LED, LED_MAX, 1);
  delay(150);
  setSingleLEDValue(STATUS1_LED, LED_MAX, 1);
  delay(150);
  setSingleLEDValue(CLNTSTATUS_LED, LED_MAX, 1);
  delay(150);
  setSingleLEDValue(DPF_LED, 0, 0);
  delay(150);
  setSingleLEDValue(STATUS1_LED, 0, 0);
  delay(150);
  setSingleLEDValue(CLNTSTATUS_LED, 0, 0);
  delay(150);

  setRGBLEDColor(LEFT_RGB, LED_MAX, 0, 0, 1);
  setRGBLEDColor(RIGHT_RGB, LED_MAX, 0, 0, 1);
  

  //-------------------SETUP LEDs END---------------------
  
  //-------------------Start NRF24L01+ Radio Module---------------------
  DEBUG_PORT.begin(115200);
  //DEBUG_PORT.println("Start my RF24 now...");
  setRGBLEDColor(LEFT_RGB, LED_MAX, LED_MAX, 0, 1);
  startmyRF24();
  //radio.printPrettyDetails();
  //DEBUG_PORT.println("RF24 started up...");
  setRGBLEDColor(LEFT_RGB, 0, LED_MAX, 0, 1);
  setRGBLEDColor(RIGHT_RGB, LED_MAX, LED_MAX, 0, 1);

  //-------------------Finished NRF24L01+ Radio Module Init---------------------
 
  //-------------------Start Bluetooth Connection---------------------
  // 1. Remove all bonded devices.  
  //DEBUG_PORT.println("Removing bonded bluetooth devices...");
  removeAllBonded();
  //DEBUG_PORT.println("Removed bonded bluetooth devices successfully!");

  // 2. Start connection to OBDII device
  //DEBUG_PORT.println("ELM: Scanning begin...");
  ELM_PORT.begin("MyDev", true);
  BTScanResults* btDeviceList = ELM_PORT.getScanResults();
  if (SerialBT.discoverAsync([](BTAdvertisedDevice* pDevice) 
  {
      BTAddress btadr = pDevice->getName();
      const char * devName = btadr.toString().c_str();
      if(strcmp(devName,"OBDII"))
      {
        devAddress = pDevice->getAddress();
      }
  } )
    )
  {
  delay(5000);
  }

  if (!ELM_PORT.connect(devAddress))
  {
    ELM_PORT.setPin("1234");
    if (!ELM_PORT.connect(devAddress))
    {
      //DEBUG_PORT.println("ELM: Couldn't connect to OBD scanner. Rebooting...");
      setRGBLEDColor(RIGHT_RGB,0,0,0,0);
      setRGBLEDColor(LEFT_RGB,0,0,0,0);
      playToneBuzzer(1, NOTE_A6);
      playToneBuzzer(1, NOTE_A5);
      playToneBuzzer(1, NOTE_A4);
      ESP.restart();  //reboot on error
    }
  }
  //DEBUG_PORT.println("ELM: Connected to OBD device successfully!");
  // Connected to OBDII device
  // 3. Begin connection with ELM327 chip via BT Serial
  //DEBUG_PORT.println("ELM: Begin bluetooth serial connection...");
  if (!myELM327.begin(ELM_PORT, false, 2000))
  {
    //DEBUG_PORT.println("ELM: Could not establish serial connection. Rebooting...");
    setRGBLEDColor(RIGHT_RGB,0,0,0,1);
    setRGBLEDColor(LEFT_RGB,0,0,0,1);
    playToneBuzzer(1, NOTE_A6);
    playToneBuzzer(1, NOTE_A5);
    playToneBuzzer(1, NOTE_A4);
    ESP.restart();  //reboot on error
  }
  // Finished connection setup with ELM327 chip via BT Serial.
  //DEBUG_PORT.println("ELM: Serial connection to ELM327 successful.");
  // Switch off LEDs
  setSingleLEDValue(DPF_LED,0,0);
  setSingleLEDValue(STATUS1_LED,LED_MAX,1); // Status 1 LED shows all is OK
  setRGBLEDColor(RIGHT_RGB, 0, LED_MAX, 0, 1);
  playToneBuzzer(1, NOTE_A4);
  playToneBuzzer(1, NOTE_A5);
  playToneBuzzer(1, NOTE_A6);
  setRGBLEDColor(RIGHT_RGB,0,0,0,0);
  setRGBLEDColor(LEFT_RGB,0,0,0,0);
}

void loop()
{
  /*-------------------------START OF MAIN LOOP--------------------------*/

  /* ----------------- UPDATE MILLIS TIMER  -----------------*/
  unsigned long cTime = millis();

  /* ----------------- RADIO MODULE GET DATA -----------------*/
  if(radio.available())
  {
    RADIO_UNAVAILABLE = 0;  // reset radio unavailable counter
    char buffer[10];
    //DEBUG_PORT.println("Received NRF24L01 packet...");
    radio.read(&buffer, sizeof(buffer));
    //DEBUG_PORT.println(buffer);
    if(buffer[0]=='?' && buffer[4]=='$')  // if valid packet is received
    {
      if(buffer[3]=='L')
      {
        MY_COOLANT_LEVEL = 0;
      }
      if(buffer[3]=='H')
      {
        MY_COOLANT_LEVEL = 1;
      }
      // option: do extra operations with digital inputs from the car in buf[1] & buf[2]
    }
  }
  else
  {    
    RADIO_UNAVAILABLE++; // no data was received
  }

/*-------------------------START OF MAIN TIMED LOOP--------------------------*/
  if(cTime - prevTime1 >= OBD_UPDATE_INTERVAL)
  { 
    //DEBUG_PORT.println("=======OBD_UPDATE_INTERVAL elapsed, cycle run start.========");
    prevTime1 = cTime;
    // check for the amount of NOT received radio cycles
    if(RADIO_UNAVAILABLE>COOLANT_LEVEL_TIMEOUT) // if there were too many NOT received cycles
    {
      RADIO_UNAVAILABLE = 0;    // reset radio unavailable counter
      MY_COOLANT_LEVEL = 2;     // coolant state is unknown
    }
    // cycle through OBD and sensor queries
    switch (queryFlag) 
    {
      case 0: 
        MY_ENGINE_OIL_TEMP = (uint32_t)myELM327.oilTemp();
        //DEBUG_PORT.println("queried engine oil.");
        break;
      case 1:
        MY_ENGINE_COOLANT_TEMP = (uint32_t)myELM327.engineCoolantTemp();
        //DEBUG_PORT.println("queried engine coolant.");
        break;
      case 2:
        {
          switch(MY_COOLANT_LEVEL)
          {
            case 0:
            {
              //DEBUG_PORT.println("Coolant level: LOW!");
              COOLANT_LOW_CNTR++; // coolant level low indicated, counter increment!
              setSingleLEDValue(CLNTSTATUS_LED,0,0); // inverted logic
              playToneBuzzer(1, NOTE_F5);
              if(COOLANT_LOW_CNTR>COOLANT_LOW_LIMIT)
              {
                uint16_t i = 0;
                for(i=0;i<5;i++)
                {
                  setRGBLEDColor(RIGHT_RGB, 0,0,0,0);
                  setRGBLEDColor(LEFT_RGB, 0,0,0,0);
                  playToneBuzzer(5, NOTE_F5);
                  setRGBLEDColor(RIGHT_RGB,LED_MAX,0,0,1);
                  setRGBLEDColor(LEFT_RGB,LED_MAX,0,0,1);
                  playToneBuzzer(5, NOTE_F5);
                }
                COOLANT_LOW_CNTR = 0; //Reset counter.
              }
            }
            break;
            case 1:
            {
              //DEBUG_PORT.println("Coolant level: OK.");
              setSingleLEDValue(CLNTSTATUS_LED,0,0); // inverted logic
              COOLANT_LOW_CNTR = 0; // reset coolant low indicator
            }
            break;
            case 2:
            {
              //DEBUG_PORT.println("Coolant level unknown...");
              setSingleLEDValue(CLNTSTATUS_LED,LED_MAX,1); // inverted logic
            }
            break;
          }
        }
        break;
      case 3:
        MY_REGEN_STATE = (uint32_t)myELM327.processPID(0x22,0x0380,1,1,1,0);
        //DEBUG_PORT.println("queried regen state.");
        break;
      default:
        break;
    }

    
    if(myELM327.nb_rx_state == ELM_SUCCESS)
    {
      queryFlag++;
      queryFlag%=4;
      //DEBUG_PORT.println("========ELM SUCCESS========");
      errorCount = 0;                           // errors count is zeroed out, when ELM_SUCCESS occurs
      // update all LEDs!
      if(queryFlag == 0)
      { //update only when all values are found
        setSingleLEDValue(DPF_LED,0,0);
        setSingleLEDValue(STATUS1_LED,LED_MAX,1);  // show status LED blink, when received value
        //printAllValues();                         // print out all read values to DEBUG_PORT (Serial)
        setRGBLEDColor(RIGHT_RGB,HeatScale[MY_ENGINE_OIL_TEMP+40].R,HeatScale[MY_ENGINE_OIL_TEMP+40].G,HeatScale[MY_ENGINE_OIL_TEMP+40].B,1);
        setRGBLEDColor(LEFT_RGB,HeatScale[MY_ENGINE_COOLANT_TEMP+40].R,HeatScale[MY_ENGINE_COOLANT_TEMP+40].G,HeatScale[MY_ENGINE_COOLANT_TEMP+40].B,1);
        if(MY_REGEN_STATE != 0)                   // alert with LED diesel particle filter (DPF) regeneration state
          setSingleLEDValue(DPF_LED,LED_MAX,1);       
        else
          setSingleLEDValue(DPF_LED,0,0);
        setSingleLEDValue(STATUS1_LED,0,0);        // show status LED blink, when received value  
      }

    }
    else if(myELM327.nb_rx_state != ELM_GETTING_MSG)  // alert with blank STATUS LED for bad received data
      {
        setSingleLEDValue(STATUS1_LED,0,0);
        //DEBUG_PORT.println("ELM ERROR OCCURRED");
        //myELM327.printError();
        errorCount++;                              // increase number of errors occured
      }
    
  }
/*-------------------------END OF MAIN TIMED LOOP--------------------------*/

/*-------------------------ERROR HANDLING START--------------------------*/
  if(errorCount>ELM_ERROR_MAX)
  {
    setRGBLEDColor(RIGHT_RGB,LED_MAX,0,LED_MAX,1);
    setRGBLEDColor(LEFT_RGB,LED_MAX,0,LED_MAX,1);
    playToneBuzzer(1, NOTE_A6);
    playToneBuzzer(1, NOTE_A5);
    playToneBuzzer(1, NOTE_A4);
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
  if(brightness>1)
    brightness = 1;
  if(brightness<0)
    brightness = 0;
  value = value*brightness;
  if(value>=0)
    {
    switch (ID)
    {
    case DPF_LED:
      ledcWrite(6,(LED_MAX-value)*(1-brightness));
      break;

    case STATUS1_LED:
      ledcWrite(7,(LED_MAX-value)*(1-brightness));
      break;

    case CLNTSTATUS_LED:
      ledcWrite(8,(LED_MAX-value)*(1-brightness));
      break;
    
    default:
      break;
    }
  }
}

void removeAllBonded()
{ 
  initBluetooth();
  //DEBUG_PORT.print("ESP32 bluetooth address: ");
  //DEBUG_PORT.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if(!count) {
    //DEBUG_PORT.print("No bonded device found.");
  } else {
    //DEBUG_PORT.print("Bonded device count: "); DEBUG_PORT.println(count);
    if(PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES; 
      //DEBUG_PORT.print("Reset bonded device count: "); DEBUG_PORT.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if(ESP_OK == tError) {
      for(int i = 0; i < count; i++) {
        //DEBUG_PORT.print("Found bonded device # "); DEBUG_PORT.print(i); DEBUG_PORT.print(" -> ");
        //DEBUG_PORT.print(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
        if(REMOVE_BONDED_DEVICES) {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if(ESP_OK == tError) {
            //DEBUG_PORT.print("Removed bonded device # "); 
          } else {
            //DEBUG_PORT.print("Failed to remove bonded device # ");
          }
          //DEBUG_PORT.println(i);
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
  const byte RF24_ADDR[6] = {"00001"};
  while(!radio.begin()) 
    {
      //DEBUG_PORT.println(F("NRF24L01 hardware is not responding, retrying..."));
      delay(1000);
    }
  int result = 1;
  radio.openReadingPipe(0, RF24_ADDR);
  radio.setPALevel(RF24_PA_MIN);
  //radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  //DEBUG_PORT.println("NRF24L01 hardware is listening!");
}

void playToneBuzzer(uint8_t num_beeps, uint32_t note)
{
  int i=0;
  ledcSetup(BUZZER_PWM_CHN, note, RES_PWM);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHN);
  ledcWrite(BUZZER_PWM_CHN, 0);
  for(i=0;i<num_beeps;i++)
  {
    ledcWrite(BUZZER_PWM_CHN, 512);
    delay(100);
    ledcWrite(BUZZER_PWM_CHN, 0);
    delay(100);
  }
  ledcWrite(BUZZER_PWM_CHN, 0);
}

void printAllValues()
{
  DEBUG_PORT.print("Oil temp: "); 
  DEBUG_PORT.println(MY_ENGINE_OIL_TEMP);
  DEBUG_PORT.print("Coolant temp: "); 
  DEBUG_PORT.println((uint32_t)MY_ENGINE_COOLANT_TEMP);
  DEBUG_PORT.print("Regeneration State: "); 
  DEBUG_PORT.println(MY_REGEN_STATE); 
  DEBUG_PORT.print("Coolant Level State: "); 
  switch(MY_COOLANT_LEVEL)
  {
    case 0: 
      DEBUG_PORT.println("LOW");
      break;
    case 1:
      DEBUG_PORT.println("OK");
      break;
    case 2:
      DEBUG_PORT.println("UNKNOWN");
      break;
    default:
      DEBUG_PORT.println("ERROR");
      break;
  }
}