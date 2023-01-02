#include <RF24.h>
#include <SPI.h>

// PINS NOW OK!
#define CE_PIN 7
#define CSN_PIN 3
// GPIO pin settings
#define LED_PIN 8
#define XKC_PIN A1
#define RELAY_1 10
#define RELAY_2 9
#define DIG_IN_1 0
#define DIG_IN_2 2
#define SEND_INTERVAL 512
#define GPIO_SET_INTERVAL 2048


const byte ADDR[6] = "00001";
uint8_t LEDSTATE = 0;
RF24 radio(CE_PIN,CSN_PIN);
void startupIONRF();
char getCoolantLevel();
char getDigitalInputLevels();
void setDigitalOutputLevels(uint8_t outputState);

void setup() 
{
  // Setup input and output pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  pinMode(CE_PIN, OUTPUT);
  pinMode(XKC_PIN, INPUT);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(DIG_IN_1,INPUT);
  pinMode(DIG_IN_2,INPUT);
  
  // Shut-off RED LED
  digitalWrite(LED_PIN, HIGH);
  // First start
  digitalWrite(LED_PIN, LOW);
  startupIONRF();
  digitalWrite(LED_PIN, HIGH);
}

void loop() 
{
  
  // TEST LOOP sending data to ESP32 NRF24L01
  unsigned long cTime = millis();
  if(cTime % SEND_INTERVAL == 0)
  {
    char buf[5]="";
    buf[0] = '?';
    if(digitalRead(DIG_IN_1) == 1)
      buf[1] = 'H';
    else
      buf[1] = 'L';
    if(digitalRead(DIG_IN_2) == 1)
      buf[2] = 'H';
    else
      buf[2] = 'L';
    uint32_t levelResult = analogRead(XKC_PIN);
    if(levelResult<750)
      buf[3] = 'H';
    else
      buf[3] = 'L';
    buf[4] = '$';
    radio.write(&buf, sizeof(buf));
  }
  if(cTime % GPIO_SET_INTERVAL == 0)
  {
    outputState = 
    setDigitalOutputLevels(uint8_t outputState);
  }

}

void startupIONRF()
{
  // startup NRF24L01 Radio Module
  while(!radio.begin())
  {
    delay(500);
  }
  uint8_t err = 0;
  radio.openWritingPipe(ADDR);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setChannel(100);
  //radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
  radio.setAutoAck(1);
}

void setDigitalOutputLevels(uint8_t outputState)
{
  // if state = 0, Relay1 ON, if 1 = Relay2 ON, if 2 = both Relays are activated, else = no relays activated
  switch(outputState)
  {
    case 0:
    {
      digitalWrite(RELAY_1,HIGH);
      digitalWrite(RELAY_2,LOW);
    }break;

    case 1:
    {
      digitalWrite(RELAY_1,LOW);
      digitalWrite(RELAY_2,HIGH);    
    }break;

    case 2:
    {
      digitalWrite(RELAY_1,HIGH);
      digitalWrite(RELAY_2,HIGH); 
    }break;

    default:
    {
      digitalWrite(RELAY_1,LOW);
      digitalWrite(RELAY_2,LOW);
    }break;
  };
}
