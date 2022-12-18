/*
Core: https://github.com/SpenceKonde/ATTinyCore

ATTiny84, 1MHz, Counter-clockwise pin configuration

                        Ard                     Ard
                        Pin Func  +-\/-+   Func Pin      RF24
 pin2 nRF24L01  VCC ---      VCC 1|o   |14 GND       --- nRF24L01 GND  pin1
                         0   PB0 2|    |13 AREF 10
                         1   PB1 3|    |12 PA1  9
                        RST  PB3 4|    |11 PA2  8    
                         2   PB2 5|    |10 PA3  7    --- nRF24L01 CE  pin4
pin 3 nRF24L01 CSN-----  3   PA7 6|    |9  PA4  6    --- nRF24L01 SCK  pin5
pin 7 nRF24L01 MISO ---  4   PA6 7|    |8  PA5  5    --- nRF24L01 MOSI pin6
                                  +----+
*/

#include <RF24.h>

// CE and CSN are configurable, specified values for ATtiny84 as connected above, FIXED!
#define CE_PIN 7
#define CSN_PIN 3
// GPIO pin settings
#define LED_PIN 8
#define XKC_PIN A1
#define RELAY_1 10
#define RELAY_2 9
#define DIG_IN_1 0
#define DIG_IN_2 2

const byte ADDR[6] = "00001";

RF24 radio(CE_PIN,CSN_PIN);
void startupIONRF();
uint8_t getCoolantLevel();
uint8_t* getDigitalInputLevels();
void setDigitalOutputLevels(uint8_t outputState);
void getAndTransmitThruRF24();

void setup() 
{
  // Setup input and output pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(XKC_PIN, INPUT);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(DIG_IN_1,INPUT);
  pinMode(DIG_IN_2,INPUT);
  
  // Shut-off RED LED
  digitalWrite(LED_PIN, HIGH);

  // First start
  startupIONRF();
  getAndTransmitThruRF24();
  radio.powerDown();
  delay(3000);
  radio.powerUp();
}

void loop() 
{
  startupIONRF();
  getAndTransmitThruRF24();
  radio.powerDown();
  delay(3000);
  radio.powerUp();
}

void startupIONRF()
{
  // startup NRF24L01 Radio Module
  while(!radio.begin());  //stuck while radio is not available
  radio.openWritingPipe(ADDR);
  radio.setPALevel(RF24_PA_LOW);
  //radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
  // Light up LED for 1 sec
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
}

uint8_t getCoolantLevel()
{
  // should check negated logic! ???
  int levelResult = analogRead(XKC_PIN);
  if(levelResult<750)
    return 1;
  else
    return 0;
}

uint8_t* getDigitalInputLevels()
{
  uint8_t inputs[2] = {0,0};
  inputs[0] = digitalRead(DIG_IN_1);
  inputs[1] = digitalRead(DIG_IN_2);
  return &inputs[0];
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

void getAndTransmitThruRF24()
{
  uint8_t state[5] = {0,0,2,0,0}; // loaded defaults
  uint8_t *temp = NULL;
  int i=0;
  // state will contain digital inputs state, coolant level state
  // DIG_IN_1&2:00,01,11,10
  // COOLANT_LEVEL: 0,1,2
  temp = getDigitalInputLevels();
  for(i=0;i<2;i++)
    state[i] = temp[i];
  state[2] = getCoolantLevel();
  state[3] = 'F';
  state[4] = 'F';
  while(!radio.write(&state, sizeof(state)));
}