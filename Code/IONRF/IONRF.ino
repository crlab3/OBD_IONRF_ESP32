#include <RF24.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// PINS NOW OK!
#define CE_PIN 7
#define CSN_PIN 3
// GPIO pin settings
#define LED_PIN 8
#define XKC_PIN A1
#define RELAY_1 10
#define XKC_ACTIVATE 9
#define DIG_IN_1 0
#define DIG_IN_2 2
#define SEND_INTERVAL 512
#define GPIO_SET_INTERVAL 2048
#define SEND_CYCLES 3

// these define cbi and sbi, for as far they are not known yet
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

const byte ADDR[6] = "00001";
uint8_t LEDSTATE = 0;
RF24 radio(CE_PIN,CSN_PIN);
void startupIONRF();
void setup_watchdog(int ii);
void system_sleep();
void startupMCU();

void setup() 
{
  startupMCU();
}

void loop() 
{
  // Enable XKC-NPN
  digitalWrite(XKC_ACTIVATE,1);
  delay(200);
  uint8_t cycleCount = 0;
  uint8_t i = 0;
  for(i=0;i<3;i++)
  {
    // Send packet 3 times
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
  // disable XKC-NPN
  digitalWrite(XKC_ACTIVATE,0);
  // disable LED
  digitalWrite(LED_PIN, HIGH);
  // send MCU to sleep
  system_sleep();
  startupMCU();
}

void startupIONRF()
{
  // startup NRF24L01 Radio Module
  while(!radio.begin())
  {
    delay(500);
  }
  radio.openWritingPipe(ADDR);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setChannel(100);
  //radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
  radio.setAutoAck(1);
}

void startupMCU()
{
  // Setup input and output pins
  // Startup IONRF
  pinMode(LED_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  pinMode(CE_PIN, OUTPUT);
  pinMode(XKC_PIN, INPUT);
  pinMode(RELAY_1, OUTPUT);
  pinMode(XKC_ACTIVATE, OUTPUT);
  pinMode(DIG_IN_1,INPUT);
  pinMode(DIG_IN_2,INPUT);
  startupIONRF();
  digitalWrite(LED_PIN, LOW);
}

void system_sleep() 
{
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  setup_watchdog(9);                   // approximately 8 seconds sleep
 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sei();                               // Enable the Interrupts so the wdt can wake us up

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

void setup_watchdog(int ii) 
{
  // 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
  // 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

  uint8_t bb;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect)
{
 // nothing here
}