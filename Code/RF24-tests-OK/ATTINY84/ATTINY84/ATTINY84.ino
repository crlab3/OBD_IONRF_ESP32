/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 3,500000); // CE, CSN

const byte address[6] = "00001";

void setup() {
  pinMode(8,OUTPUT);
  while(!radio.begin())
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);

  radio.stopListening();
  digitalWrite(8,LOW);
  delay(1000);
  digitalWrite(8,HIGH);
  delay(1000);
  digitalWrite(8,LOW);
  delay(1000);
  digitalWrite(8,HIGH);
  delay(1000);
}

void loop() {
  const char text[] = "setme Up";
  radio.write(&text, sizeof(text));
  digitalWrite(8,LOW);
  delay(200);
  digitalWrite(8,HIGH);
  delay(200);
}