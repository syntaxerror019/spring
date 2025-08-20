/* 
* Spring / Well Water Level Monitor
* https://www.mileshilliard.com/
* Copywright 2025 - Miles Hilliard
* 
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define LED 3
#define RELAY 2
#define BUTTON A1
#define HW685 A0

#define CE 9
#define CSN 10

bool DEBUG = 0;

RF24 radio(CE, CSN);

const byte address[6] = "13789";

void setup() {
  Serial.begin(115200);

  Serial.println("Remote Spring Monitor - rev 1");
  Serial.println("https://www.mileshilliard.com");
  Serial.println("Copyright 2025 Miles Hilliard");

  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(HW685, INPUT);

  digitalWrite(A3, LOW);  // enable button functionality (v-gnd)

  if (digitalRead(BUTTON) == LOW) {
    DEBUG = 1;
  }

  Serial.print("Radio begin...");

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Serial.println("ok");

  if (DEBUG) {
    debug_init();
  }
}

void loop() {
  if (!DEBUG) {

  } else {
    debug();
  }
}

void debug() {
  Serial.println("Debug Mode");
  delay(500);
}
void debug_init() {
  Serial.println("DEBUG MODE");
  digitalWrite(LED, 1);
}
