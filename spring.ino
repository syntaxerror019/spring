/* 
* Spring or Well Water Level Monitor
*
* https://www.mileshilliard.com/
*
* Copyright 2025 - Miles Hilliard
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

#define LED 3
#define RELAY 2
#define BUTTON A1
#define HW685 A0

#define CE 9
#define CSN 10

bool DEBUG = 0;

RF24 radio(CE, CSN);

const byte address[6] = "13789";

const int DEBUG_FREQ = 5000;
const int BLINK_DUR = 50;
const int BLINK_FREQ = 1000;
const int N = 16;

unsigned long last_tx, last_blink;

// --- Sleep/WDT globals ---
volatile bool watchdogFired = false;
ISR(WDT_vect) { watchdogFired = true; }

const unsigned long SLEEP_CYCLES = 5400; // 12h / 8s
unsigned long sleepCounter = 0;

void setupWatchdog() {
  MCUSR = 0;
  WDTCSR |= (1 << WDCE) | (1 << WDE);   // enable config
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // 8s, interrupt mode
}

void goToSleep() {
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  // Disable peripherals to save power
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sleep_bod_disable(); // turn off brown-out detector
  sei();
  sleep_cpu();

  // --- wake up here ---
  sleep_disable();

  // Re-enable ADC & peripherals when awake
  power_all_enable();
  ADCSRA |= (1 << ADEN);
}

void setup() {
  Serial.begin(115200);

  Serial.println("Remote Spring Monitor - rev 1");
  Serial.println("https://www.mileshilliard.com");
  Serial.println("Copyright 2025 Miles Hilliard");

  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(HW685, INPUT);

  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);  // enable button functionality (v-gnd)

  // Set unused pins to INPUT_PULLUP to avoid floating
  uint8_t unusedPins[] = {4, 5, 6, 7, 8};
  for (uint8_t i = 0; i < sizeof(unusedPins); i++) {
    pinMode(unusedPins[i], INPUT_PULLUP);
  }

  if (digitalRead(BUTTON) == LOW) {
    DEBUG = 1;
  }

  Serial.print("Radio begin...");
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(5, 15);
  radio.setAutoAck(true);
  radio.stopListening();
  Serial.println("ok");

  if (DEBUG) {
    debug_init();
  }

  setupWatchdog();
}

void loop() {
  if (!DEBUG) {
    // Sleep loop
    if (sleepCounter >= SLEEP_CYCLES) {
      sleepCounter = 0;

      // --- Wake task ---
      digitalWrite(LED, HIGH);
      radio.powerUp();
      int HW685_readings = read_sensor();
      radio.write(&HW685_readings, sizeof(HW685_readings));
      radio.powerDown();
      digitalWrite(LED, LOW);

    }

    watchdogFired = false;
    goToSleep();
    if (watchdogFired) {
      sleepCounter++;
      blink(BLINK_DUR);
    }

  } else {
    debug();
  }
}

void debug() {
  if ((millis() - last_tx) > DEBUG_FREQ) {
    last_tx = millis();

    int HW685_readings = read_sensor();
    radio.write(&HW685_readings, sizeof(HW685_readings));

    Serial.println("sent");
  }
}

void debug_init() {
  Serial.println("DEBUG MODE");
  digitalWrite(LED, HIGH);
}

void blink(int dur) {
  digitalWrite(LED, HIGH);
  delay(dur);
  digitalWrite(LED, LOW);
}

int read_sensor() {
  digitalWrite(RELAY, HIGH);  // turn on sensor
  delay(3000);                // let values stable.

  int accum_tx = 0;
  for (int i = 0; i < N; i++) {
    accum_tx += analogRead(HW685);
    delay(5);
  }

  digitalWrite(RELAY, LOW);
  return accum_tx;
}
