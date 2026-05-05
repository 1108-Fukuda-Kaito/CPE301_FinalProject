#include <avr/io.h>
#include <avr/interrupt.h>
#include "DHT.h"
#include <LiquidCrystal.h>

// LCD Digital Pins
const int RS = 52, EN = 53, D4 = 44, D5 = 45, D6 = 47, D7 = 46;

// UART 
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// Timer (delay)
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// Bit Definitions
#define TBE 0x20  // Transmit Buffer Empty

#define PIN_RESET      4  // PB4 (Digital 10)
#define PIN_OFF        5  // PB5 (Digital 11)
#define PIN_ON         6  // PB6 (Digital 12)

#define LED_IDLE         6  // PH6 (Digital 9)
#define LED_DISABLED         5  // PH5 (Digital 8)
#define LED_RUNNING        4  // PH4 (Digital 7)
#define LED_ERROR         3  // PH3 (Digital 6)

#define DHTPIN         2  // Digital Pin 2
#define DHTTYPE        DHT11

enum State {
  DISABLED, // System off, waiting for Start button
  IDLE,     // System on, temp is low, fan off
  RUNNING,  // System on, temp is high, fan on
  ERROR     // Sensor unplugged or failed
};

// Default: DISABLED
volatile State currentState = DISABLED;
// bool offButtonLastState = false;
volatile unsigned char lastPinB = 0xFF;
volatile bool requestIDLE = false;
volatile bool requestDISABLED = false;

const float TEMP_THRESHOLD = 25.0;

DHT dht(DHTPIN, DHTTYPE); // Initialize temperature and humidity sensor
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); // Initialize LCD screen

unsigned long lastDHTRead = 0; // For non-blocking timing


void setup() {
  U0init(9600);
  dht.begin();
  init_hardware();
  sei(); 
  lcd.begin(16, 2); // set up number of columns and rows
  transitionTo(DISABLED);
}

void loop() {
  if (requestIDLE) {
    requestIDLE = false;
    transitionTo(IDLE);
  }
  if (requestDISABLED) {
    requestDISABLED = false;
    transitionTo(DISABLED);
  }

  // 2. State-Specific Logic
  switch (currentState) {
    case DISABLED:
      // In DISABLED, we wait for the Start Interrupt to flip us to IDLE
      break;

    case IDLE:
      // monitor_environment();
      // Transition to RUNNING if temp is too high
      temp_humid_sensor();
      break;

    case RUNNING:
      // monitor_environment();
      // control_actuator(true); // Turn on fan
      // Transition to IDLE if temp drops
      temp_humid_sensor();
      break;

    case ERROR:
      // Wait for Reset button to go back to DISABLED or IDLE
      break;
  }
}

ISR(PCINT0_vect) {
  unsigned char currentB = PINB;

  if (!(currentB & (1 << PIN_ON)) && (lastPinB & (1 << PIN_ON)) && (currentState == DISABLED)) {
    requestIDLE = true;
  }
  if (!(currentB & (1 << PIN_OFF)) && (lastPinB & (1 << PIN_OFF)) && (currentState != DISABLED)) {
    requestDISABLED = true;
  }
  if (!(currentB & (1 << PIN_RESET)) && (lastPinB & (1 << PIN_RESET)) && (currentState == ERROR)) {
    requestIDLE = true;
  }

  lastPinB = currentB;
}

void init_hardware(void) {
    // Buttons as Input with Pull-up
    DDRB &= ~((1 << PIN_RESET) | (1 << PIN_OFF) | (1 << PIN_ON));
    PORTB |= ((1 << PIN_RESET) | (1 << PIN_OFF) | (1 << PIN_ON));

    // LEDs as Output
    DDRH |= (1 << LED_IDLE) | (1 << LED_DISABLED) | (1 << LED_RUNNING) | (1 << LED_ERROR);
    
    // Initial state: OFF
    // set_system_power(false);

    // Pin Change Interrupts
    PCICR  |= (1 << PCIE0);   
    PCMSK0 |= (1 << PCINT6) | (1 << PCINT5) | (1 << PCINT4); // ON, OFF, RESET

}

// --- POINTER BASED UART FUNCTIONS ---

void U0init(unsigned long U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / U0baud - 1);

  *myUCSR0A = 0x20; 
  *myUCSR0B = 0x18; // Enable TX and RX
  *myUCSR0C = 0x06; // 8-bit data format
  *myUBRR0  = tbaud;
}

void U0putchar(unsigned char U0pdata) {
  while (!(*myUCSR0A & TBE)); // Wait for empty transmit buffer
  *myUDR0 = U0pdata;
}

void U0print_string(const char* str) {
  while (*str) {
    U0putchar(*str++);
  }
}

void temp_humid_sensor() {
  unsigned long currentMillis = millis();
    
  if (currentMillis - lastDHTRead >= 2000) {
    lastDHTRead = currentMillis;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // ERROR STATE: Transition if sensor fails (10 pts for Sensor Integration)
    if (isnan(h) || isnan(t)) {
      if (currentState != ERROR) {
        transitionTo(ERROR);
      }
      return; 
    }

    // Update Display (10 pts for Display Output)
    update_lcd(t, h);

    // LOGIC: Transition between IDLE and RUNNING
    if (t > TEMP_THRESHOLD) {
      if (currentState == IDLE) {
        transitionTo(RUNNING);
      }
    } else {
      if (currentState == RUNNING) {
        transitionTo(IDLE);
      }
    }
  }
}

void my_delay_ms(unsigned int ms)
{
  // 1. Calculate ticks needed for the requested milliseconds
  // Formula: Ticks = (Time in seconds) / (Clock Period * Prescaler)
  // At 16MHz and 1024 Prescaler, each tick = 0.000064 seconds (64 microseconds)
  // Ticks = (ms / 1000) / 0.000064  =>  Ticks = ms * 15.625
  
  unsigned int ticks = (unsigned int)(ms * 15.625);

  // 2. Stop the timer and clear control registers
  *myTCCR1B &= 0xF8; 
  *myTCCR1A = 0x0;

  // 3. Set the counts (65536 is the rollover point)
  // Note: This works for 'ms' values up to roughly 4000ms (4 seconds)
  *myTCNT1 = (unsigned int)(65536 - ticks);

  // 4. Clear the overflow flag (writing a 1 to TIFR1 clears it)
  *myTIFR1 |= 0x01;

  // 5. Start the timer with 1024 Prescaler (CS12=1, CS11=0, CS10=1)
  *myTCCR1B |= 0b00000101;

  // 6. Wait for the overflow flag (TOV1) to be set
  while((*myTIFR1 & 0x01) == 0); 

  // 7. Stop the timer
  *myTCCR1B &= 0xF8;
}

void transitionTo(State newState) {
  currentState = newState;

  switch (newState) {
    case DISABLED:
      set_leds(0, 1, 0, 0); // Example: Only Red/Yellow LED on
      U0print_string("STATE: DISABLED\n");
      lcd.display();
      lcd.clear();
      lcd.print("System Disabled");
      break;

    case IDLE:
      set_leds(1, 0, 0, 0); // Green LED
      U0print_string("STATE: IDLE\n");
      lcd.display();
      lcd.clear();
      lcd.print("System Idle");
      break;

    case RUNNING:
      set_leds(0, 0, 1, 0); // Blue LED
      U0print_string("STATE: RUNNING\n");
      break;

    case ERROR:
      set_leds(0, 0, 0, 1); // Red LED blinking or solid
      U0print_string("STATE: ERROR!\n");
      lcd.clear();
      lcd.print("SENSOR ERROR");
      break;
  }
}

void set_leds(bool g, bool y, bool b, bool r){
  if(g) {
    PORTH |= (1 << LED_IDLE);
  } else {
    PORTH &= ~(1 << LED_IDLE);
  }
  if(y) {
    PORTH |= (1 << LED_DISABLED);
  } else {
    PORTH &= ~(1 << LED_DISABLED);
  }
  if(b) {
    PORTH |= (1 << LED_RUNNING);
  } else {
    PORTH &= ~(1 << LED_RUNNING);
  }
  if(r) {
    PORTH |= (1 << LED_ERROR);
  } else {
    PORTH &= ~(1 << LED_ERROR);
  }
}

void update_lcd(float t, float h){
  char buffer[10];

  dtostrf(h, 4, 1, buffer); // Convert float to string
  lcd.clear();
  lcd.print("Humidity: ");
  lcd.print(buffer);
  lcd.print(" %");
  U0print_string("Humidity: ");
  U0print_string(buffer);
  U0print_string(" %\t");

  dtostrf(t, 4, 1, buffer); // Convert float to string
  lcd.setCursor (0, 1);
  lcd.print("Temp: ");
  lcd.print(buffer);
  lcd.print(" *C");
  U0print_string("Temperature: ");
  U0print_string(buffer);
  U0print_string(" *C\n");
}