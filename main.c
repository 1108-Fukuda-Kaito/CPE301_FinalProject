#include <avr/io.h>
#include <avr/interrupt.h>
#include "DHT.h"

#define PIN_RESET      4  // PB4
#define PIN_OFF        5  // PB5
#define PIN_ON         6  // PB6

#define LED_ON         6  // PH6
#define LED_OFF        4  // PH4

#define DHTPIN         2  // Digital Pin 2
#define DHTTYPE        DHT11

DHT dht(DHTPIN, DHTTYPE);

volatile unsigned char lastPinB = 0xFF;
volatile bool systemActive = false; 
unsigned long lastDHTRead = 0; // For non-blocking timing

void init_hardware(void);
void set_system_power(bool state);

void setup() {
  Serial.begin(9600);
  dht.begin();
  init_hardware();
  sei(); 
}

void loop() {
  unsigned char currentB = PINB;

  // OFF Button Logic: Instant response
  if (!(currentB & (1 << PIN_OFF)) && (lastPinB & (1 << PIN_OFF))) {
      set_system_power(false);
  }

  // ON Button Logic (Redundant with ISR but keeps logic safe)
  if (!(currentB & (1 << PIN_ON)) && (lastPinB & (1 << PIN_ON))) {
      set_system_power(true);
  }

  // SENSOR LOGIC: Only run if active AND 2 seconds have passed
  if (systemActive) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastDHTRead >= 2000) {
      lastDHTRead = currentMillis; // Reset timer

      float h = dht.readHumidity();
      float t = dht.readTemperature();

      if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
      } else {
        Serial.print("Humidity: ");
        Serial.print(h);
        Serial.print(" %\t");
        Serial.print("Temperature: ");
        Serial.print(t);
        Serial.println(" *C");
      }
    }
  }
  
  lastPinB = currentB;
}

// ISR triggers when ON button (PB6) is pressed
ISR(PCINT0_vect) {
    unsigned char currentB = PINB;
    // Check if PB6 transitioned to LOW
    if (!(currentB & (1 << PIN_ON)) && (lastPinB & (1 << PIN_ON))) {
        systemActive = true;
        PORTH |= (1 << LED_ON);
        PORTH &= ~(1 << LED_OFF);
    }
    lastPinB = currentB;
}

void init_hardware(void) {
    // Buttons as Input with Pull-up
    DDRB &= ~((1 << PIN_RESET) | (1 << PIN_OFF) | (1 << PIN_ON));
    PORTB |= ((1 << PIN_RESET) | (1 << PIN_OFF) | (1 << PIN_ON));
    
    // LEDs as Output
    DDRH |= (1 << LED_ON) | (1 << LED_OFF);
    
    // Initial state: OFF
    set_system_power(false);

    // Pin Change Interrupts
    PCICR  |= (1 << PCIE0);   
    PCMSK0 |= (1 << PCINT6);  
}

void set_system_power(bool state) {
    systemActive = state;
    if (state) {
        PORTH |= (1 << LED_ON);
        PORTH &= ~(1 << LED_OFF);
        Serial.println("SYSTEM ON");
    } else {
        PORTH &= ~(1 << LED_ON);
        PORTH |= (1 << LED_OFF);
        Serial.println("SYSTEM OFF");
    }
}