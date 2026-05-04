#include <avr/io.h>
#include <avr/interrupt.h>
#include "DHT.h"
#include <LiquidCrystal.h>

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

#define LED_ON         6  // PH6 (Digital 9)
#define LED_OFF        4  // PH4 (Digital 7)

#define DHTPIN         2  // Digital Pin 2
#define DHTTYPE        DHT11

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

volatile unsigned char lastPinB = 0xFF;
volatile bool systemActive = false; 
unsigned long lastDHTRead = 0; // For non-blocking timing
bool lastSystemState = false;

void setup() {
  U0init(9600);
  Serial.begin(9600);
  dht.begin();
  init_hardware();
  sei(); 
  lcd.begin(16, 2); // set up number of columns and rows
}

void loop() {
  unsigned char currentB = PINB;

  // 1. POLL OFF BUTTON: Set the flag, but don't print here
  if (!(currentB & (1 << PIN_OFF)) && (lastPinB & (1 << PIN_OFF))) {
      systemActive = false;
      _delay_ms(50); // Small debounce
  }

  // 2. STATE MANAGER: This is the ONLY place that calls set_system_power
  // It checks if the flag was changed by the ISR or the loop
  if (systemActive != lastSystemState) {
      set_system_power(systemActive);
      lastSystemState = systemActive;
  }

  // SENSOR LOGIC: Only run if active AND 2 seconds have passed
  if (systemActive) {
    temp_humid_sensor();
  }
  
  lastPinB = currentB;
}

// ISR: ONLY detects the hardware event and flips the flag
ISR(PCINT0_vect) {
    unsigned char currentB = PINB;
    if (!(currentB & (1 << PIN_ON)) && (lastPinB & (1 << PIN_ON))) {
        systemActive = true; 
    }
    // We don't update lastPinB here; loop handles it to keep logic in sync
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

void set_system_power(bool state) {
    if (state) {
        PORTH |= (1 << LED_ON);
        PORTH &= ~(1 << LED_OFF);
        U0print_string("SYSTEM ON\n");
        lcd.display();           // Turn on the display
        lcd.clear();
        lcd.print("System Starting...");
        my_delay_ms(2000);
    } else {
        PORTH &= ~(1 << LED_ON);
        PORTH |= (1 << LED_OFF);
        U0print_string("SYSTEM OFF\n");
        lcd.clear();
        lcd.print("System OFF");
        my_delay_ms(2000);
        lcd.noDisplay();
    }
}

void temp_humid_sensor(){
  unsigned long currentMillis = millis();
    
  if (currentMillis - lastDHTRead >= 2000) {
    lastDHTRead = currentMillis; // Reset timer

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      U0print_string("Failed to read from DHT sensor!\n");
    } else {
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