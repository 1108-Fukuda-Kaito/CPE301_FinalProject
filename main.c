#include <avr/io.h>
#include <avr/interrupt.h>
#include <DHT.h>

/* --- 1. HARDWARE DEFINITIONS --- */
#define PIN_RESET      4  // PB4
#define PIN_OFF        5  // PB5
#define PIN_ON         6  // PB6 (PCINT6)

#define LED_ON         6  // PH6
#define LED_OFF        4  // PH4

#define DHTPIN         2  // Digital Pin 2
#define DHTTYPE        DHT11

/* --- 2. GLOBAL VARIABLES --- */
DHT dht(DHTPIN, DHTTYPE);

volatile unsigned char lastPinB = 0xFF;
volatile bool systemActive = false; // Tracks if the system is logically ON
unsigned long lastDHTRead = 0;      // For non-blocking timing

/* --- 3. FUNCTION PROTOTYPES --- */
void init_hardware(void);
void set_system_power(bool state);
void handle_dht(void);
void handle_reset(void);

/* --- 4. MAIN APPLICATION --- */
int main(void) {
    init_hardware();
    Serial.begin(9600);
    dht.begin();
    sei(); // Enable Interrupts

    while (1) {
        unsigned char currentB = PINB;

        // Reset Button (Falling Edge)
        if (!(currentB & (1 << PIN_RESET)) && (lastPinB & (1 << PIN_RESET))) {
            handle_reset();
        }

        // OFF Button (Falling Edge)
        if (!(currentB & (1 << PIN_OFF)) && (lastPinB & (1 << PIN_OFF))) {
            set_system_power(false);
        }

        // Handle Sensor if System is Active
        if (systemActive) {
            handle_dht();
        }

        lastPinB = currentB;
    }
    return 0;
}

/* --- 5. INTERRUPT SERVICE ROUTINES --- */
ISR(PCINT0_vect) {
    unsigned char currentB = PINB;

    // ON button (PB6) Falling Edge
    if (!(currentB & (1 << PIN_ON)) && (lastPinB & (1 << PIN_ON))) {
        systemActive = true;
        set_system_power(true);
    }
    
    lastPinB = currentB;
}

/* --- 6. DRIVER IMPLEMENTATIONS --- */

void init_hardware(void) {
    // Inputs & Pull-ups
    DDRB &= ~((1 << PIN_RESET) | (1 << PIN_OFF) | (1 << PIN_ON));
    PORTB |= ((1 << PIN_RESET) | (1 << PIN_OFF) | (1 << PIN_ON));
    
    // Outputs
    DDRH |= (1 << LED_ON) | (1 << LED_OFF);
    
    set_system_power(false); // Start in OFF state

    // Interrupt Setup
    PCICR  |= (1 << PCIE0);   
    PCMSK0 |= (1 << PCINT6);  
}

void set_system_power(bool state) {
    if (state) {
        PORTH |= (1 << LED_ON);
        PORTH &= ~(1 << LED_OFF);
        systemActive = true;
    } else {
        PORTH &= ~(1 << LED_ON);
        PORTH |= (1 << LED_OFF);
        systemActive = false;
        Serial.println("System Shutdown.");
    }
}

void handle_dht(void) {
    if (millis() - lastDHTRead >= 2000) {
        lastDHTRead = millis();
        
        Serial.println("Attempting to read sensor..."); // Debug line

        float h = dht.readHumidity();
        float t = dht.readTemperature();

        if (isnan(h) || isnan(t)) {
            // This means the sensor is powered but not sending data
            Serial.println("DHT Error: Check wiring/pull-up resistor!"); 
            return;
        }

        Serial.print("Temp: ");
        Serial.print(t);
        Serial.print("C | Hum: ");
        Serial.print(h);
        Serial.println("%");
    }
}

void handle_reset(void) {
    Serial.println("Resetting System...");
    set_system_power(false);
    // Add additional variable clearing here
}