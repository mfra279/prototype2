/*
 *  ATTiny85 - Pulse Oximetry/Temperature/Heart Rate Monitor
 *  DEFINITELY NOT FOR MEDICAL USE
 *
 * Original by j.n.magee 15-10-2019 https://github.com/jeffmer/tinyPulsePPG
 * Modified by: M Fraser (reducing program size, adding temperature, etc)
 */

// Definitions for enabling various space-saving measures
#define USE_CUSTOM_DELAY = true
#define USE_CHECK_BUTTON_MACRO = true
#define USE_ADC_VIA_REGISTERS = true

// Library Includes
#include "ATtinySerialOut.hpp"
#include "ssd1306h.h" // modified version with unused functions commented out to reduce size
#include "MAX30102.h" // modified version with temperature support
#include "Pulse.h"
#include <avr/pgmspace.h>
#include "EEPROM.h"
#include <avr/sleep.h>
#include <avr/wdt.h>

// Routines to clear and set bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Pin definitions
#define TX_PIN PB5
#define LED 1
#define BUTTON PB4
#define OPTIONS 7

// Program variables
SSD1306 oled;
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;
int tfs = 256;
int hun = 100;

#ifdef USE_CUSTOM_DELAY
inline void customDelay(int milliseconds) __attribute__((always_inline)); // Uses this instead of delay() to reduce code size (if USE_CUSTOM_DELAY is true)
void customDelay(int milliseconds)
{
  static unsigned long iterations = (milliseconds * F_CPU) / 1000;
  for (unsigned long i = 0; i < iterations; i++)
  {
    asm(""); // do nothing
  }
}
#endif

// spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t spo2_table[184] PROGMEM =
    {95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
     99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
     100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
     97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
     90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
     80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
     66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
     49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
     28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
     3, 2, 1};

int getVCC()
{
#ifdef USE_ADC_VIA_REGISTERS
  ADMUX |= (1 << MUX1) | (1 << MUX0); // select analog input ADC2 (pin PB4)
  ADCSRA |= (1 << ADSC);              // start the ADC conversion
  while (ADCSRA & (1 << ADSC))
    ;                            // wait for the ADC conversion to complete
  static int analogValue = ADCW; // read the ADC result from ADCW register
  return min(11264 / analogValue, 99);
#else
  return min(11264 / analogRead(12), 99);
#endif
}

void print_digit(int x, int y, long val, char c = ' ', uint8_t field = 3,
                 const int BIG = 2)
{
  uint8_t ff = field;
  do
  {
    char ch = (val != 0) ? val % 10 + '0' : c;
    oled.drawChar(x + BIG * (ff - 1) * 6, y, ch, BIG);
    val = val / 10;
    --ff;
  } while (ff > 0);
}

int SPO2, SPO2f, voltage, beatAvg, tempC;
bool filter_for_graph = false;
bool draw_Red = false;
uint8_t pcflag = 0;
uint8_t istate = 0;
uint8_t sleep_counter = 0;
long lastBeat = 0;    // Time of the last beat
long displaytime = 0; // Time of the last display update
bool led_on = false;

ISR(PCINT0_vect)
{
  pcflag = 1;
}

#ifdef USE_CHECK_BUTTON_MACRO
#define CHECK_BUTTON()                           \
  do                                             \
  {                                              \
    if (pcflag)                                  \
    {                                            \
      if (!(PINB & (1 << BUTTON)))               \
      {                                          \
        istate = (istate + 1) % 4;               \
        filter_for_graph = istate & 0x01;        \
        draw_Red = istate & 0x02;                \
        EEPROM.write(OPTIONS, filter_for_graph); \
        EEPROM.write(OPTIONS + 1, draw_Red);     \
      }                                          \
    }                                            \
    pcflag = 0;                                  \
  } while (0)
#else
void checkbutton() // check if the button is pressed (this function is only used instead of the CHECK_BUTTON macro if USE_CHECK_BUTTON_MACRO is false)
{
  if (pcflag)
  {
    if (!(PINB & (1 << BUTTON)))
    {
      istate = (istate + 1) % 4;
      filter_for_graph = istate & 0x01;
      draw_Red = istate & 0x02;
      EEPROM.write(OPTIONS, filter_for_graph);
      EEPROM.write(OPTIONS + 1, draw_Red);
    }
  }
  pcflag = 0;
}
#endif

void go_sleep() // Enter sleep mode (used after 10 seconds of inactivity)
{
  oled.fill(0);
  oled.off();
#ifdef USE_CUSTOM_DELAY
  customDelay(10);
#else
  delay(10);
#endif
  sensor.off();
#ifdef USE_CUSTOM_DELAY
  customDelay(10);
#else
  delay(10);
#endif
  cbi(ADCSRA, ADEN); // disable adc
#ifdef USE_CUSTOM_DELAY
  customDelay(10);
#else
  delay(10);
#endif
  DDRB |= (0 << 0); // set pin 0 as input
  DDRB |= (0 << 2); // set pin 2 as input
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode(); // sleep until button press
  // cause reset
  wdt_enable(WDTO_15MS);
  wdt_reset();
  while (1)
    ;
}

void draw_oled(int msg)
{
  // Displays data on OLED and sends via Serial
  // Format: "temp,hr,spo2."

  oled.firstPage();
  do
  {
    switch (msg)
    {
    case 0:
      oled.drawStr(10, 0, F("Err"), 1);
      break;
    case 1:
      oled.drawStr(13, 10, F("Start"), 1);
      break;
    case 2:
      print_digit(86, 0, beatAvg);     // print heart rate to oled
      print_digit(16, 16, (int)tempC); // print temp to oled
      print_digit(98, 16, SPO2f, ' ', 3, 1);
      oled.drawChar(116, 16, '%');
      print_digit(98, 24, SPO2, ' ', 3, 1); // print spo2 to oled
      oled.drawChar(116, 24, '%');
      break;
    default:
      break;
    }
  } while (oled.nextPage());
}

void setup(void)
{

  MCUSR = 0;
  wdt_disable();
  DDRB |= (1 << LED);     // set LED pin for output
  PORTB |= (1 << BUTTON); // set the corresponding bit in the PORTB register to 1 (pullup resistor)
  DDRB &= ~(1 << BUTTON); // set the corresponding bit in the DDRB register to 0 (input mode)
  initTXPin();            // init ATTinySerialOut
  filter_for_graph = EEPROM.read(OPTIONS);
  draw_Red = EEPROM.read(OPTIONS + 1);
  oled.init();
  oled.fill(0x00);
  draw_oled(3);
#ifdef USE_CUSTOM_DELAY
  customDelay(3000);
#else
  delay(3000);
#endif
  if (!sensor.begin())
  { // sensor isn't starting, show error on OLED
    draw_oled(0);
    while (1)
      ;
  }
  sensor.setup();
  sensor.enableDIETEMPRDY(); // enable the temp ready interrupt (required for temperature reading)
  sbi(GIMSK, PCIE);          // set up pin change interrupt
  sbi(PCMSK, PCINT3);
  sei();
}

void loop()
{
  sensor.check();
  long now = millis(); // start time of this cycle
  if (!sensor.available())
    return;
  uint32_t irValue = sensor.getIR();
  uint32_t redValue = sensor.getRed();
  sensor.nextSample();
  if (irValue < 5000)
  {
    voltage = getVCC();
#ifdef USE_CHECK_BUTTON_MACRO
    CHECK_BUTTON(); // check button press
#else
    checkbutton(); // check button press
#endif
    draw_oled(sleep_counter <= 70 ? 1 : 4); // finger not down message
#ifdef USE_CUSTOM_DELAY
    customDelay(hun);
#else
    delay(hun);
#endif
    ++sleep_counter;
    if (sleep_counter > hun)
    {
      go_sleep();
      sleep_counter = 0;
    }
  }
  else
  {
    sleep_counter = 0;
    int16_t IR_signal, Red_signal;
    bool beatRed, beatIR;
    IR_signal = pulseIR.dc_filter(irValue); // remove DC element
    Red_signal = pulseRed.dc_filter(redValue);
    beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
    beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));
    // wave.record(draw_Red ? -Red_signal : -IR_signal );
    //  check temperature
    tempC = sensor.readTemperature();
    // check IR or Red for heartbeat
    if (draw_Red ? beatRed : beatIR)
    {
      long btpm = 60000 / (now - lastBeat);
      if (btpm > 0 && btpm < 200)
        beatAvg = bpm.filter((int16_t)btpm);
      lastBeat = now;
      PORTB |= (1 << LED); // Set LED pin high
      led_on = true;
      // compute SpO2 ratio
      long numerator = (pulseRed.avgAC() * pulseIR.avgDC()) / tfs;
      long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / tfs;
      int RX100 = (denominator > 0) ? (numerator * hun) / denominator : 999;
      // using formula
      SPO2f = (10400 - RX100 * 17 + 50) / hun;
      // from table
      if ((RX100 >= 0) && (RX100 < 184))
        SPO2 = pgm_read_byte_near(&spo2_table[RX100]);
    }
    // update display every 50 ms if fingerdown
    if (now - displaytime > 50)
    {
      displaytime = now;
      draw_oled(2);
    }
  }
  // flash led for 25 ms
  if (led_on && (now - lastBeat) > 25)
  {
    PORTB |= (0 << LED); // Set LED pin low (off)
    led_on = false;
  }
}
