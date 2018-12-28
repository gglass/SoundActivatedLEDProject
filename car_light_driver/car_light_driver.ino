/**
 * 
 * BLE Stuff
 * 
 */

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include "wiring_private.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);



/**
 * 
 * LED/Mic Stuff
 * 
 */

#define N_PIXELS  253  // Number of pixels in strand
#define MIC_PIN   A0  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     10  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 4  // Rate of peak falling dot

byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512,
  MODE = 5,
  r_pin = 12,
  g_pin = 5,
  b_pin = 11,
  color[3],
  wiper = 255,
  wipeg = 128,
  MINIMUM = 10,
  wipeb = 0;
char
  r_dir = 'd',  
  g_dir = 'u',  
  b_dir = 'u';

void setup(void) {
  
  analogWrite(r_pin,0);
  analogWrite(g_pin,255);
  analogWrite(b_pin,0);

  color[0] = 0;
  color[1] = 0;
  color[2] = 0;
  
  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
//  analogReference(EXTERNAL);

  memset(vol, 0, sizeof(vol));
//  strip.begin();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  Serial.println( "OK!" );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));

  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  ble.verbose(false);  // debug info is a little annoying after this point!

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);

    // Set module to DATA mode
    Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("******************************"));
  }


  
}

void loop() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  int value[3];

  String command = "";
  int time = 0;
  

  while ( ble.available() )
  {
    time = time+1;
    if (time > 20000) {
      break;
    }
    ble.readline();
    Serial.println(ble.buffer);

    command = ble.buffer;

    Serial.println(command.substring(0,2));

    if(command.substring(0,2) == "AC") {
      Serial.print("SETTING COLOR: [");
      color[0] = command.substring(2,5).toInt();
      color[1] = command.substring(5,8).toInt();
      color[2] = command.substring(8,11).toInt();
      Serial.print(color[0]);
      Serial.print(",");
      Serial.print(color[1]);
      Serial.print(",");
      Serial.print(color[2]);
      Serial.println("]");
    }

    if(command.substring(0,2) ==  "RC") {
      Serial.print("REMOVING COLOR: [");
      color[0] = command.substring(2,5).toInt();
      color[1] = command.substring(5,8).toInt();
      color[2] = command.substring(8,11).toInt();
      Serial.print(color[0]);
      Serial.print(",");
      Serial.print(color[1]);
      Serial.print(",");
      Serial.print(color[2]);
      Serial.println("]");
    }

    if(command.substring(0,2) == "SM") {
      MODE = command.substring(2,3).toInt();
      Serial.print("SETTING MODE: ");
      Serial.println(MODE);
    }
    
  }

  if(MODE == 0) {
    n   = analogRead(MIC_PIN);                        // Raw reading from mic 
    n   = abs(n - 512 - DC_OFFSET); // Center on zero
    n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
    lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
  
    // Calculate bar height based on dynamic min/max levels (fixed point):
    height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
  
    if(height < 0L)       height = 0;      // Clip output
    else if(height > TOP) height = TOP;
    if(height > peak)     peak   = height; // Keep 'peak' dot at top

    if(peak < MINIMUM){
      peak = MINIMUM;
    }
  
//    Serial.print("Height: ");  Serial.print(height);  Serial.print( "Peak: ");  Serial.println(peak); 

    value[0] = (color[0]*peak)/N_PIXELS;
    value[1] = (color[1]*peak)/N_PIXELS;
    value[2] = (color[2]*peak)/N_PIXELS;
    
//    Serial.print( "Value: [");  Serial.print(value[0]); Serial.print(",");Serial.print(value[1]);Serial.print(",");Serial.print(value[2]);Serial.println("]");

    analogWrite(r_pin,value[0]);
    analogWrite(g_pin,value[1]);
    analogWrite(b_pin,value[2]);
    
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
       dotCount = 0;
    }

    vol[volCount] = n;                      // Save sample for dynamic leveling
    if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
  
    // Get volume range of prior frames
    minLvl = maxLvl = vol[0];
    for(i=1; i<SAMPLES; i++) {
      if(vol[i] < minLvl)      minLvl = vol[i];
      else if(vol[i] > maxLvl) maxLvl = vol[i];
    }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
    if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
    minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
    maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
    
  }

  if(MODE == 1) {
    analogWrite(r_pin,color[0]);
    analogWrite(g_pin,color[1]);
    analogWrite(b_pin,color[2]);
  }

  if(MODE == 2) {

    Serial.print("Showing COLOR: [");
    Serial.print(wiper);
    Serial.print(",");
    Serial.print(wipeg);
    Serial.print(",");
    Serial.print(wipeb);
    Serial.println("]");
      
    analogWrite(r_pin,wiper);
    analogWrite(g_pin,wipeg);
    analogWrite(b_pin,wipeb);
    
    if(wiper == 255) {
      r_dir = 'd';
    }
    if(wiper == 0) {
      r_dir = 'u';
    }
    if(r_dir == 'd') {
      wiper--;
    } else {
      wiper++;
    }

    if(wipeg == 255) {
      g_dir = 'd';
    }
    if(wipeg == 0) {
      g_dir = 'u';
    }
    if(g_dir == 'd') {
      wipeg--;
    } else {
      wipeg++;
    }

    if(wipeb == 255) {
      b_dir = 'd';
    }
    if(wipeb == 0) {
      b_dir = 'u';
    }
    if(b_dir == 'd') {
      wipeb--;
    } else {
      wipeb++;
    }
    delay(30);
    
  }

  if(MODE == 3) {
    color[0] = 255;
    color[1] = 0;
    color[2] = 0;
    MODE = 0;
  }

  //christmas mode
  if(MODE == 4) {
    analogWrite(r_pin,255);
    analogWrite(g_pin,0);
    analogWrite(b_pin,0);
    delay(333);
    analogWrite(r_pin,0);
    analogWrite(g_pin,255);
    analogWrite(b_pin,0);
    delay(333);
  }

//christmas mode
  if(MODE == 5) {
    analogWrite(r_pin,0);
    analogWrite(g_pin,0);
    analogWrite(b_pin,0);
  }

}

