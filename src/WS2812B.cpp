/*-------------------------------------------------------------------------
  Arduino library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for various color orders.  Handles most output pins
  (possible exception with upper PORT registers on the Arduino Mega).

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC, Michael Miller and other members of the open
  source community.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "WS2812B.h"
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>


// Constructor when length, pin and type are known at compile-time:
WS2812B::WS2812B(uint16_t n) :
  begun(false), brightness(0), pixels(NULL), endTime(0)  
{
  updateLength(n);
}


WS2812B::~WS2812B() {
  if(pixels)   
  {
	  free(pixels);
  }
  SPI.end();
}

void WS2812B::begin(void) {

  SPI.setClockDivider(SPI_CLOCK_DIV32);// need bit rate of 400nS but closest we can do @ 72Mhz is 444ns (which is within spec)
  SPI.begin();
  begun = true;
}

void WS2812B::updateLength(uint16_t n) {
  if(pixels) 
  {
	  free(pixels); // Free existing data (if any)
  }
  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  numBytes = (n<<3) + n + 2; 
  if((pixels = (uint8_t *)malloc(numBytes)))
  {
    memset(pixels, 0, numBytes);
    numLEDs = n;
	clear();
  } 
  else 
  {
    numLEDs = numBytes = 0;
  }
}


void WS2812B::show(void) 
{
  SPI.dmaSend(pixels,numBytes);
}


void WS2812B::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
 {
   uint8_t *bptr = pixels + (n<<3) + n +1;

   *bptr++ = byte0Lookup[g];
   *bptr++ = byte1Lookup[g];
   *bptr++ = byte2Lookup[g];

   *bptr++ = byte0Lookup[r];
   *bptr++ = byte1Lookup[r];
   *bptr++ = byte2Lookup[r];

   *bptr++ = byte0Lookup[b];
   *bptr++ = byte1Lookup[b];
   *bptr++ = byte2Lookup[b];
 }

void WS2812B::setPixelColor(uint16_t n, uint32_t c)
  {
     uint8_t r,g,b;
   
    if(brightness) 
	{ 
		// See notes in setBrightness()
      r = ((int)((uint8_t)(c >> 16)) * (int)brightness) >> 8;
      g = ((int)((uint8_t)(c >>  8)) * (int)brightness) >> 8;
      b = ((int)((uint8_t)c) * (int)brightness) >> 8;
	}
	else
	{
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
	  b = (uint8_t)c;		
	}
   setPixelColor(n,r,g,b);
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t WS2812B::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

// Convert separate R,G,B,W into packed 32-bit WRGB color.
// Packed format is always WRGB, regardless of LED strand color order.
uint32_t WS2812B::Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}


uint16_t WS2812B::numPixels(void) const {
  return numLEDs;
}

// Adjust output brightness; 0=darkest (off), 255=brightest.  This does
// NOT immediately affect what's currently displayed on the LEDs.  The
// next call to show() will refresh the LEDs at this level.  However,
// this process is potentially "lossy," especially when increasing
// brightness.  The tight timing in the WS2811/WS2812 code means there
// aren't enough free cycles to perform this scaling on the fly as data
// is issued.  So we make a pass through the existing color data in RAM
// and scale it (subsequent graphics commands also work at this
// brightness level).  If there's a significant step up in brightness,
// the limited number of steps (quantization) in the old data will be
// quite visible in the re-scaled version.  For a non-destructive
// change, you'll need to re-render the full strip data.  C'est la vie.
void WS2812B::setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed.
  // This simplifies the actual scaling math later, allowing a fast
  // 8x8-bit multiply and taking the MSB.  'brightness' is a uint8_t,
  // adding 1 here may (intentionally) roll over...so 0 = max brightness
  // (color values are interpreted literally; no scaling), 1 = min
  // brightness (off), 255 = just below max brightness.
  uint8_t newBrightness = b + 1;
  if(newBrightness != brightness) { // Compare against prior value
    // Brightness has changed -- re-scale existing data in RAM
    uint8_t  c,
            *ptr           = pixels,
             oldBrightness = brightness - 1; // De-wrap old brightness value
    uint16_t scale;
    if(oldBrightness == 0) scale = 0; // Avoid /0
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    for(uint16_t i=0; i<numBytes; i++) {
      c      = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}

//Return the brightness value
uint8_t WS2812B::getBrightness(void) const {
  return brightness - 1;
}


void WS2812B::clear() 
{
	uint8_t * bptr= pixels+1;
	for(int i=0;i< (numLEDs *3);i++)
	{
		*bptr++ = byte0Lookup[0];
		*bptr++ = byte1Lookup[0];
		*bptr++ = byte2Lookup[0];
	}
}
