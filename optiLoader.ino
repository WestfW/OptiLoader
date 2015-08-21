// optiLoader.pde
//
// this sketch allows an Arduino to program Optiboot onto any other
// Arduino-like device containing ATmega8, ATmega168, or ATmega328
// microcontroller chips.
//
// Copyright (c) 2011 by Bill Westfield ("WestfW")

//-------------------------------------------------------------------------------------
// "MIT Open Source Software License":
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//-------------------------------------------------------------------------------------

//
// this sketch allows an Arduino to program Optiboot onto any other
// Arduino-like device containing ATmega8, ATmega168, or ATmega328
// microcontroller chips.
//
// It is based on AVRISP
//
// Designed to connect to a generic programming cable,
// using the following pins:
// 10: slave reset
// 11: MOSI
// 12: MISO
// 13: SCK
//  9: Power to external chip.
//     This is a little questionable, since the power it is legal to draw
//     from a PIC pin is pretty close to the power consumption of an AVR
//     chip being programmed.  But it permits the target to be entirely
//     powered down for safe reconnection of the programmer to additional
//     targets, and it seems to work for most Arduinos.  If the target board
//     contains additional circuitry and is expected to draw more than 40mA,
//     connect the target power to a stronger source of +5V.  Do not use pin
//     9 to power more complex Arduino boards that draw more than 40mA, such
//     as the Arduino Uno Ethernet !
//
// If the aim is to reprogram the bootloader in one Arduino using another
// Arudino as the programmer, you can just use jumpers between the connectors
// on the Arduino board.  In this case, connect:
// Pin 13 to Pin 13
// Pin 12 to Pin 12
// Pin 11 to Pin 11
// Pin 10 (of "programmer") to RESET (of "target" (on the "power" connector))
// +5V to +5V and GND to GND.  Only the "programmer" board should be powered
//     by USB or external power.
//
// ----------------------------------------------------------------------

// The following credits are from AVRISP.  It turns out that there isn't
// a lot of AVRISP left in this sketch, but probably if AVRISP had never
// existed,  this sketch would not have been written.
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
// 
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer 
// - More information at http://code.google.com/p/mega-isp


#include <avr/pgmspace.h>
#include "optiLoader.h"

char Arduino_preprocessor_hint;

/*
 * Pins to target
 */
#define SCK 13
#define MISO 12
#define MOSI 11
#define RESET 10
#define POWER 9

// STK Definitions; we can still use these as return codes
#define STK_OK 0x10
#define STK_FAILED 0x11


// Useful message printing definitions
#define fp(string) flashprint(PSTR(string));
#define debug(string) // flashprint(PSTR(string));
#define error(string) flashprint(PSTR(string));

// Forward references
void pulse(int pin, int times);
void read_image(image_t *ip);

// Global Variables

/*
 * Table of defined images
 */
image_t *images[] = {
  &image_328, &image_328p, &image_168, &image_8, 0
};

int pmode=0;
// address for reading and writing, set by 'U' command
int here;

uint16_t target_type = 0;		/* type of target_cpu */
uint16_t target_startaddr;
uint8_t target_pagesize;       /* Page size for flash programming (bytes) */
uint8_t *buff;

image_t *target_flashptr; 	       /* pointer to target info in flash */
uint8_t target_code[512];	       /* The whole code */


void setup () {
  Serial.begin(19200); 			/* Initialize serial for status msgs */
  pinMode(13, OUTPUT); 			/* Blink the pin13 LED a few times */
  pulse(13,20);
}

void loop (void) {
  fp("\nOptiLoader Bootstrap programmer.\n2011 by Bill Westfield (WestfW)\n\n");
  if (target_poweron()) {		/* Turn on target power */
    do {
      if (!target_identify()) 		/* Figure out what kind of CPU */
        break;
      if (!target_findimage())		/* look for an image */
        break;
      if (!target_progfuses())		/* get fuses ready to program */
        break;
      if (!target_program()) 		/* Program the image */
        break;
      (void) target_normfuses(); 	/* reset fuses to normal mode */
    } 
    while (0);
  } 
  else {
    Serial.println();
  }
  target_poweroff(); 			/* turn power off */

  fp ("\nType 'G' or hit RESET for next chip\n")
    while (1) {
      if (Serial.read() == 'G')
        break;
    }
}

/*
 * Low level support functions
 */

/*
 * flashprint
 * print a text string direct from flash memory to Serial
 */
void flashprint (const char p[])
{
  uint8_t c;
  while (0 != (c = pgm_read_byte(p++))) {
    Serial.write(c);
  }
}

/*
 * hexton
 * Turn a Hex digit (0..9, A..F) into the equivalent binary value (0-16)
 */
uint8_t hexton (uint8_t h)
{
  if (h >= '0' && h <= '9')
    return(h - '0');
  if (h >= 'A' && h <= 'F')
    return((h - 'A') + 10);
  error("Bad hex digit!");
}

/*
 * pulse
 * turn a pin on and off a few times; indicates life via LED
 */
#define PTIME 30
void pulse (int pin, int times) {
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } 
  while (times--);
}

/*
 * spi_init
 * initialize the AVR SPI peripheral
 */
void spi_init () {
  uint8_t x;
  SPCR = 0x53;  // SPIE | MSTR | SPR1 | SPR0
  x=SPSR;
  x=SPDR;
}

/*
 * spi_wait
 * wait for SPI transfer to complete
 */
void spi_wait () {
  debug("spi_wait");
  do {
  } 
  while (!(SPSR & (1 << SPIF)));
}

/*
 * spi_send
 * send a byte via SPI, wait for the transfer.
 */
uint8_t spi_send (uint8_t b) {
  uint8_t reply;
  SPDR=b;
  spi_wait();
  reply = SPDR;
  return reply;
}


/*
 * Functions specific to ISP programming of an AVR
 */

/*
 * target_identify
 * read the signature bytes (if possible) and check whether it's
 * a legal value (atmega8, atmega168, atmega328)
 */

boolean target_identify ()
{
  boolean result;
  target_type = 0;
  fp("\nReading signature:");
  target_type = read_signature();
  if (target_type == 0 || target_type == 0xFFFF) {
    fp(" Bad value: ");
    result = false;
  } 
  else {
    result = true;
  }
  Serial.println(target_type, HEX);
  if (target_type == 0) {
    fp("  (no target attached?)\n");
  }
  return result;
}

unsigned long spi_transaction (uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n, m;
  spi_send(a); 
  n=spi_send(b);
  //if (n != a) error = -1;
  m=spi_send(c);
  return 0xFFFFFF & ((n<<16)+(m<<8) + spi_send(d));
}

uint16_t start_pmode () {
  uint16_t result;

  pinMode(13, INPUT); // restore to default
  spi_init();
  debug("...spi_init done");
  // following delays may not work on all targets...
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  debug("...spi_transaction");
  result = spi_transaction(0xAC, 0x53, 0x00, 0x00);
  debug("...Done");
  pmode = 1;
  return result;
}

void end_pmode () {
  SPCR = 0; 				/* reset SPI */
  digitalWrite(MISO, 0); 		/* Make sure pullups are off too */
  pinMode(MISO, INPUT);
  digitalWrite(MOSI, 0);
  pinMode(MOSI, INPUT);
  digitalWrite(SCK, 0);
  pinMode(SCK, INPUT);
  digitalWrite(RESET, 0);
  pinMode(RESET, INPUT);
  pmode = 0;
}

/*
 * read_image
 *
 * Read an intel hex image from a string in pgm memory.
 * We assume that the image does not exceed the 512 bytes that we have
 * allowed for it to have.  that would be bad.
 * Also read other data from the image, such as fuse and protecttion byte
 * values during programming, and for after we're done.
 */
void read_image (image_t *ip)
{
  uint16_t len, totlen=0, addr;
  char *hextext = &ip->image_hexcode[0];
  target_startaddr = 0;
  target_pagesize = pgm_read_byte(&ip->image_pagesize);
  uint8_t b, cksum = 0;

  while (1) {
    if (pgm_read_byte(hextext++) != ':') {
      error("No colon");
      break;
    }
    len = hexton(pgm_read_byte(hextext++));
    len = (len<<4) + hexton(pgm_read_byte(hextext++));
    cksum = len;

    b = hexton(pgm_read_byte(hextext++)); /* record type */
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    addr = b;
    b = hexton(pgm_read_byte(hextext++)); /* record type */
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    addr = (addr << 8) + b;
    if (target_startaddr == 0) {
      target_startaddr = addr;
      fp("  Start address at ");
      Serial.println(addr, HEX);
    } 
    else if (addr == 0) {
      break;
    }

    b = hexton(pgm_read_byte(hextext++)); /* record type */
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;

    for (uint8_t i=0; i < len; i++) {
      b = hexton(pgm_read_byte(hextext++));
      b = (b<<4) + hexton(pgm_read_byte(hextext++));
      if (addr - target_startaddr >= sizeof(target_code)) {
        error("Code extends beyond allowed range");
        break;
      }
      target_code[addr++ - target_startaddr] = b;
      cksum += b;
#if VERBOSE
      Serial.print(b, HEX);
      Serial.write(' ');
#endif
      totlen++;
      if (totlen >= sizeof(target_code)) {
        error("Too much code");
        break;
      }
    }
    b = hexton(pgm_read_byte(hextext++)); /* checksum */
    b = (b<<4) + hexton(pgm_read_byte(hextext++));
    cksum += b;
    if (cksum != 0) {
      error("Bad checksum: ");
      Serial.print(cksum, HEX);
    }
    if (pgm_read_byte(hextext++) != '\n') {
      error("No end of line");
      break;
    }
#if VERBOSE
    Serial.println();
#endif
  }
  fp("  Total bytes read: ");
  Serial.println(totlen);
}

/*
 * target_findimage
 *
 * given target_type loaded with the relevant part of the device signature,
 * search the hex images that we have programmed in flash, looking for one
 * that matches.
 */

boolean target_findimage ()
{
  image_t *ip;
  fp("Searching for image...\n");
  for (uint8_t i=0; i < sizeof(images)/sizeof(images[0]); i++) {
    target_flashptr = ip = images[i];
    if (ip && (pgm_read_word(&ip->image_chipsig) == target_type)) {
      fp("  Found \"");
      flashprint(&ip->image_name[0]);
      fp("\" for ");
      flashprint(&ip->image_chipname[0]);
      fp("\n");
      read_image(ip);
      return true;
    }
  }
  fp(" Not Found\n");
  return(false);
}

/*
 * target_progfuses
 * given initialized target image data, re-program the fuses to allow
 * the optiboot image to be programmed.
 */

boolean target_progfuses ()
{
  uint8_t f;
  fp("\nSetting fuses for programming");

  f = pgm_read_byte(&target_flashptr->image_progfuses[FUSE_PROT]);
  if (f) {
    fp("\n  Lock: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xE0, 0x00, f), HEX);
  }
  f = pgm_read_byte(&target_flashptr->image_progfuses[FUSE_LOW]);
  if (f) {
    fp("  Low: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xA0, 0x00, f), HEX);
  }
  f = pgm_read_byte(&target_flashptr->image_progfuses[FUSE_HIGH]);
  if (f) {
    fp("  High: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xA8, 0x00, f), HEX);
  }
  f = pgm_read_byte(&target_flashptr->image_progfuses[FUSE_EXT]);
  if (f) {
    fp("  Ext: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xA4, 0x00, f), HEX);
  }
  Serial.println();
  return true; 			/* */
}

/*
 * target_program
 * Actually program the image into the target chip
 */

boolean target_program ()
{
  int l; 				/* actual length */

  fp("\nProgramming bootloader: ");
  here = target_startaddr>>1; 		/* word address */
  buff = target_code;
  l = 512;
  Serial.print(l, DEC);
  fp(" bytes at 0x");
  Serial.println(here, HEX);

  spi_transaction(0xAC, 0x80, 0, 0); 	/* chip erase */
  delay(1000);
  if (write_flash(l) != STK_OK) {
    error("\nFlash Write Failed");
    return false;
  }
  return true; 			/*  */
}

/*
 * target_normfuses
 * reprogram the fuses to the state they should be in for bootloader
 * based programming
 */
boolean target_normfuses ()
{
  uint8_t f;
  fp("\nRestoring normal fuses");

  f = pgm_read_byte(&target_flashptr->image_normfuses[FUSE_PROT]);
  if (f) {
    fp("\n  Lock: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xE0, 0x00, f), HEX);
  }
  f = pgm_read_byte(&target_flashptr->image_normfuses[FUSE_LOW]);
  if (f) {
    fp("  Low: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xA0, 0x00, f), HEX);
  }
  f = pgm_read_byte(&target_flashptr->image_normfuses[FUSE_HIGH]);
  if (f) {
    fp("  High: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xA8, 0x00, f), HEX);
  }
  f = pgm_read_byte(&target_flashptr->image_normfuses[FUSE_EXT]);
  if (f) {
    fp("  Ext: ");
    Serial.print(f, HEX);
    fp(" ");
    Serial.print(spi_transaction(0xAC, 0xA4, 0x00, f), HEX);
  }
  Serial.println();
  return true; 			/* */
}

/*
 * target_poweron
 * Turn on power to the target chip (assuming that it is powered through
 * the relevant IO pin of THIS arduino.)
 */
boolean target_poweron ()
{
  uint16_t result;

  fp("Target power on! ...");
  digitalWrite(POWER, LOW);
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, HIGH);
  digitalWrite(RESET, LOW);  // reset it right away.
  pinMode(RESET, OUTPUT);
  /*
   * Check if the target is pulling RESET HIGH by reverting to input
   */
  delay(5);
  pinMode(RESET, INPUT);
  delay(1);
  if (digitalRead(RESET) != HIGH) {
    fp("No RESET pullup detected! - no target?");
    return false;
  }
  pinMode(RESET, OUTPUT);

  delay(200);
  fp("\nStarting Program Mode");
  result = start_pmode();
  if ((result & 0xFF00) != 0x5300) {
    fp(" - Failed, result = 0x");
    Serial.print(result, HEX);
    return false;
  }
  fp(" [OK]\n");
  return true;
}

boolean target_poweroff ()
{
  end_pmode();
  digitalWrite(POWER, LOW);
  delay(200);
  pinMode(POWER, INPUT);
  fp("\nTarget power OFF!\n");
  return true;
}

void flash (uint8_t hilo, int addr, uint8_t data) {
#if VERBOSE
  Serial.print(data, HEX);
  fp(":");
  Serial.print(spi_transaction(0x40+8*hilo, 
  addr>>8 & 0xFF, 
  addr & 0xFF,
  data), HEX);
  fp(" ");
#else
  (void) spi_transaction(0x40+8*hilo, 
  addr>>8 & 0xFF, 
  addr & 0xFF,
  data);
#endif
}

void commit (int addr) {
  fp("  Commit Page: ");
  Serial.print(addr, HEX);
  fp(":");
  Serial.println(spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0), HEX);
  delay(100);
}

//#define _current_page(x) (here & 0xFFFFE0)
int current_page (int addr) {
  if (target_pagesize == 32) return here & 0xFFFFFFF0;
  if (target_pagesize == 64) return here & 0xFFFFFFE0;
  if (target_pagesize == 128) return here & 0xFFFFFFC0;
  return here;
}

uint8_t write_flash (int length) {
  if (target_pagesize < 1) return STK_FAILED;
  //if (target_pagesize != 64) return STK_FAILED;
  int page = current_page(here);
  int x = 0;
  while (x < length) {
    if (page != current_page(here)) {
      commit(page);
      page = current_page(here);
    }
    flash(LOW, here, buff[x]);
    flash(HIGH, here, buff[x+1]);
    x+=2;
    here++;
  }

  commit(page);

  return STK_OK;
}

uint16_t read_signature () {
  uint8_t sig_middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  uint8_t sig_low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  return ((sig_middle << 8) + sig_low);
}

/*
 * Bootload images.
 * These are the intel Hex files produced by the optiboot makefile,
 * with a small amount of automatic editing to turn them into C strings,
 * and a header attched to identify them
 */


image_t PROGMEM image_328 = {
  {
    "optiboot_atmega328.hex"    }
  ,
  {
    "atmega328"    }
  ,
  0x9514,				/* Signature bytes for 328 (non-P) */
  {
    0x3F,0xFF,0xDE,0x05,0    }
  ,
  {
    0x2F,0,0,0,0    }
  ,
  128,
  {
    ":107E0000112484B714BE81FFF0D085E080938100F7\n"
      ":107E100082E08093C00088E18093C10086E0809377\n"
      ":107E2000C20080E18093C4008EE0C9D0259A86E02C\n"
      ":107E300020E33CEF91E0309385002093840096BBD3\n"
      ":107E4000B09BFECF1D9AA8958150A9F7CC24DD24C4\n"
      ":107E500088248394B5E0AB2EA1E19A2EF3E0BF2EE7\n"
      ":107E6000A2D0813461F49FD0082FAFD0023811F036\n"
      ":107E7000013811F484E001C083E08DD089C08234E0\n"
      ":107E800011F484E103C0853419F485E0A6D080C0E4\n"
      ":107E9000853579F488D0E82EFF2485D0082F10E0AE\n"
      ":107EA000102F00270E291F29000F111F8ED06801E7\n"
      ":107EB0006FC0863521F484E090D080E0DECF843638\n"
      ":107EC00009F040C070D06FD0082F6DD080E0C81688\n"
      ":107ED00080E7D80618F4F601B7BEE895C0E0D1E017\n"
      ":107EE00062D089930C17E1F7F0E0CF16F0E7DF06D8\n"
      ":107EF00018F0F601B7BEE89568D007B600FCFDCFD4\n"
      ":107F0000A601A0E0B1E02C9130E011968C91119780\n"
      ":107F100090E0982F8827822B932B1296FA010C0160\n"
      ":107F200087BEE89511244E5F5F4FF1E0A038BF0790\n"
      ":107F300051F7F601A7BEE89507B600FCFDCF97BE46\n"
      ":107F4000E89526C08437B1F42ED02DD0F82E2BD052\n"
      ":107F50003CD0F601EF2C8F010F5F1F4F84911BD097\n"
      ":107F6000EA94F801C1F70894C11CD11CFA94CF0C13\n"
      ":107F7000D11C0EC0853739F428D08EE10CD085E9AC\n"
      ":107F80000AD08FE07ACF813511F488E018D01DD067\n"
      ":107F900080E101D065CF982F8091C00085FFFCCF94\n"
      ":107FA0009093C60008958091C00087FFFCCF809118\n"
      ":107FB000C00084FD01C0A8958091C6000895E0E648\n"
      ":107FC000F0E098E1908380830895EDDF803219F02E\n"
      ":107FD00088E0F5DFFFCF84E1DECF1F93182FE3DFCA\n"
      ":107FE0001150E9F7F2DF1F91089580E0E8DFEE27F6\n"
      ":047FF000FF270994CA\n"
      ":027FFE00040479\n"
      ":0400000300007E007B\n"
      ":00000001FF\n"
  }
};

image_t PROGMEM image_328p = {
  {
    "optiboot_atmega328.hex"    }
  ,
  {
    "atmega328P"    }
  ,
  0x950F,				/* Signature bytes for 328P */
  {
    0x3F,0xFF,0xDE,0x05,0    }
  ,
  {
    0x2F,0,0,0,0    }
  ,
  128,
  {
    ":107E0000112484B714BE81FFF0D085E080938100F7\n"
      ":107E100082E08093C00088E18093C10086E0809377\n"
      ":107E2000C20080E18093C4008EE0C9D0259A86E02C\n"
      ":107E300020E33CEF91E0309385002093840096BBD3\n"
      ":107E4000B09BFECF1D9AA8958150A9F7CC24DD24C4\n"
      ":107E500088248394B5E0AB2EA1E19A2EF3E0BF2EE7\n"
      ":107E6000A2D0813461F49FD0082FAFD0023811F036\n"
      ":107E7000013811F484E001C083E08DD089C08234E0\n"
      ":107E800011F484E103C0853419F485E0A6D080C0E4\n"
      ":107E9000853579F488D0E82EFF2485D0082F10E0AE\n"
      ":107EA000102F00270E291F29000F111F8ED06801E7\n"
      ":107EB0006FC0863521F484E090D080E0DECF843638\n"
      ":107EC00009F040C070D06FD0082F6DD080E0C81688\n"
      ":107ED00080E7D80618F4F601B7BEE895C0E0D1E017\n"
      ":107EE00062D089930C17E1F7F0E0CF16F0E7DF06D8\n"
      ":107EF00018F0F601B7BEE89568D007B600FCFDCFD4\n"
      ":107F0000A601A0E0B1E02C9130E011968C91119780\n"
      ":107F100090E0982F8827822B932B1296FA010C0160\n"
      ":107F200087BEE89511244E5F5F4FF1E0A038BF0790\n"
      ":107F300051F7F601A7BEE89507B600FCFDCF97BE46\n"
      ":107F4000E89526C08437B1F42ED02DD0F82E2BD052\n"
      ":107F50003CD0F601EF2C8F010F5F1F4F84911BD097\n"
      ":107F6000EA94F801C1F70894C11CD11CFA94CF0C13\n"
      ":107F7000D11C0EC0853739F428D08EE10CD085E9AC\n"
      ":107F80000AD08FE07ACF813511F488E018D01DD067\n"
      ":107F900080E101D065CF982F8091C00085FFFCCF94\n"
      ":107FA0009093C60008958091C00087FFFCCF809118\n"
      ":107FB000C00084FD01C0A8958091C6000895E0E648\n"
      ":107FC000F0E098E1908380830895EDDF803219F02E\n"
      ":107FD00088E0F5DFFFCF84E1DECF1F93182FE3DFCA\n"
      ":107FE0001150E9F7F2DF1F91089580E0E8DFEE27F6\n"
      ":047FF000FF270994CA\n"
      ":027FFE00040479\n"
      ":0400000300007E007B\n"
      ":00000001FF\n"
  }
};

image_t PROGMEM image_168 = {
  {
    "optiboot_atmega168.hex"    }
  ,
  {
    "atmega168"    }
  ,
  0x9406,				/* Signature bytes for 168 */
  {
    0x3F, 0xC6, 0xDD, 0x04    }
  ,
  {
    0x2F, 0,0,0,0    }
  ,
  128,
  {
    ":103E0000112484B714BE81FFF0D085E08093810037\n"
      ":103E100082E08093C00088E18093C10086E08093B7\n"
      ":103E2000C20080E18093C4008EE0C9D0259A86E06C\n"
      ":103E300020E33CEF91E0309385002093840096BB13\n"
      ":103E4000B09BFECF1D9AA8958150A9F7CC24DD2404\n"
      ":103E500088248394B5E0AB2EA1E19A2EF3E0BF2E27\n"
      ":103E6000A2D0813461F49FD0082FAFD0023811F076\n"
      ":103E7000013811F484E001C083E08DD089C0823420\n"
      ":103E800011F484E103C0853419F485E0A6D080C024\n"
      ":103E9000853579F488D0E82EFF2485D0082F10E0EE\n"
      ":103EA000102F00270E291F29000F111F8ED0680127\n"
      ":103EB0006FC0863521F484E090D080E0DECF843678\n"
      ":103EC00009F040C070D06FD0082F6DD080E0C816C8\n"
      ":103ED00088E3D80618F4F601B7BEE895C0E0D1E053\n"
      ":103EE00062D089930C17E1F7F0E0CF16F8E3DF0614\n"
      ":103EF00018F0F601B7BEE89568D007B600FCFDCF14\n"
      ":103F0000A601A0E0B1E02C9130E011968C911197C0\n"
      ":103F100090E0982F8827822B932B1296FA010C01A0\n"
      ":103F200087BEE89511244E5F5F4FF1E0A038BF07D0\n"
      ":103F300051F7F601A7BEE89507B600FCFDCF97BE86\n"
      ":103F4000E89526C08437B1F42ED02DD0F82E2BD092\n"
      ":103F50003CD0F601EF2C8F010F5F1F4F84911BD0D7\n"
      ":103F6000EA94F801C1F70894C11CD11CFA94CF0C53\n"
      ":103F7000D11C0EC0853739F428D08EE10CD084E9ED\n"
      ":103F80000AD086E07ACF813511F488E018D01DD0B0\n"
      ":103F900080E101D065CF982F8091C00085FFFCCFD4\n"
      ":103FA0009093C60008958091C00087FFFCCF809158\n"
      ":103FB000C00084FD01C0A8958091C6000895E0E688\n"
      ":103FC000F0E098E1908380830895EDDF803219F06E\n"
      ":103FD00088E0F5DFFFCF84E1DECF1F93182FE3DF0A\n"
      ":103FE0001150E9F7F2DF1F91089580E0E8DFEE2736\n"
      ":043FF000FF2709940A\n"
      ":023FFE000404B9\n"
      ":0400000300003E00BB\n"
      ":00000001FF\n"
  }
};

image_t PROGMEM image_8 = {
  {
    "optiboot_atmega8.hex"    }
  ,
  {
    "atmega8"    }
  ,
  0x9307,				/* Signature bytes for 8 */
  {
    0x3F, 0xBF, 0xCC, 0, 0    }
  ,
  {
    0x2F, 0xBF, 0xCC, 0, 0    }
  ,
  64,
  {
    ":101E000011248FE594E09EBF8DBF84B714BE81FF7F\n"
      ":101E1000E2D085E08EBD82E08BB988E18AB986E8A0\n"
      ":101E200080BD80E189B98EE0C2D0BD9A96E020E302\n"
      ":101E30003CEF54E040E23DBD2CBD58BF08B602FE69\n"
      ":101E4000FDCF88B3842788BBA8959150A1F7CC24F7\n"
      ":101E5000DD2488248394B5E0AB2EA1E19A2EF3E033\n"
      ":101E6000BF2E9ED0813461F49BD0082FA4D00238BD\n"
      ":101E700011F0013811F484E001C083E08DD089C0F5\n"
      ":101E8000823411F484E103C0853419F485E09BD0D9\n"
      ":101E900080C0853579F484D0E82EFF2481D0082FC6\n"
      ":101EA00010E0102F00270E291F29000F111F83D0CB\n"
      ":101EB00068016FC0863521F484E085D080E0DECFF4\n"
      ":101EC000843609F040C06CD06BD0082F69D080E018\n"
      ":101ED000C81688E1D80618F4F601B7BEE895C0E048\n"
      ":101EE000D1E05ED089930C17E1F7F0E0CF16F8E16E\n"
      ":101EF000DF0618F0F601B7BEE8955DD007B600FC26\n"
      ":101F0000FDCFA601A0E0B1E02C9130E011968C91BC\n"
      ":101F1000119790E0982F8827822B932B1296FA0125\n"
      ":101F20000C0187BEE89511244E5F5F4FF1E0A034AD\n"
      ":101F3000BF0751F7F601A7BEE89507B600FCFDCF35\n"
      ":101F400097BEE89526C08437B1F42AD029D0F82E60\n"
      ":101F500027D031D0F601EF2C8F010F5F1F4F8491F6\n"
      ":101F60001BD0EA94F801C1F70894C11CD11CFA9463\n"
      ":101F7000CF0CD11C0EC0853739F41DD08EE10CD0AA\n"
      ":101F800083E90AD087E07ACF813511F488E00FD059\n"
      ":101F900012D080E101D065CF5D9BFECF8CB9089552\n"
      ":101FA0005F9BFECF5C9901C0A8958CB1089598E124\n"
      ":101FB00091BD81BD0895F4DF803219F088E0F7DF2C\n"
      ":101FC000FFCF84E1E9CF1F93182FEADF1150E9F723\n"
      ":101FD000F2DF1F91089580E0EADFEE27FF270994E2\n"
      ":021FFE000404D9\n"
      ":0400000300001E00DB\n"
      ":00000001FF\n"
  }
};


