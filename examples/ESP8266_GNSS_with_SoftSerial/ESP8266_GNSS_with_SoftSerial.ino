/* ----------------------------------------
  NMEA0183 GNSS example.
  for ESP8266 and any more.

  Copyright (c) 2020 hamayan (hamayan.contact@gmail.com).
  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Created 2020 by hamayan (hamayan.contact@gmail.com)
---------------------------------------- */
#include  <gnss.h>

/* ----------------------------------------
    defines
---------------------------------------- */
//#define  ASSOC      0  // output setup active low
//#define  LED    ASSOC
//#define  GPS_1PPS  13  // input setup active ???
#define  GPS_TXD    2  // output setup
#define  GPS_RXD   12  // input setup
#define  FIVE_VOLT 15  // output setup active high
//#define  RELAY     14  // output setup active high
//#define  SDA33      4  // tri state
//#define  SCL33      5  // tri state
//#define  WAKE_UP   16  // output setup active low

/* ----------------------------------------
    prototypes
---------------------------------------- */

/* ----------------------------------------
    instances or global variables
---------------------------------------- */
GNSS_BY_SOFT gps;

/* ----------------------------------------
   GPIO initialize.
  ---------------------------------------- */
void gpioInit()
{
//  pinMode( GPS_1PPS, INPUT_PULLUP );
//  pinMode( ASSOC, OUTPUT );
//  digitalWrite( ASSOC, HIGH );  /* active low */

  digitalWrite( GPS_TXD, HIGH );  /* active high */
  pinMode( GPS_TXD, OUTPUT );
  pinMode( GPS_RXD, INPUT );
  pinMode( FIVE_VOLT, OUTPUT );
  digitalWrite( FIVE_VOLT, HIGH );  /* active high */
//  pinMode( RELAY, OUTPUT );
//  digitalWrite( RELAY, HIGH );  /* active high */

  /* LED blink start */
//  assocTick.attach( 0.125, ledBlink );  /* attach blink led handler. */
//  ledBlinkPattern = 0xAAAAAAAA;  // 0b1010 1010 1010 1010 1010 1010 1010 1010

  delay( 1000UL );
}

/* ----------------------------------------
   setup.
  ---------------------------------------- */
void setup()
{
  /* gpio initialize */
  gpioInit();

  Serial.begin( 115200UL );
  delay( 20UL );
  Serial.println( "\r\nESP8266 GNSS with Soft Serial." );
  gps.begin( 9600UL, GPS_RXD, GPS_TXD );
  delay( 20UL );
//  gps.monitor();
}

/* ----------------------------------------
   loop.
  ---------------------------------------- */
void loop()
{
  char buffer[ 128 ];
  int rcv = gps.line( buffer, sizeof(buffer) );
  if( rcv )
  {
    int type;
    GPS_GPGGA gga;
    GPS_GPGLL gll;
    GPS_GPGNS gns;
    GPS_GPGSA gsa;
    GPS_GPGSV gsv;
    GPS_GPRMC rmc;
    GPS_GPVTG vtg;
    GPS_GPZDA zda;

    if( gps.gpgga( (const char *)buffer, &gga ) > 0 )
    {
      Serial.print( "GPGGA:" );
      Serial.print( " utc = " + (String)gga.hhmmss );
      Serial.printf( " lat = %f", gga.latitude );
      Serial.printf( " lon = %f", gga.longitude );
      Serial.print( " ele = " + (String)gga.altitude );
      Serial.print( " sat = " + (String)gga.satellites );
      Serial.print( " hdop = " + (String)gga.hdop );
      Serial.println();
    }
    else if( gps.gpgll( (const char *)buffer, &gll ) > 0 )
    {
      Serial.print( "GNGLL:" );
      Serial.printf( " lat = %f", gll.latitude );
      Serial.printf( " lon = %f", gll.longitude );
      Serial.print( " utc = " + (String)gll.hhmmss );
      Serial.printf( " sta = %c", gll.pStatus );
      Serial.printf( " mod = %c", gll.pMode );
      Serial.println();
    }
    else if( gps.gpgns( (const char *)buffer, &gns ) > 0 )
    {
      Serial.print( "GNGNS:" );
      Serial.print( " utc = " + (String)gns.hhmmss );
      Serial.printf( " lat = %f", gns.latitude );
      Serial.printf( " lon = %f", gns.longitude );
      Serial.print( " sat = " + (String)gns.satellites );
      Serial.print( " hdop = " + (String)gns.hdop );
      Serial.print( " ele = " + (String)gns.altitude );
      Serial.println();
    }
    else if( gps.gpgsa( (const char *)buffer, &gsa ) > 0 )
    {
      Serial.print( "GPGSA: " );
      for( int i = 0; i < 12; i++ )
      {
        Serial.print( (String)gsa.sNumber[i] + "," );
      }
      Serial.print( " PDOP = " + (String)gsa.pdop );
      Serial.print( " HDOP = " + (String)gsa.hdop );
      Serial.print( " VDOP = " + (String)gsa.vdop );
      Serial.println();
    }
    else if( gps.gpgsv( (const char *)buffer, &gsv, &type ) > 0 )
    {
      Serial.print( (type == TYPE_GPS) ? "GPGSV: " : (type == TYPE_QZSS) ? "QZGSV: " : "GLGSV: " );
      Serial.print( (String)gsv.seq + "/" + (String)gsv.total );
      Serial.print( " view = " + (String)gsv.inField + " " );
      for( int i = 0; i < 4; i++ )
      {
        Serial.print( (String)gsv.info[i].number + "," );
        Serial.print( (String)gsv.info[i].elevation + "," );
        Serial.print( (String)gsv.info[i].azimuth + "," );
        Serial.print( (String)gsv.info[i].snr + " " );
      }
      Serial.println();
    }
    else if( gps.gprmc( (const char *)buffer, &rmc ) > 0 )
    {
      Serial.print( "GNRMC:" );
      Serial.print( " utc = " + (String)rmc.hhmmss );
      Serial.printf( " lat = %f", rmc.latitude );
      Serial.printf( " lon = %f", rmc.longitude );
      Serial.print( " km/h = " + (String)(rmc.knot * 1.852F) );
      Serial.print( " dir = " + (String)rmc.azimuth );
      Serial.println();
    }
    else if( gps.gpvtg( (const char *)buffer, &vtg ) > 0 )
    {
      Serial.print( "GNVTG:" );
      Serial.print( " dir = " + (String)vtg.azimuth );
      Serial.print( " ori = " + (String)vtg.orientation );
      Serial.print( " kmh = " + (String)vtg.kmh );
      Serial.println();
    }
    else if( gps.gpzda( (const char *)buffer, &zda ) > 0 )
    {
      Serial.print( "GPZDA:" );
      Serial.print( " utc = " + (String)zda.hhmmss + " " );
      Serial.print( (String)zda.year + "/" + (String)zda.month + "/" + (String)zda.day );
      Serial.println();
    }
    else
    {
      Serial.println( buffer );
    }
  }
}
