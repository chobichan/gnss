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
#include  <mt3339.h>

/* ----------------------------------------
    defines
---------------------------------------- */
#define  GPS_TXD    2  // output setup
#define  GPS_RXD   12  // input setup
#define  FIVE_VOLT 15  // output setup active high

/* ----------------------------------------
    prototypes
---------------------------------------- */

/* ----------------------------------------
    instances or global variables
---------------------------------------- */
GNSS_BY_SOFT gps;
MT3339 mt;
uint32_t baseTim;

/* ----------------------------------------
   GPIO initialize.
  ---------------------------------------- */
void gpioInit()
{
  digitalWrite( GPS_TXD, HIGH );  /* active high */
  pinMode( GPS_TXD, OUTPUT );
  pinMode( GPS_RXD, INPUT );
  pinMode( FIVE_VOLT, OUTPUT );
  digitalWrite( FIVE_VOLT, HIGH );  /* active high */

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
  gps.begin( 38400UL, GPS_RXD, GPS_TXD );  // original 9600UL
  delay( 20UL );
//  gps.monitor();

  /* PMTK_SET_NMEA_BAUDRATE */
  gps.print( mt.pmtk_set_baudrate( 38400UL ) );
  Serial.print( ">> " + mt.pmtk_set_baudrate( 38400UL ) );
  /* PMTK_API_SET_FIX_CTL */
  gps.print( mt.pmtk_api_set_fix_ctl( 1000UL ) );
  Serial.print( ">> " + mt.pmtk_api_set_fix_ctl( 1000UL ) );
  /* gll,rmc,vtg,gga,gsa,gsv,zda */
  gps.print( mt.pmtk_api_set_nmea_output(0,1,0,0,0,0,0) );
  Serial.print( ">> " + mt.pmtk_api_set_nmea_output(0,1,0,0,0,0,0) );
  /* PMTK_API_SET_RTC_TIME */
  gps.print( mt.pmtk_api_set_rtc_time( 1595222678UL ) );
  Serial.print( ">> " + mt.pmtk_api_set_rtc_time( 1595222678UL ) );
  /* PMTK_API_SET_SUPPORT_QZSS_NMEA */
  gps.print( mt.pmtk_api_set_support_qzss_nmea( true ) );
  Serial.print( ">> " + mt.pmtk_api_set_support_qzss_nmea( true ) );
  /* PMTK_API_SET_STATIC_NAV_THD */
  gps.print( mt.pmtk_api_set_static_nav_thd( 2.0F ) );
  Serial.print( ">> " + mt.pmtk_api_set_static_nav_thd( 2.0F ) );

  baseTim = millis();
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
      int cmd;
      uint8_t result;
      if( (cmd = mt.pmtk_ack( (const char *)buffer, &result )) >= 0 )
      {
        Serial.println( "PMTK_ACK " + (String)cmd + " " + (String)mt.ackMessage( result ) );
      }
      else
      {
        Serial.println( buffer );
      }
    }
  }

  if( (millis() - baseTim) >= 1000UL )
  {
    baseTim = millis();

//    gps.print( mt.pmtk_test() );
//    Serial.print( ">> " + mt.pmtk_test() );

//    gps.print( mt.pmtk_sys_msg( 1 ) );
//    Serial.print( ">> " + mt.pmtk_sys_msg( 1 ) );

//    gps.print( mt.pmtk_hot_start() );
//    Serial.print( ">> " + mt.pmtk_hot_start() );

//    gps.print( mt.pmtk_warm_start() );
//    Serial.print( ">> " + mt.pmtk_warm_start() );

//    gps.print( mt.pmtk_set_baudrate( 9600UL ) );
//    Serial.print( ">> " + mt.pmtk_set_baudrate( 9600UL ) );

//    gps.print( mt.pmtk_api_set_fix_ctl( 100UL ) );
//    Serial.print( ">> " + mt.pmtk_api_set_fix_ctl( 100UL ) );

    /* gll,rmc,vtg,gga,gsa,gsv,zda */
//    gps.print( mt.pmtk_api_set_nmea_output(0,1,0,0,0,0,0) );  // 0,0,0,1,0,0,0
//    Serial.print( ">> " + mt.pmtk_api_set_nmea_output(0,1,0,0,0,0,0) );
  }
}
