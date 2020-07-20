/* ----------------------------------------
  MT3339 GPS reciever code
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
#include  "mt3339.h"

extern "C" {
}

/* ----------------------------------------
    defines
---------------------------------------- */

/* ----------------------------------------
    prototypes
---------------------------------------- */

/* ----------------------------------------
    instances or global variables
---------------------------------------- */
static const char talkerID[] = "$PMTK";
static const char *result_Message[] = 
{
  "Invalid command / packet.",
  "Unsupported command / packet.",
  "Valid command / packet, but action failed.",
  "Valid command / packet, and action succeeded.",
};

/* ----------------------------------------
    constructor destructor
---------------------------------------- */
MT3339::MT3339()
{
}

MT3339::~MT3339()
{
}

/* ----------------------------------------
    check sum
---------------------------------------- */
uint8_t MT3339::checkSum( const uint8_t *src, int size )
{
  uint8_t sum = 0;
  for( int i = 0; i < size; i++ )
  {
    sum ^= *src++;
  }
  return sum;
}

/* ----------------------------------------
    ack result messages
---------------------------------------- */
const char *MT3339::ackMessage( int index )
{
  return result_Message[ index ];
}

/* ----------------------------------------
    pmtk end
    Perform the second half of the process.
---------------------------------------- */
String MT3339::pmtk_end( String str )
{
  const char *ptr = str.c_str();
  uint8_t sum = checkSum( (const uint8_t *)(ptr + 1), str.length() - 1 );
  String s = "*";
  s += binToHexString( sum );
  s += "\r\n";

  return s;
}

/* ----------------------------------------
    pmtk ack
---------------------------------------- */
int MT3339::pmtk_ack( const char *ack, uint8_t *result )
{
  if( strncmp( ack, "$PMTK001", sizeof("$PMTK001") - 1 ) != 0 ) return (-1);

//  Serial.println( ack );
  int argc;
  char *argv[3];
  char buffer[ strlen(ack) + 1 ];
  strcpy( buffer, ack );

  /* Isolate Checksums */
  char checkSumStr[3];
  argc = split( '*', buffer, argv, sizeof(argv) / sizeof(argv[0]) );
  if( argc == 2 )
  {
    strncpy( checkSumStr, argv[1], 2 );
    checkSumStr[2] = '\0';
  }
  else return (-1);
  uint8_t rcvSum = (uint8_t)HexStringToBin( checkSumStr );

  int len = strlen( buffer );
  uint8_t sum = checkSum( (const uint8_t *)(buffer + 1), len - 1 );
  if( rcvSum != sum )
  {
//    Serial.printf( "pmtk_ack : checksum error. %02x %02x\r\n", sum , rcvSum );
    return (-1);
  }

  /* Split the string by commas. */
  argc = split( ',', buffer, argv, sizeof(argv) / sizeof(argv[0]) );
//  Serial.println( "argc=" + (String)argc );

  int cmd = IntStringToBin( (const char *)argv[1], 3 );
  char res = *argv[2];
  *result = (res == '3') ? PMTK_VALID_AND_SUCCEEDED :
            (res == '2') ? PMTK_VALID_BUT_FAILED : 
            (res == '1') ? PMTK_UNSUPPORTED : PMTK_INVALID;

  return cmd;
}


/* ----------------------------------------
    pmtk test
    Test Packet.

    None
---------------------------------------- */
String MT3339::pmtk_test()
{
  String str = talkerID;
  str += TYPE_PMTK_TEST;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk system message
    Output system message. 

    Msg: The system message. 
      ‘0’ = UNKNOWN 
      ‘1’ = STARTUP 
      ‘2’ = Notification : Notfication for the host aiding EPO. 
      ‘3’ = Notification: Notification for the transition to Normal mode is   
             successfully done   
---------------------------------------- */
String MT3339::pmtk_sys_msg( int type )
{
  String str = talkerID;
  str += TYPE_PMTK_SYS_MSG;
  str += (type == 0) ? ",000" : ",001";  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk hot start
    Hot Restart.   
    Use all available data in the NV Store. 

    None
---------------------------------------- */
String MT3339::pmtk_hot_start()
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_HOT_START;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk warm start
    Warm Restart.
    Don’t use Ephemeris at re-start. 

    None
---------------------------------------- */
String MT3339::pmtk_warm_start()
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_WARM_START;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk cold start
    Cold Restart.   
    Don’t use Time, Position, Almanacs and Ephemeris data at re-start. 

    None
---------------------------------------- */
String MT3339::pmtk_cold_start()
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_COLD_START;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk full cold start
    Full Cold Restart.
    It’s essentially a Cold Restart,
     but additionally clear system/user configurations at re-start.
    That is, reset the receiver to the factory status. 

    None
---------------------------------------- */
String MT3339::pmtk_full_cold_start()
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_FULL_COLD_START;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk clear flash aid
    Erase EPO data stored in the flash memory.

    None 
---------------------------------------- */
String MT3339::pmtk_clear_flash_aid()
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_CLEAR_FLASH_AID;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk clear epo
---------------------------------------- */
String MT3339::pmtk_clear_epo()
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_CLEAR_EPO;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk standby mode
    Enter standby mode for power saving. 

      ‘0’ = Stop mode 
      ‘1’ = Sleep mode 
---------------------------------------- */
String MT3339::pmtk_standby_mode( int type )
{
  String str = talkerID;
  str += TYPE_PMTK_CMD_STANDBY_MODE;
  str += (type == 0) ? ",0" : ",1";  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk set periodic mode
    Periodic Power Saving Mode Setting.  

    Type: Set operation mode of power saving. 
      ‘0’ = Back to normal mode. 
      ‘1’ = Periodic backup Mode. 
      ‘2’ = Periodic standby Mode. 
      ‘8’ = AlwaysLocate standby Mode. 
      ‘9’ = AlwaysLocate backup Mode. 

    Run Time: Duration [msec] to fix for (or attempt to fix for)
     before switching from running mode back to minimum power sleep mode. 
      ‘0’ = Disable 
      >= ‘1000’ = Enable  [Range: 1000～518400000]

    Sleep Time: Interval [msec] to come out of a minimum power sleep mode
     and start running in order to get a new position fix. 
      [Range: 1000～518400000] 

    Second Run Time: Duration [msec] to fix for (or attempt to fix for)
     before switching from running mode back to minimum power sleep mode. 
      ‘0’ = Disable 
      >= ‘1000’ = Enable  [Range: 1000～518400000] 

    Second Sleep Time: Interval [msec] to come out of a minimum power sleep mode
     and start running in order to get a new position fix. 
      [Range: 1000～518400000] 

    *Note the Second run time should larger than First run time when non-zero value.

    Example: How to enter Periodic modes. 
      Periodic Backup mode 
        $PMTK225,0*2B<CR><LF> 
        $PMTK225,1,25,180000,60000*0E<CR><LF> 
        $PMTK225,1,3000,12000,18000,72000*16<CR><LF> 
      Periodic Standby mode 
        $PMTK225,0*2B<CR><LF> 
        $PMTK225,2,25,180000,60000*0D<CR><LF> 
        $PMTK225,2,3000,12000,18000,72000*15<CR><LF> 

    How to enter AlwaysLocate modes. 
      AlwaysLocate Standby 
        $PMTK225,0*2B<CR><LF> 
        $PMTK225,8*23<CR><LF> 
      AlwaysLocate Backup 
        $PMTK225,0*2B<CR><LF> 
        $PMTK225,9*22<CR><LF>
---------------------------------------- */
String MT3339::pmtk_set_periodic_mode( int type )
{
  String str = talkerID;
  str += TYPE_PMTK_SET_PERIODIC_MODE;
  str += (type == 1) ? ",1" :  // data field
         (type == 2) ? ",2" :
         (type == 8) ? ",8" :
         (type == 9) ? ",9" : ",0";
  str += pmtk_end( str );

  return str;
}

String MT3339::pmtk_set_periodic_mode(
  int type,
  uint32_t runTime, uint32_t sleepTime,
  uint32_t secondRunTime )
{
  String str = talkerID;
  str += TYPE_PMTK_SET_PERIODIC_MODE;
  str += (type == 1) ? ",1" :  // data field
         (type == 2) ? ",2" :
         (type == 8) ? ",8" :
         (type == 9) ? ",9" : ",0";
  str += "," + (String)runTime;
  str += "," + (String)sleepTime;
  str += "," + (String)secondRunTime;
  str += pmtk_end( str );

  return str;
}

String MT3339::pmtk_set_periodic_mode(
  int type,
  uint32_t runTime, uint32_t sleepTime,
  uint32_t secondRunTime, uint32_t secondSleepTime )
{
  String str = talkerID;
  str += TYPE_PMTK_SET_PERIODIC_MODE;
  str += (type == 1) ? ",1" :  // data field
         (type == 2) ? ",2" :
         (type == 8) ? ",8" :
         (type == 9) ? ",9" : ",0";
  str += "," + (String)runTime;
  str += "," + (String)sleepTime;
  str += "," + (String)secondRunTime;
  str += "," + (String)secondSleepTime;
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk set nmea baudrate

    Baud rate 
      ‘0’ = default setting 
      “4800” = 4800bps 
      “9600” = 9600bps 
      “14400” = 14400bps 
      “19200” = 19200bps 
      “38400” = 38400bps 
      “57600” = 57600bps 
      “115200” = 115200bps
---------------------------------------- */
String MT3339::pmtk_set_baudrate( uint32_t brr )
{
  String str = talkerID;
  str += TYPE_PMTK_SET_NMEA_BAUDRATE;
  str += "," + (String)brr;  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set fix control
    This command controls the rate of position fixing activity. 

    Fixinterval: Position fix interval [msec]. Must be larger than 100. 
      0,0,0,0 
---------------------------------------- */
String MT3339::pmtk_api_set_fix_ctl( uint32_t period )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_FIX_CTL;
  str += "," + (String)period + ",0,0,0,0";  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set nmea output
    Set NMEA sentence output frequencies. 
    There are totally 19 data fields that present output 
    frequencies for the 19 supported NMEA sentences individually. 

    Supported Frequency Setting 
     0 Disabled or not supported sentence. 
     1 Output once every one position fix. 
     2 Output once every two position fixes. 
     3 Output once every three position fixes. 
     4 Output once every four position fixes. 
     5 Output once every five position fixes. 
---------------------------------------- */
String MT3339::pmtk_api_set_nmea_output(
  uint8_t gll, uint8_t rmc, uint8_t vtg, uint8_t gga,
  uint8_t gsa, uint8_t gsv, uint8_t zda )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_NMEA_OUTPUT;
  str += "," + (String)gll;  // data field 0
  str += "," + (String)rmc;  // data field 1
  str += "," + (String)vtg;  // data field 2
  str += "," + (String)gga;  // data field 3
  str += "," + (String)gsa;  // data field 4
  str += "," + (String)gsv;  // data field 5
  str += ",0,0,0,0,0,0,0,0,0,0,0";  // data field 6 7 8 9 10 11 12 13 14 15 16
  str += "," + (String)zda + ",0";  // data field 17 and 18
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set rtc time
    This command set RTC UTC time. To be noted,
    the command doesn’t update the GPS time
     which maintained by GPS receiver.
    After setting, the RTC UTC time finally may be updated
     by GPS receiver with more accurate time after 60 seconds

    Year: XXXX 
    Month: 1 ~ 12 
    Day:   1 ~ 31 
    Hour:  0 ~ 23 
    Min:   0 ~ 59 
    Sec:   0 ~ 59
---------------------------------------- */
String MT3339::pmtk_api_set_rtc_time( time_t utcTime )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_RTC_TIME;
  struct tm *t = gmtime( (const time_t *)&utcTime );
  // ex. ",2020,7,16,14,30,0"
  str += "," + (String)(t->tm_year + 1900);  // data field year
  str += "," + (String)(t->tm_mon + 1);  // data field month
  str += "," + (String)t->tm_mday;  // data field day
  str += "," + (String)t->tm_hour;  // data field hour
  str += "," + (String)t->tm_min;  // data field minute
  str += "," + (String)t->tm_sec;  // data field second
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set support qzss nmea
    The receiver support new NMEA format for QZSS.
    The command allow user enable or disable QZSS NMEA format.   
    (Default is disable QZSS NMEA format (use NMEA 0183 V3.01) 

    ‘0’: Disable 
    ‘1’: Enable 
---------------------------------------- */
String MT3339::pmtk_api_set_support_qzss_nmea( bool onOff )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_SUPPORT_QZSS_NMEA;
  str += (onOff) ? ",1" : ",0";  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set stop qzss
    Since QZSS is regional positioning service.
    The command allow user enable or disable QZSS function.
    Default is enable QZSS function.

    ‘0’: Disable 
    ‘1’: Enable 
---------------------------------------- */
String MT3339::pmtk_api_set_stop_qzss( bool onOff )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_STOP_QZSS;
  str += (onOff) ? ",0" : ",1";  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set datum
    Set default datum. 

    Datum: 
      ‘0’ = WGS84
      ‘1’ = TOKYO-M 
      ‘2’ = TOKYO-A 
---------------------------------------- */
String MT3339::pmtk_api_set_datum( int datum )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_DATUM;
  str += "," + (String)datum;  // data field
  str += pmtk_end( str );

  return str;
}

/* ----------------------------------------
    pmtk api set static nav thd
    Set the speed threshold for static navigation.
    If the actual speed is below the threshold,
     output position will keep the same and output speed will be zero.
    If threshold value is set to 0, this function is disabled. 

    PMTK386,speed_threshold 
      Speed_threshold: 0~2m/sec 
      ‘0’ = disable 
      >’0’ = speed threshold in m/s.(The minimum is 0.1m/sec,the max is 2.0m/sec) 
---------------------------------------- */
String MT3339::pmtk_api_set_static_nav_thd( float speed )
{
  String str = talkerID;
  str += TYPE_PMTK_API_SET_STATIC_NAV_THD;
  str += "," + (String)speed;  // data field
  str += pmtk_end( str );

  return str;
}
