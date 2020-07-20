/* ----------------------------------------
  MT3339 GPS reciever header
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
#ifndef _mt3339_h_
#define _mt3339_h_

#include  <Arduino.h>
//#include  <SoftwareSerial.h>
#include  <stringUtility.h>

extern "C"
{
  #include  <time.h>
}

/* ----------------------------------------
    defines
---------------------------------------- */
#define  TYPE_PMTK_TEST                       "000"
#define  TYPE_PMTK_ACK                        "001"
#define  TYPE_PMTK_SYS_MSG                    "010"
//#define  TYPE_PMTK_TXT_MSG                    "011"  //
#define  TYPE_PMTK_CMD_HOT_START              "101"
#define  TYPE_PMTK_CMD_WARM_START             "102"
#define  TYPE_PMTK_CMD_COLD_START             "103"
#define  TYPE_PMTK_CMD_FULL_COLD_START        "104"
#define  TYPE_PMTK_CMD_CLEAR_FLASH_AID        "120"
#define  TYPE_PMTK_CMD_CLEAR_EPO              "127"
#define  TYPE_PMTK_CMD_STANDBY_MODE           "161"
#define  TYPE_PMTK_LOCUS_QUERY_STATUS         "183"
#define  TYPE_PMTK_LOCUS_ERASE_FLASH          "184"
#define  TYPE_PMTK_LOCUS_STOP_LOGER           "185"
#define  TYPE_PMTK_LOCUS_LOG_NOW              "186"
#define  TYPE_PMTK_SET_AL_DEE_CFG             "223"
#define  TYPE_PMTK_SET_PERIODIC_MODE          "225"
#define  TYPE_PMTK_SET_NMEA_BAUDRATE          "251"
#define  TYPE_PMTK_API_SET_FIX_CTL            "300"
#define  TYPE_PMTK_API_SET_DGPS_MODE          "301"
#define  TYPE_PMTK_API_SET_SBAS_ENABLED       "313"
#define  TYPE_PMTK_API_SET_NMEA_OUTPUT        "314"
#define  TYPE_PMTK_API_SET_DATUM              "330"
#define  TYPE_PMTK_API_SET_DATUM_ADVANCE      "331"
#define  TYPE_PMTK_API_SET_RTC_TIME           "335"
#define  TYPE_PMTK_API_SET_SUPPORT_QZSS_NMEA  "351"
#define  TYPE_PMTK_API_SET_STOP_QZSS          "352"
#define  TYPE_PMTK_API_SET_STATIC_NAV_THD     "386"
#define  TYPE_PMTK_API_SET_TCXO_DEBUG         "389"
#define  TYPE_PMTK_Q_RELEASE                  "605"
#define  TYPE_PMTK_Q_EPO_INFO                 "607"
#define  TYPE_PMTK_Q_LOCUS_DATA               "622"
#define  TYPE_PMTK_Q_AVAILABLE_SV_EPH         "660"
#define  TYPE_PMTK_Q_AVAILABLE_SV_ALM         "661"
#define  TYPE_PMTK_DT_RELEASE                 "705"
#define  TYPE_PMTK_DT_UTC                     "740"
#define  TYPE_PMTK_DT_POS                     "741"
#define  TYPE_PMTK_EASY_ENABLE                "869"


typedef struct
{
  char  preamble;  // $
  char  talkerID;  // "PMTK"
  char  type[3];   // packet type 3byte
  char  data[1];   // The Data Field has variable length depending on the packet type.
  char  asterisk;  // *
  char  chk[2];    // 2 bytes character string. CHK1 and CHK2 are the checksum of data between Preamble and �e*�f.
  char  delimiter[2];  // CR.LF
} PMTK_BASIC_FORMAT;

typedef struct
{
  char  preamble;  // $
  char  talkerID;  // "PMTK"
  char  type[3];   // packet type 3byte
  char  asterisk;  // *
  char  chk[2];    // 2 bytes character string. CHK1 and CHK2 are the checksum of data between Preamble and �e*�f.
  char  delimiter[2];  // CR.LF
} PMTK_TEST;
#define  TYPE_PMTK_TEST                       "000"

enum PMTK_ACK_RESULT { PMTK_INVALID, PMTK_UNSUPPORTED, PMTK_VALID_BUT_FAILED, PMTK_VALID_AND_SUCCEEDED };
enum PMTK_CMD_STANDBY_MODE { STOP_MODE, SLEEP_MODE };
enum PMTK_PERIODIC_MODE { BACK_TO_NORMAL, PERIODIC_BACKUP, PERIODIC_STANDBY, ALWAYS_LOCATE_STNDBY = 8, ALWAYS_LOCATE_BACKUP = 9,};
enum PMTK_API_DATUM { WGS84, TOKYO_M, TOKYO_A, };


/* ----------------------------------------
    prototypes
---------------------------------------- */

/* ----------------------------------------
    instances or global variables
---------------------------------------- */

/* ----------------------------------------
    class
---------------------------------------- */
class MT3339
{
public:
  MT3339();
  ~MT3339();

  const char *ackMessage( int index );
  int  pmtk_ack( const char *ack, uint8_t *result );
  String pmtk_test();
  String pmtk_sys_msg( int type );
  String pmtk_hot_start();
  String pmtk_warm_start();
  String pmtk_cold_start();
  String pmtk_full_cold_start();
  String pmtk_clear_flash_aid();
  String pmtk_clear_epo();
  String pmtk_standby_mode( int type );
  String pmtk_set_periodic_mode( int type );
  String pmtk_set_periodic_mode(
    int type,
    uint32_t runTime, uint32_t sleepTime,
    uint32_t secondRunTime );
  String pmtk_set_periodic_mode(
    int type,
    uint32_t runTime, uint32_t sleepTime,
    uint32_t secondRunTime, uint32_t secondSleepTime );
  String pmtk_set_baudrate( uint32_t brr );
  String pmtk_api_set_fix_ctl( uint32_t period );
  String pmtk_api_set_nmea_output(
    uint8_t gll, uint8_t rmc, uint8_t vtg, uint8_t gga,
    uint8_t gsa, uint8_t gsv, uint8_t zda );
  String pmtk_api_set_rtc_time( time_t utcTime );
  String pmtk_api_set_support_qzss_nmea( bool onOff );
  String pmtk_api_set_stop_qzss( bool onOff );
  String pmtk_api_set_datum( int datum );
  String pmtk_api_set_static_nav_thd( float speed );

private:
  uint8_t checkSum( const uint8_t *src, int size );
  String pmtk_end( String str );

};

#endif /* _mt3339_h_ */
