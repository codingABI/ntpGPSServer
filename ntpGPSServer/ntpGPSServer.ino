/*
 * Project: ntpGPSServer (Ding24)
 * 
 * Description:
 * ESP32 based Stratum 1 NTP server that gets his time from a GPS module 
 * and provides NTP via Wifi. The ESP32 can connect to your Wifi access point, when you set your 
 * connection credentials in secrets.h or act as an independent Wifi access point, if connection to 
 * your Wifi failed.
 * 
 * It is a very minimalistic NTP server (Version 3) without fancy stuff like 
 * authentication, control messages, cryptographic keys...
 *
 * License: CC0
 * Copyright (c) 2024 codingABI
 * For details see: License.txt
 *
 * Inspired by ESP32TimeServer, Copyright (c) 2023 Rob Latour, MIT License, https://github.com/roblatour/ESP32TimeServer/blob/main/Arduino/ESP32TimeServer
 *
 * Used external libraries from Arduino IDE Library Manager
 * - TinyGPS (by Mikal Hart, Version 13.0.0, GPL-3.0 license)
 *
 * Hardware:
 * - Microcontroller ESP32 LOLIN32 (other ESP32s should also works)
 * - GY-NEO6MV2 NEO-6M GPS-Module
 *
 * External references
 * - https://www.rfc-editor.org/rfc/rfc5905
 * - https://www.rfc-editor.org/rfc/rfc1305
 * - https://tickelton.gitlab.io/articles/ntp-timestamps/
 * - https://learn.microsoft.com/en-US/windows-server/networking/windows-time-service/how-the-windows-time-service-works
 *
 * History:
 * 20241001, Initial version
 */

#include "secrets.h"

#include <WiFi.h>
#include <TinyGPS.h>

// #define DEBUG_SERIALPRINT // Uncommenting this line will slows down packet processing and is useful only for debugging

// Pin definitions
#define LED_PIN 22 // Builtin, low active LED from ESP32 LOLIN32
#define GPSTX_PIN 33
#define GPSRX_PIN 32

// Baud for GPS
#define GPSBAUD 9600

// Wifi
#define WIFIMAXRETRIES 30
    
// Max milliseconds without a GPS time sync after that device time will be assumed as invalid
#define MAXGPSUNSYNCEDTIMEMS 60000

// NTP Port
#define NTP_PORT 123

// Number of seconds from 1st January 1900 (NTP start time) to start of Unix epoch
#define NTPEPOCHOFFSET 2208988800ULL

// Global variables
WiFiUDP g_udp; // UDP Server

// NTP receive buffer
#define NTP_PACKET_SIZE 48 // We only process NTP packets with a payload of 48 bytes (payload including Transmit Timestamp)
/*
 * From rfc5905:
 * ...
 *  0                   1                   2                   3
 *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |LI | VN  |Mode |    Stratum     |     Poll      |  Precision   |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                         Root Delay                            |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                         Root Dispersion                       |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *      |                          Reference ID                         |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * +                     Reference Timestamp (64)                  +
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * +                      Origin Timestamp (64)                    +
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * +                      Receive Timestamp (64)                   +
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * +                      Transmit Timestamp (64)                  +
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * .                                                               .
 * .                    Extension Field 1 (variable)               .
 * .                                                               .
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * .                                                               .
 * .                    Extension Field 2 (variable)               .
 * .                                                               .
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                          Key Identifier                       |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                                               |
 * |                            dgst (128)                         |
 * |                                                               |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * 
 * ...
 */
byte g_receivePacketBuffer[NTP_PACKET_SIZE];
signed char g_precision = 0;

// Serial for GPS
HardwareSerial g_SerialGPS(1);
// GPS
TinyGPS g_gps;
bool g_validTime = false; // true, when first time sync to GPS was successful

// Converts 8 consecutive bytes to an uint64_t value
uint64_t packetBytesToUINT64(byte* packet) {
  uint64_t result = 0;

  if (packet == NULL) return 0;
  for (int i=0;i<8;i++) {
    result += (uint64_t(*(packet+i)) << (56-i*8));
  }
  return result;
}

// Print NTP time as text to Serial
void SerialPrintNtpTime(uint64_t ntpTime) {
  #define MAXSTRDATALENGTH 29
  char strData[MAXSTRDATALENGTH+1];
  struct tm * ptrTimeinfo;

  time_t unixTime = (ntpTime >> 32) - NTPEPOCHOFFSET;
  unsigned long fraction = (ntpTime & 0xffffffff) / (double)(1ULL << 32) * 1000000000UL;

  ptrTimeinfo = gmtime ( &unixTime ); // Get UTC time
  snprintf(strData,MAXSTRDATALENGTH+1,"%02d.%02d.%04d %02d:%02d:%02d.%09lu",
    ptrTimeinfo->tm_mday,
    ptrTimeinfo->tm_mon + 1,
    ptrTimeinfo->tm_year + 1900,
    ptrTimeinfo->tm_hour,
    ptrTimeinfo->tm_min,
    ptrTimeinfo->tm_sec,
    fraction);
  Serial.println(strData);
}

  /*
   * Convertes a floating point value to NTP short format
   * 
   * From rfc5905:
   * ...
   *
   *  0                   1                   2                   3
   *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * |          Seconds              |           Fraction            |
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   *                          NTP Short Format
   */
uint32_t doubleToNtpShortFormat(double value) {
  uint16_t seconds = (uint16_t) value;
  uint16_t fraction = (double) (value - seconds) * 0xFFFF;
  return ((uint32_t) seconds << 16) | fraction;
}

// Get current time from ESP32 in 64 bit NTP format
// Based on https://tickelton.gitlab.io/articles/ntp-timestamps/
uint64_t getTimeOfDayInNtpFormat() {
  struct timeval tv;
  gettimeofday(&tv, NULL);

  return ((uint64_t) (tv.tv_sec + NTPEPOCHOFFSET) << 32)
    | ((uint32_t)((double) tv.tv_usec * (double)(1LL << 32) * 1.0e-6));
}

// Send NTP response
void sendNtpResponse(IPAddress clientIP, int clientPort)
{
  byte sendPacketBuffer[NTP_PACKET_SIZE];

  // Set the receive time to current time
  uint64_t receiveTime = getTimeOfDayInNtpFormat();

  /*
   * LI Leap Indicator (leap), VN Version Number (version), Mode (mode), Byte 0
   * ---
   * If first two LI Leap Indicator (leap) bits are set 0b11 = 3 = unknown (clock unsynchronized)
   * a NTP client refuses a time sync (For example on Windows 11: "
   * Packet test 6 failed (not synced or bad interval since last sync)")
   *
   * From rfc5905:
   * ...LI Leap Indicator (leap): 2-bit integer warning of an impending leap
   * second to be inserted or deleted in the last minute of the current
   * month with values defined in Figure 9.
   *
   * +-------+----------------------------------------+
   * | Value | Meaning                                |
   * +-------+----------------------------------------+
   * | 0     | no warning                             |
   * | 1     | last minute of the day has 61 seconds  |
   * | 2     | last minute of the day has 59 seconds  |
   * | 3     | unknown (clock unsynchronized)         |
   * +-------+----------------------------------------+
   *
   * VN Version Number (version): 3-bit integer representing the NTP
   * version number, currently 4.
   *
   * Mode (mode): 3-bit integer representing the mode, with values defined
   * in Figure 10.
   *
   * +-------+--------------------------+
   * | Value | Meaning                  |
   * +-------+--------------------------+
   * | 0     | reserved                 |
   * | 1     | symmetric active         |
   * | 2     | symmetric passive        |
   * | 3     | client                   |
   * | 4     | server                   |
   * | 5     | broadcast                |
   * | 6     | NTP control message      |
   * | 7     | reserved for private use |
   * +-------+--------------------------+
   * ...
   */

  // LI: 0 or 3, Version: 3, Mode: 4 (server)
  if (g_validTime) sendPacketBuffer[0] = 0b00011100; else sendPacketBuffer[0] = 0b11011100;

  /*
   * Stratum (stratum), Byte 1
   * ---
   * From rfc5905:
   * ...Stratum (stratum): 8-bit integer representing the stratum, with
   * values defined in Figure 11.
   *
   * +--------+-----------------------------------------------------+
   * | Value  | Meaning                                             |
   * +--------+-----------------------------------------------------+
   * | 0      | unspecified or invalid                              |
   * | 1      | primary server (e.g., equipped with a GPS receiver) |
   * | 2-15   | secondary server (via NTP)                          |
   * | 16     | unsynchronized                                      |
   * | 17-255 | reserved                                            |
   * +--------+-----------------------------------------------------+
   * ...
   */
  sendPacketBuffer[1] = 0b00000001;

  /*
   * Poll
   * ---
   * Time period between time syncs of a NTP client
   *
   * From rfc5905:
   * ...Poll: 8-bit signed integer representing the maximum interval between
   * successive messages, in log2 seconds.  Suggested default limits for
   * minimum and maximum poll intervals are 6 and 10, respectively...
   */
  sendPacketBuffer[2] = 10; // 2^10=1024 seconds (=~17 minutes)

  /*
   * Precision, Byte 3
   * ---
   *
   * From rfc5905:
   * ...Precision: 8-bit signed integer representing the precision of the
   * system clock, in log2 seconds.  For instance, a value of -18
   * corresponds to a precision of about one microsecond.  The precision
   * can be determined when the service first starts up as the minimum
   * time of several iterations to read the system clock...
   */
  sendPacketBuffer[3] = g_precision;

  /*
   * Root Delay (rootdelay), Byte 4-7
   * ---
   * From rfc5905:
   * ...Root Delay (rootdelay): Total round-trip delay to the reference
   * clock, in NTP short format...
   *
   *  0                   1                   2                   3
   *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * |          Seconds              |           Fraction            |
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   *                          NTP Short Format
   */
  uint32_t rootdelay = 0;
  for (int i=0;i<4;i++) sendPacketBuffer[4+i] = (int)((rootdelay >> (24-i*8)) & 0xFF);

  /*
   * Root Dispersion (rootdisp), Byte 8-11
   * ---
   * From rfc5905:
   * ...Root Dispersion (rootdisp): Total dispersion to the reference clock,
   * in NTP short format...
   *
   *  0                   1                   2                   3
   *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * |          Seconds              |           Fraction            |
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   *                          NTP Short Format
   */
  uint32_t rootdisp = doubleToNtpShortFormat(0.001221f);
  for (int i=0;i<4;i++) sendPacketBuffer[8+i] = (int)((rootdisp >> (24-i*8)) & 0xFF);

  /*
   * Reference ID (refid), Byte 12-15
   * ---
   * From rfc5905:
   * ...Reference ID (refid): 32-bit code identifying the particular server
   * or reference clock.  The interpretation depends on the value in the
   * stratum field.  For packet stratum 0 (unspecified or invalid), this
   * is a four-character ASCII [RFC1345] string, called the "kiss code",
   * used for debugging and monitoring purposes.  For stratum 1 (reference
   * clock), this is a four-octet, left-justified, zero-padded ASCII
   * string assigned to the reference clock.  The authoritative list of
   * Reference Identifiers is maintained by IANA; however, any string
   * beginning with the ASCII character "X" is reserved for unregistered
   * experimentation and development.  The identifiers in Figure 12 have
   * been used as ASCII identifiers:
   *
   * +------+----------------------------------------------------------+
   * | ID   | Clock Source                                             |
   * +------+----------------------------------------------------------+
   * | GOES | Geosynchronous Orbit Environment Satellite               |
   * | GPS  | Global Position System                                   |s
   * | GAL  | Galileo Positioning System                               |
   * | PPS  | Generic pulse-per-second                                 |
   * | IRIG | Inter-Range Instrumentation Group                        |
   * | WWVB | LF Radio WWVB Ft. Collins, CO 60 kHz                     |
   * | DCF  | LF Radio DCF77 Mainflingen, DE 77.5 kHz                  |
   * | HBG  | LF Radio HBG Prangins, HB 75 kHz                         |
   * | MSF  | LF Radio MSF Anthorn, UK 60 kHz                          |
   * | JJY  | LF Radio JJY Fukushima, JP 40 kHz, Saga, JP 60 kHz       |
   * | LORC | MF Radio LORAN C station, 100 kHz                        |
   * | TDF  | MF Radio Allouis, FR 162 kHz                             |
   * | CHU  | HF Radio CHU Ottawa, Ontario                             |
   * | WWV  | HF Radio WWV Ft. Collins, CO                             |
   * | WWVH | HF Radio WWVH Kauai, HI                                  |
   * | NIST | NIST telephone modem                                     |
   * | ACTS | NIST telephone modem                                     |
   * | USNO | USNO telephone modem                                     |
   * | PTB  | European telephone modem                                 |
   * +------+----------------------------------------------------------+
   *
   * Above stratum 1 (secondary servers and clients): this is the
   * reference identifier of the server and can be used to detect timing
   * loops.  If using the IPv4 address family, the identifier is the four-
   * octet IPv4 address.  If using the IPv6 address family, it is the
   * first four octets of the MD5 hash of the IPv6 address...
   */
  sendPacketBuffer[12] = 'G';
  sendPacketBuffer[13] = 'P';
  sendPacketBuffer[14] = 'S';
  sendPacketBuffer[15] = 0;

  /*
   * Reference Timestamp (reftime), Byte 16-23
   * ---------------------
   * get the current time and write it out as the reference time to bytes 16 to 23 of the response packet
   *
   * From rfc5905:
   * ...Reference Timestamp: Time when the system clock was last set or
   * corrected, in NTP timestamp format...
   */
  uint64_t referenceTime = getTimeOfDayInNtpFormat();
  for (int i=0;i<8;i++) sendPacketBuffer[16+i] = (int)((referenceTime >> (56-i*8)) & 0xFF);

  /*
   * Origin Timestamp (org), Byte 24-31
   * ---
   * copy transmit timestamp (xmt) from NTP client request to origin timestamp (org)
   * of the NTP server response. NTP clients compares sent xmt and received org and
   * refuses a time sync if the timestamps does not match (For example on Windows:
   * "Packet test 2 failed (response does not match request)")
   *
   * From rfc5905:
   * ...Origin Timestamp (org): Time at the client when the request departed
   * for the server, in NTP timestamp format...
   */
  memcpy(&sendPacketBuffer[24],&g_receivePacketBuffer[40],8);

  /*
   * Receive Timestamp (rec), Byte 32-39
   * ---
   * write out the receive time (it was set above) to bytes 32 to 39 of the response packet
   *
   * From rfc5905:
   * ...Receive Timestamp (rec): Time at the server when the request arrived
   * from the client, in NTP timestamp format...
   */
  for (int i=0;i<8;i++) sendPacketBuffer[32+i] = (int)((referenceTime >> (56-i*8)) & 0xFF);

  /*
   * Transmit Timestamp (xmt), Byte 40-47
   * ---
   * get the current time and write it out as the transmit time to bytes 40 to 47 of the response packet
   *
   * From rfc5905:
   * ...Transmit Timestamp (xmt): Time at the server when the response left
   * for the client, in NTP timestamp format...
   */
  uint64_t transmitTime = getTimeOfDayInNtpFormat();
  for (int i=0;i<8;i++) sendPacketBuffer[40+i] = (int)((transmitTime >> (56-i*8)) & 0xFF);

  // Send NTP response to client
  g_udp.beginPacket(clientIP, clientPort);
  g_udp.write(sendPacketBuffer, NTP_PACKET_SIZE);
  g_udp.endPacket();
}

void setup() {
  int wifiRetry = 0;

  Serial.begin(115200);

  pinMode(LED_PIN,OUTPUT);
  Serial.println("Connect Wifi ");
  WiFi.persistent(false);
  WiFi.begin(WIFISTASSID,WIFISTAPASSWORD);

  while ((WiFi.status() != WL_CONNECTED) && (wifiRetry <= WIFIMAXRETRIES)) {
    wifiRetry++;
    Serial.print(".");

    // Flash LED
    digitalWrite(LED_PIN,LOW);
    delay(50);
    digitalWrite(LED_PIN,HIGH);
    delay(450);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wifi successfully connected");
    Serial.print("NTP server ip address: ");
    Serial.println(WiFi.localIP());
  } else { // Wifi connection failed
    // Start Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFIAPSSID,WIFIAPPASSWORD);

    Serial.print("Wifi connection failed => Start Wifi AP: ");
    Serial.println(WIFIAPSSID);
    Serial.print("Password: ");
    Serial.println(WIFIAPPASSWORD);
    Serial.print("NTP server ip address: ");
    Serial.println(WiFi.softAPIP());
  }

  // Start NTP server
  g_udp.begin(NTP_PORT);

  // Start GPS communication
  g_SerialGPS.begin(GPSBAUD, SERIAL_8N1, GPSRX_PIN, GPSTX_PIN);

  /*
   * Calculate the precision value for the NTP response packets
   * rfc5905 says "minimum time of several iterations to read the system clock", but
   * to keep it simple we make only one iteration and build the average
   * 
   * From rfc5905:
   * ...Precision: 8-bit signed integer representing the precision of the
   * system clock, in log2 seconds.  For instance, a value of -18
   * corresponds to a precision of about one microsecond.  The precision
   * can be determined when the service first starts up as the minimum
   * time of several iterations to read the system clock...
   */
  #define SAMPLECOUNT 1000
  unsigned long start = micros();
  for (int i=0;i<SAMPLECOUNT;i++) {
    uint64_t testData = getTimeOfDayInNtpFormat();
  }
  unsigned long end = micros();
  g_precision = (double) log2(double ((end - start)/(double)1000000U)/(double)SAMPLECOUNT);
  #ifdef DEBUG_SERIALPRINT
  Serial.print("Clock precision: ");
  Serial.println(g_precision);
  #endif
}

void loop() {
  static unsigned long lastGPSTimeSyncMS = 0;

  while (g_SerialGPS.available()) { // When GPS data are available
    if (g_gps.encode(g_SerialGPS.read())) { // Process GPS data
      unsigned long age; // Age of GPS data in milliseconds
      int Year;
      byte Month, Day, Hour, Minute, Second, Hundredths;
      time_t t;
      struct tm tm;

      // Get GPS time in time components
      g_gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, &Hundredths, &age);

      // Convert time components to unixtime
      tm.tm_year = Year - 1900;
      tm.tm_mon = Month - 1;
      tm.tm_mday = Day;
      tm.tm_hour = Hour;
      tm.tm_min = Minute;
      tm.tm_sec = Second;
      tm.tm_isdst = 0;
      t = mktime(&tm);
      if (age < 100) { // Accept only GPS timestamps not older than 100ms
        // Set ESP32 time to the latest GPS time
        struct timeval tv;
        tv.tv_sec = t;  // epoch time (seconds)
        tv.tv_usec = Hundredths * 10000; // microseconds
        // Add GPS timestamp age
        if ((uint64_t) (tv.tv_usec + age * 1000) >= 1000000ULL) tv.tv_sec++;
        tv.tv_usec += age * 1000;
        // Set time
        settimeofday(&tv, NULL);

        g_validTime = true;
        lastGPSTimeSyncMS = millis();

        digitalWrite(LED_PIN,!digitalRead(LED_PIN)); // Toggle LED
        #ifdef DEBUG_SERIALPRINT
        Serial.print("Age ");
        Serial.println(age);
        #endif
      }
    }
  }

  int receivedPacketSize = g_udp.parsePacket(); // Get received packet size

  if (receivedPacketSize == NTP_PACKET_SIZE) // Valid NTP request
  {
    // Get client ip
    IPAddress clientIP = g_udp.remoteIP();

    // Save UDP data to buffer
    g_udp.read(g_receivePacketBuffer, NTP_PACKET_SIZE);

    // Send NTP resonse
    sendNtpResponse(clientIP, g_udp.remotePort());

    #ifdef DEBUG_SERIALPRINT
    Serial.print("Clientrequest from: ");
    Serial.print(clientIP);
    Serial.print(", Port: ");
    Serial.println(g_udp.clientPort());

    Serial.print("Leap indicator (3=clock unsynchronized): ");
    Serial.println((g_receivePacketBuffer[0] & 0b1100000) >> 6);

    Serial.print("NTP-Version: ");
    Serial.println((g_receivePacketBuffer[0] & 0b00111000) >> 3);

    Serial.print("Mode (3=Client,4=Server): ");
    Serial.println(g_receivePacketBuffer[0] & 0b00000111);

    Serial.print("Reference ID: ");
    Serial.print(g_receivePacketBuffer[12]);
    Serial.print(".");
    Serial.print(g_receivePacketBuffer[13]);
    Serial.print(".");
    Serial.print(g_receivePacketBuffer[14]);
    Serial.print(".");
    Serial.println(g_receivePacketBuffer[15]);

    Serial.print("Client time: ");
    SerialPrintNtpTime(packetBytesToUINT64(&g_receivePacketBuffer[40]));
    #endif
  } else if (receivedPacketSize > 0) {
    g_udp.flush(); // Discard packet, when packet size is not NTP_PACKET_SIZE

    #ifdef DEBUG_SERIALPRINT
    Serial.print("Invalid packet received. Size: ");
    Serial.println(receivedPacketSize);
    #endif
  }

  // Check age of last GPS time sync
  if (millis()-lastGPSTimeSyncMS > MAXGPSUNSYNCEDTIMEMS) { // Too old...
    g_validTime = false; // Invalidate time
    lastGPSTimeSyncMS = millis() - MAXGPSUNSYNCEDTIMEMS + 1; // Prevent overrun
  }
}