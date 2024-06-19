/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <tinycbor.h>
#include <Arduino.h>
#include "wiring_private.h"

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x3D, 0x3C, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xE4, 0x5F, 0x9B, 0x0D, 0xB6, 0xB6, 0x40, 0x81, 0xC0, 0x13, 0x8B, 0x4A, 0x4F, 0xF6, 0x02, 0x16 };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// static uint8_t mydata[] = "Hello, world!";
//static uint8_t mydata[] = {0xA1, 0x6B, 0x74, 0x65, 0x6D, 0x70, 0x65, 0x72, 0x61, 0x74, 0x75, 0x72, 0x65, 0x15};
uint8_t encode_buffer[256];

static osjob_t sendjob;

int SerialRxData;

static int crawlerID = 12345678;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 4,  //chip select
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = { 6, 7, LMIC_UNUSED_PIN },  //interchangeable with GPIO
};

//cbor stuff
int err = 0;
int err_line = 0;
uint8_t* txbuf;
size_t txsz;

bool flag = false;

//GNSS/GPS Stuff
#include <Wire.h>  //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#define LORA_TXPIN 0
#define LORA_RXPIN 1

Uart mySerial(&sercom3, LORA_RXPIN, LORA_TXPIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);



#define CHECK_ERROR(proc) \
  { \
    if ((err = proc) != 0) { \
      err_line = __LINE__; \
      goto on_error; \
    } \
  }




void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      if (flag){
      mySerial.write('d'); //area 3: after acknowlegement (after it confirms things are sent and recieved)
      }

      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}



void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    //mydata[sizeof(mydata) - 1] = SerialRxData;
    //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    LMIC_setTxData2(1, txbuf, txsz, 0);
    Serial.println(F("Packet queued"));

    //mySerial.write(); //area 2: after packet queued, but hasn't transmitted
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  //Serial.begin(9600);
  //comment out the below line to start inference immediately after upload
  // while (!Serial)
  //   delay(100);
  // Serial.println("serial beginning (LORA to Computer)");

  mySerial.begin(9600);
  //comment out the below line to start inference immediately after upload
  while (!mySerial)
    delay(100);
  Serial.println("serial beginning (LORA to UNO)");

  pinPeripheral(1, PIO_SERCOM);
  pinPeripheral(0, PIO_SERCOM);

  TinyCBOR.init();


#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_selectSubBand(1);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);

  //GNSS Stuff

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false)  //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);                  //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR
}

void loop() {

  if (mySerial.available()) {



    SerialRxData = mySerial.read();
    mySerial.flush();
    if (SerialRxData > 100 || SerialRxData < 0) {
      SerialRxData = 0xff;
    }
    Serial.print("probability Recieved: ");
    Serial.println(SerialRxData, DEC);

    double latitude = myGNSS.getLatitude() / 10000000.0;
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    double longitude = myGNSS.getLongitude() / 10000000.0;
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.println(F(" (degrees"));



    // Assign buffer to encoder.
    TinyCBOR.Encoder.init(encode_buffer, sizeof(encode_buffer));

    CHECK_ERROR(TinyCBOR.Encoder.create_map(3));
    {
      CHECK_ERROR(TinyCBOR.Encoder.encode_text_stringz("CrawlerID"));
      CHECK_ERROR(TinyCBOR.Encoder.encode_int(crawlerID));
      CHECK_ERROR(TinyCBOR.Encoder.encode_text_stringz("prediction"));
      CHECK_ERROR(TinyCBOR.Encoder.encode_int(SerialRxData));


      CHECK_ERROR(TinyCBOR.Encoder.encode_text_stringz("location"));
      CHECK_ERROR(TinyCBOR.Encoder.create_map(2));
      {
        CHECK_ERROR(TinyCBOR.Encoder.encode_text_stringz("latitude"));
        CHECK_ERROR(TinyCBOR.Encoder.encode_double(latitude));
        CHECK_ERROR(TinyCBOR.Encoder.encode_text_stringz("longitude"));
        CHECK_ERROR(TinyCBOR.Encoder.encode_double(longitude));
      }
      CHECK_ERROR(TinyCBOR.Encoder.close_container());
    }
    CHECK_ERROR(TinyCBOR.Encoder.close_container());


on_error:
    // Detect error in CHECK_ERROR macro, jump to here.
    if (err) {
      Serial.print("Error: ");
      Serial.print(err);
      Serial.print(" raise at line:");
      Serial.print(err_line);
      Serial.println();
    }

    txbuf = TinyCBOR.Encoder.get_buffer();
    txsz = TinyCBOR.Encoder.get_buffer_size();

    os_setCallback(&sendjob, do_send);

    flag = true;

    //mySerial.write(); //area 1: after do_send begins, but not completes
  }

  else {
    os_runloop_once();
  }
}


void SERCOM3_Handler() {

  mySerial.IrqHandler();
}
