
/* 
 * Cloudcrowd - a LoraWAN weather alert and reporting network
 *
 *
 * LoRaWAN client based on esp32-ttnmapper-gps.ino by Luiz H. Cassettari
 */
#include "config.h"

//----------------------------------------//

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>

#ifdef ESP32
#include <Preferences.h>
#include <WiFi.h>

Preferences preferences;

#endif

#define LORA_HTOI(c) ((c<='9')?(c-'0'):((c<='F')?(c-'A'+10):((c<='f')?(c-'a'+10):(0))))
#define LORA_TWO_HTOI(h, l) ((LORA_HTOI(h) << 4) + LORA_HTOI(l))
#define LORA_HEX_TO_BYTE(a, h, n) { for (int i = 0; i < n; i++) (a)[i] = LORA_TWO_HTOI(h[2*i], h[2*i + 1]); }
#define LORA_DEVADDR(a) (uint32_t) ((uint32_t) (a)[3] | (uint32_t) (a)[2] << 8 | (uint32_t) (a)[1] << 16 | (uint32_t) (a)[0] << 24)

static uint8_t DEVADDR[4];
static uint8_t NWKSKEY[16];
static uint8_t APPSKEY[16];

bool i2c_probe[128];
#define I2C_ADDR_DHT (uint8_t)0x5c
#define I2C_ADDR_MPU6050 (uint8_t)0x68

// Pin mapping - TTGO Lora32
//
#if defined(CC_PLATFORM_LORA32)
const lmic_pinmap lmic_pins = {
  .nss = 18, 
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};
#elif defined(CC_PLATFORM_MINI32)
// Pin mapping for TTGO Mini32/D1
// use the D1 bus pins for SPI (SS=D8 MOSI=D7 MISO=D6 SCK=D5)
// use D4=DIO1 and D3=RST
// (Leaving only D1/D2 for I2C, and D0 for INT)
const lmic_pinmap lmic_pins = {
  .nss = 5, 
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = {/*dio0*/ 16, /*dio1*/ 25, /*dio2*/ LMIC_UNUSED_PIN}
};
#elif defined(CC_PLATFORM_UNO)
const lmic_pinmap lmic_pins = {
  .nss = 10, 
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {/*dio0*/ 2, /*dio1*/ 6, /*dio2*/ LMIC_UNUSED_PIN}
};
#else
#error no pin map
#endif

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


#define DHT_INVALID 32768

uint16_t hum = DHT_INVALID, last_hum=DHT_INVALID;
int16_t temp = DHT_INVALID, last_temp=DHT_INVALID;
uint16_t hum_hysteresis = 10,temp_hysteresis = 5;
uint16_t battery_mv = 0;
uint8_t battery_level = 0;

#define BATTERY_MV_EMPTY 3100
#define BATTERY_MV_FULL  4100


uint32_t dht_poll_ms = 15 * 1000;
uint32_t motion_poll_ms = 1000;
uint32_t last_dht_poll = 0;
uint32_t last_motion_poll = 0;
uint32_t raincount = 0,rain_debounce=0;
uint32_t hailcount = 0,hail_debounce=0;
#define DEBOUNCE_MILLIS 50
bool rain_detect = false, hail_detect = false, button_detect=false, dht_detect=false, battery_detect=false;

bool mpu6050_present = false;
extern void mpu6050_setup();

static osjob_t sendjob;


#if defined(CC_PLATFORM_LORA32)
// TTGO Lora32 - share I2C with Oled screen
const uint8_t PIN_SDA=4;
const uint8_t PIN_SCL=15;
const uint8_t PIN_RAIN=13;
const uint8_t PIN_BUTTON=0;
const uint8_t PIN_BATTERY=LMIC_UNUSED_PIN;  // schematic says 35, but does not work!

const uint8_t PIN_MOTION_INT=17;

#elif defined(CC_PLATFORM_MINI32)
// TTGO mini32 - use the D1 mini bus pins
const uint8_t PIN_SDA=21;
const uint8_t PIN_SCL=22;
const uint8_t PIN_RAIN=27;
const uint8_t PIN_BUTTON=0;
const uint8_t PIN_BATTERY=LMIC_UNUSED_PIN;

// TTGo mini32 - use D1 pin D0 (GPIO26), or no interrupt pin and poll the interrupt status
//const uint8_t PIN_MOTION_INT=26; 
//const uint8_t PIN_MOTION_INT=LMIC_UNUSED_PIN;
const uint8_t PIN_MOTION_INT=14;  // on the outer row, next to 5(NSS)
#else
const uint8_t PIN_MOTION_INT=LMIC_UNUSED_PIN;
const uint8_t PIN_RAIN=3;
const uint8_t PIN_BUTTON=4;
#endif

#if LMIC_ENABLE_event_logging
extern "C" {
    void LMICOS_logEvent(const char *pMessage);
    void LMICOS_logEventUint32(const char *pMessage, uint32_t datum);
}

void LMICOS_logEvent(const char *pMessage)
    {
      Serial.printf("LMIC EVENT %s\n", pMessage);
    }

void LMICOS_logEventUint32(const char *pMessage, uint32_t datum)
    {
      Serial.printf("LMIC EVENT %s 0x%08lx\n", pMessage, datum);
    }
#endif // LMIC_ENABLE_event_logging


void DumpHex(const char *leader, const void* data, size_t size) {
  char ascii[17];
  size_t i, j;
  int leader_len = strlen(leader)+2;
  ascii[16] = '\0';

  LMIC_DEBUG_PRINTF("%s: ", leader);
  for (i = 0; i < size; ++i) {
    LMIC_DEBUG_PRINTF("%02X ", ((unsigned char*)data)[i]);
    if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
      ascii[i % 16] = ((unsigned char*)data)[i];
    } else {
      ascii[i % 16] = '.';
    }
    if ((i+1) % 8 == 0 || i+1 == size) {
      LMIC_DEBUG_PRINTF(" ");
      if ((i+1) % 16 == 0) {
	LMIC_DEBUG_PRINTF("|  %s \n", ascii);
	if (i+1 != size) {
	  for (int ldr=0;ldr<leader_len;ldr++) {
	    LMIC_DEBUG_PRINTF(" ");
	  }
	}
      } else if (i+1 == size) {
	ascii[(i+1) % 16] = '\0';
	if ((i+1) % 16 <= 8) {
	  LMIC_DEBUG_PRINTF(" ");
	}
	for (j = (i+1) % 16; j < 16; ++j) {
	  LMIC_DEBUG_PRINTF("   ");
	}
	LMIC_DEBUG_PRINTF("|  %s \n", ascii);
      }
    }
  }
}

void i2c_scan() 
{
  int nDevices = 0;

  LMIC_DEBUG_PRINTF("Scanning I2C bus\n");
  for(int address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    i2c_probe[address] = false;
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      LMIC_DEBUG_PRINTF("  I2C device present at 0x%02x\n",address);
      nDevices++;
      i2c_probe[address] = true;
    }
    else if (error==4) 
    {
      LMIC_DEBUG_PRINTF("  Unknown error at address 0x%02x\n", address);
    }    
  }
  LMIC_DEBUG_PRINTF("I2C Scan found %d device%s\n", nDevices, (nDevices==1)?"":"s");
}

int i2c_read_reg(int address, uint8_t reg, int timeout=1000) 
{
  byte v;
  unsigned long start = millis();
  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom((int)address, (int)1);
  while (!Wire.available()) {
    unsigned long now = millis();
    if ((now - start) > timeout) {
      LMIC_DEBUG_PRINTF("ERROR i2c_read_reg(%d, %d) timeout\n", (int)address, (int)reg);
      return -1;
    }
  }
  v = Wire.read();
  return v;
}

void i2c_write_reg(int address, uint8_t reg, uint8_t value, int timeout=1000) 
{
  unsigned long start = millis();

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void set_bit(uint8_t &v, uint8_t b) 
{
  v |= (1<<b);
}

void clear_bit(uint8_t &v, uint8_t b) 
{
  v &= ~(1<<b);
}


void onEvent (ev_t ev) {
  LMIC_DEBUG_PRINTF("%lu: ",(unsigned long)os_getTime());
  switch (ev) {
  case EV_SCAN_TIMEOUT:
    oled_status(" - SCAN_TIMEOUT - ");
    break;
  case EV_SCAN_FOUND:
    oled_status(" - SCAN_FOUND - ");
    break;
  case EV_BEACON_FOUND:
    oled_status(" - BEACON_FOUND - ");
    break;
  case EV_BEACON_MISSED:
    oled_status(" - BEACON_MISSED - ");
    break;
  case EV_BEACON_TRACKED:
    oled_status(" - BEACON_TRACKED - ");
    break;
  case EV_JOINING:
    oled_status(" --- JOINING --- ");
    break;
  case EV_JOINED:
    oled_status(" --- JOINED --- ");
    break;
  case EV_JOIN_FAILED:
    oled_status(" --- JOIN_FAIL --- ");
    break;
  case EV_REJOIN_FAILED:
    oled_status(" -- REJOIN_FAIL -- ");
    break;
  case EV_LOST_TSYNC:
    oled_status(" -- LOST_TSYNC -- ");
    break;
  case EV_RESET:
    oled_status(" --- RESET --- ");
    break;
  case EV_RXCOMPLETE:
    oled_status(" --- RXCOMPLETE --- ");
    break;
  case EV_LINK_DEAD:
    oled_status(" --- LINK_DEAD --- ");
    break;
  case EV_LINK_ALIVE:
    oled_status(" --- LINK_ALIVE --- ");
    break;
  case EV_TXCOMPLETE:
    oled_status(" --- TXCOMPLETE --- ");
    LMIC_DEBUG_PRINTF("EV_TXCOMPLETE (includes waiting for RX windows)\n");
    if (LMIC.txrxFlags & TXRX_ACK) 
    {
      LMIC_DEBUG_PRINTF("Received ack\n");
    }
    if (LMIC.dataLen != 0 || LMIC.dataBeg != 0) {
      uint8_t port = 0;
      if (LMIC.txrxFlags & TXRX_PORT)
      {
	port = LMIC.frame[LMIC.dataBeg - 1];
      }
      message(LMIC.frame + LMIC.dataBeg, LMIC.dataLen , port);
    }
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(SEND_TIMER), do_send);
    break;
  case EV_TXSTART:
    oled_status(" --- TXSTART --- ");
    break;
  case EV_TXCANCELED:
    oled_status(" -- TXCANCELED -- ");
    break;
  case EV_RXSTART:
    oled_status(" --- RXSTART --- ");
    break;
  case EV_JOIN_TXCOMPLETE:
    oled_status("- JOIN_TXCOMPLETE -");
    break;
  }
}

bool radioInUse() 
{
  return (LMIC.opmode & OP_TXRXPEND);
}

#ifdef __AVR
#define htons(x) (((x&0xFF)<<8)|((((unsigned int)x)>>8)&0xff))
#define htonl(x) ((x>>24)&0xff|(x>>8)&0xFF00|(x<<8)&0xFFFF00|(x<<24)&0xFF000000)
#endif

void appendShort(unsigned char *buf, int *pos, uint16_t value) 
{
  uint16_t nval = htons(value);
  memcpy(buf+*pos, &nval, sizeof(nval));
  *pos += sizeof(nval);
}

void appendLong(unsigned char *buf, int *pos, uint32_t value) 
{
  uint32_t nval = htonl(value);
  memcpy(buf+*pos, &nval, sizeof(nval));
  *pos += sizeof(nval);
}

void appendByte(unsigned char *buf, int *pos, unsigned char value) 
{
  buf[*pos] = value;
  *pos ++;
}

void do_send(osjob_t* j) {
  unsigned char txBuffer[16];
  int txLen = 0;
  bool timer = (j!=NULL);
  lmic_tx_error_t err;
  static const char *tx_error_name[] = {LMIC_ERROR_NAME__INIT};
  

  // Check if there is not a current TX/RX job running
  if (radioInUse()) {
    LMIC_DEBUG_PRINTF("\ndo_send%s radio in use, not sending\n",timer?" (timer)":"");
  } else {
    LMIC_DEBUG_PRINTF("\ndo_send%s hum=%.1f temp=%.1f raincount=%lu hailcount=%lu\n",
		  timer?" (timer)":"",
		  hum/10.0, temp/10.0, raincount, hailcount);
    appendByte(txBuffer, &txLen, CLOUDCROWD_VERSION);
    appendByte(txBuffer, &txLen, battery_level);
    appendShort(txBuffer, &txLen, hum);
    appendShort(txBuffer, &txLen, temp);
    appendLong(txBuffer, &txLen, raincount);
    appendLong(txBuffer, &txLen, hailcount);
    DumpHex("Transmit", txBuffer, txLen);
    err = LMIC_setTxData2(1, txBuffer, txLen, 0);

    if (err != 0) {
      Serial.printf("LMIC transmit error %d (%s)\n", err, (err>=LMIC_ERROR_TX_FAILED)?tx_error_name[-err]:"unknown");
    }

  }
}

void
IRAM_ATTR
rain() {
    uint32_t now = millis();

    if ((rain_debounce > 0) && ((rain_debounce + DEBOUNCE_MILLIS) > now)) {
      // Ignore events during debounce window
      return;
    }

    //LMIC_DEBUG_PRINTF("RAIN\n");
    rain_debounce = now;
    raincount++;
    rain_detect = true;
}

void
IRAM_ATTR 
hail() {
    uint32_t now = millis();

    if ((hail_debounce > 0) && ((hail_debounce + DEBOUNCE_MILLIS) > now)) {
      // Ignore events during debounce window
      return;
    }
    //LMIC_DEBUG_PRINTF("HAIL\n");
    hail_debounce = now;
    hailcount++;
    hail_detect = true;
}


void
IRAM_ATTR
button() {
    button_detect = true;
}

void motion_loop() 
{
  if (!mpu6050_present) {
    // motion sensor not present
    return;
  }
  
  if (PIN_MOTION_INT != LMIC_UNUSED_PIN) {
    // motion sensor uses interrupt, poll not required
    return;
  }
  
  uint32_t start = millis();
  if ((last_motion_poll > 0) && (start < (last_motion_poll + motion_poll_ms))) {
    // It is not yet time for the next poll
    return;
  }

  // We are going to do a poll
  //LMIC_DEBUG_PRINTF("Motion poll\n");
  last_motion_poll = start;
  
  int status = i2c_read_reg(I2C_ADDR_MPU6050, 0x3a);
  if (status!=0) {
    LMIC_DEBUG_PRINTF("MPU6050 INT_STATUS %02x\n", status);
  }

  if ((status > 0) && (status & 0x40)) {
    LMIC_DEBUG_PRINTF("Hail detect\n");
    hail_detect = true;
  }
}

void battery_loop(bool log=false) 
{
  static uint8_t last_battery_level = 0;
  
  if (PIN_BATTERY == LMIC_UNUSED_PIN) return;

  int battery = analogRead(PIN_BATTERY); // 0-4096 means 0--6300mv
  Serial.printf("Raw battery reading %d\n", battery);

  battery_mv = map(battery, 0, 1024, 0, 6600);

  if (battery_mv < BATTERY_MV_EMPTY ) {
    battery_level = 0;
  }
  else if (battery_mv  >= BATTERY_MV_FULL) {
    battery_level = 100;
  }
  else {
    // consider 3100mv to be "flat" and 4100mv to be full
    battery_level = map(battery_level, BATTERY_MV_EMPTY, BATTERY_MV_FULL, 0, 100);
  }
  if (log) {
    Serial.printf("Battery voltage %dmV (%d%%)\n", battery_mv, (int)battery_level);
  }

  if (battery_level != last_battery_level) {
    battery_detect = true;
  }
}

void dht_loop() 
{
  uint32_t start = millis();
  if ((last_dht_poll > 0) && (start < (last_dht_poll + dht_poll_ms))) {
    return;
  }
  last_dht_poll = start;
  //LMIC_DEBUG_PRINTF("DHT poll\n");

  const int timeout = 250;

  Wire.beginTransmission(I2C_ADDR_DHT);
  Wire.write(0);
  if (Wire.endTransmission() != 0) {
    LMIC_DEBUG_PRINTF("DHT not responding\n");
    return;
  }

  Wire.requestFrom(I2C_ADDR_DHT, (uint8_t)5);
  /*

  while (!Wire.available()) {
    unsigned long now = millis();
    if (now > (start+timeout)) {
      LMIC_DEBUG_PRINTF("DHT: Timeout\n");
      return;
    }
  }
  */
  
  unsigned char dht_data[5];
  unsigned char sum = 0;
  sum += dht_data[0] = Wire.read();
  sum += dht_data[1] = Wire.read();
  sum += dht_data[2] = Wire.read();
  sum += dht_data[3] = Wire.read();
  dht_data[4] = Wire.read();

  //DumpHex("DHT data", dht_data, 5);

  if (sum != dht_data[4]) {
    LMIC_DEBUG_PRINTF("DHT checksum error got %d expected %d\n", (int)dht_data[4], (int)sum);
    return;
  }

  // Data are byte packed fixed-point decimal (first byte is units, second byte is tenths)
  hum = dht_data[0]*10 + dht_data[1];
  temp = ((char *)dht_data)[2]*10 + dht_data[3];

  if ((last_temp == DHT_INVALID) || abs(temp-last_temp) >= temp_hysteresis) {
    LMIC_DEBUG_PRINTF("Significant temperature change %d => %d\n", (int)last_temp, (int)temp);
    dht_detect = true;
    last_temp = temp;
  }

  if ((last_hum == DHT_INVALID) || abs(hum-last_hum) >= hum_hysteresis) {
    LMIC_DEBUG_PRINTF("Significant humidity change %d => %d\n", (int)last_hum, (int)hum);
    dht_detect = true;
    last_hum = hum;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("CloudCrowd starting"));
  
#ifdef ESP32
  Serial.println(F("Turn wifi off to save battery"));
  WiFi.mode(WIFI_OFF);

  Serial.println(F("Read Preferences"));
  preferences.begin("cloudcrowd", false);
  raincount = preferences.getULong("raincount", 0);
  if (raincount != 0) Serial.printf("raincount=%lu\n", raincount);
  hailcount = preferences.getULong("hailcount", 0);
  if (hailcount != 0) Serial.printf("hailcount=%lu\n", hailcount);
#endif

  if (PIN_BATTERY != LMIC_UNUSED_PIN) {
    Serial.println("Configure battery input");
    pinMode(PIN_BATTERY, INPUT);
    battery_loop(true);
  }

  Serial.println("Configure raingauge interrupt");
  pinMode(PIN_RAIN, INPUT_PULLUP);
  attachInterrupt(PIN_RAIN, rain, FALLING);

  Serial.println("Configure button interrupt");
  pinMode(0, INPUT_PULLUP);
  attachInterrupt(0, button, FALLING);

  Serial.println(F("Configure I2C"));
  // I2C on 4,15 for TTGO lora32 boards
#ifdef __AVR
  Wire.begin();
#else
  Wire.begin(PIN_SDA, PIN_SCL);
#endif
  i2c_scan();

  if (i2c_probe[I2C_ADDR_MPU6050]) {
    int whoami = i2c_read_reg(I2C_ADDR_MPU6050, 117);
    if (whoami == 0x68) {
      Serial.println("MPU-6050 IMU device detected");
      mpu6050_setup();
      mpu6050_present = true;
      if (PIN_MOTION_INT != LMIC_UNUSED_PIN) {
	pinMode(PIN_MOTION_INT, INPUT_PULLUP);
	attachInterrupt(PIN_MOTION_INT, hail, FALLING);
      }
	
    }
    else {
      LMIC_DEBUG_PRINTF("IMU device not detected (whoami=0x%x)\n", whoami);
    }
  }
  

#ifdef USE_OLED
  Serial.println(F("Configure OLED"));
  oled_setup();
#endif

#ifdef USE_GPS
  Serial.println(F("Configure GPS"));
  gps_setup();
#endif
  
  Serial.println(F("Configure LoRa"));
  LORA_HEX_TO_BYTE(DEVADDR, devAddr, 4);
  LORA_HEX_TO_BYTE(NWKSKEY, nwkSKey, 16);
  LORA_HEX_TO_BYTE(APPSKEY, appSKey, 16);
  
  if (LORA_DEVADDR(DEVADDR) == 0) while(true);

  // Initialise the LMIC (LoraWAN) Library
  os_init();
  LMIC_reset();
  LMIC_setSession (0x13, LORA_DEVADDR(DEVADDR), NWKSKEY, APPSKEY);
  
  LMIC_setAdrMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
#if defined(CFG_au915) || defined(CFG_us915)
  LMIC_selectSubBand(1);
#ifdef SINGLE_CHANNEL
  for (int c = 0; c < 72; c++){
    LMIC_disableChannel(c);
  }
  LMIC_enableChannel(SINGLE_CHANNEL);
#endif
#endif  

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  //LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  //LMIC_setDrTxpow(DR_SF7,14);

  // The first DHT poll will initiate a send on change from DHT_INVALID to
  // valid values (but if the DHT is disconnected, force a send anyway)
  //do_send(NULL);
  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(SEND_TIMER), do_send);
}

void loop() {
  os_runloop_once();

  battery_loop();

  // Poll DHT (will be no-op if poll interval has not elapsed)
  dht_loop();

  // Maybe poll MPU6050 (only needed if no interrupt pin defined)
  motion_loop();

#ifdef USE_GPS
  gps_loop();
#endif
#ifdef USE_OLED
  oled_loop();
#endif

  // Accumulate changes until the radio is idle
  if (!radioInUse()) {
    bool need_to_send = false;
    unsigned long now = millis();

    if (rain_detect) {
      LMIC_DEBUG_PRINTF("Trigger send for updated rain count\n");
#ifdef ESP32
      preferences.putULong("raincount", raincount);
#endif
      rain_detect = false;
      need_to_send = true;
    }
    if (hail_detect) {
      LMIC_DEBUG_PRINTF("Trigger send for updated hail count\n");
#ifdef ESP32
      preferences.putULong("halicount", hailcount);
#endif
      hail_detect = false;
      need_to_send = true;

      // read MPU6050 INT_STATUS register to clear interrupt flag
      int readdata = i2c_read_reg(I2C_ADDR_MPU6050, 0x3a);
      LMIC_DEBUG_PRINTF("MPU6050 INT_STATUS %02x\n", readdata);

    }
    if (button_detect) {
      LMIC_DEBUG_PRINTF("Trigger send for manual button press\n");
      button_detect = false;
      need_to_send = true;
    }
    if (dht_detect) {
      LMIC_DEBUG_PRINTF("Trigger send for change of DHT12 reading\n");
      dht_detect = false;
      need_to_send = true;
    }
    if (battery_detect) {
      // report updated battery level
      battery_detect = false;
      need_to_send = true;
    }
    
    // The OS timer job triggered by TXCOMPLETE will also act as a 15min
    // heartbeat
    
    if (need_to_send) {
      do_send(NULL);
    }
  }
  
}

void message(const uint8_t *payload, size_t size, uint8_t port)
{
  Serial.println("-- MESSAGE");
  Serial.println("Received " + String(size) + " bytes on port " + String(port) + ":");
  if (port == 0) 
  {
    oled_status(" --- TX_CONFIRMED --- ");
    return;
  }
  if (size == 0) return;
  switch (port) {
    case 1:
      break;
  }
}
