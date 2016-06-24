/*
  This example controls an Analog Devices ADE7913 energy monitor.

  The circuit (For details see PDF schematic)
  ==================================================================================
  Arduino Mega    --    ADE7913 (NB: All connections made via TXB0108 5V-3V3 Shifter
  ==================================================================================
  Digital 2       --    DREADY/CLKOUT
  Digital 3       --    SS
  Digital 9       --    XTAL 1
  GND             --    GND (on non-isolated side)
  3V3             --    VDD (non-isolated side)
  5V              --    5V rail of TXB0108 level shifter
  ISCP-1          --    MISO
  ICSP-3          --    SCLK
  ICSP-4          --    MOSI
  ===================================================================================

  This code is in the public domain, no guarantees of performance and all that:
  

  Credit to Corgitronics for getting me started with his github code:
  https://github.com/corgitronics/Arduino/blob/master/ADE7913_average/ADE7913_average.ino
  https://corgitronics.com/
*/

// inslude the SPI library:
#include <SPI.h>

// SPI settings for the ADE7913, use min. SCLK speed of 250kHz for debugging:
SPISettings spiSettings(4096000, MSBFIRST, SPI_MODE3);

// DEFINE COMMAND BYTES FOR ADE7913, 5-bit address. reads end 100:
#define STATUS0_READ      (0x9  << 3 | 0b100)
#define CONFIG_READ       (0x8  << 3 | 0b100)
#define TEMPOS_READ       (0x18 << 3 | 0b100)
#define IWV_READ          (0x0  << 3 | 0b100)  // Also starts 'burst' read of (IWV, V1WV, V2WV, ADC_CRC, STATUS0, CNT_SNAPSHOT)
#define EMI_CTRL_READ     (0xE  << 3 | 0b100)
#define V1WV_READ         (0x1  << 3 | 0b100)
#define V2WV_READ         (0x2  << 3 | 0b100)
#define ADC_CRC_READ      (0x4  << 3 | 0b100)
#define CNT_SNAPSHOT_READ (0x7  << 3 | 0b100)

// writes end 000:
#define CONFIG_WRITE     (0x8 << 3 | 0b000)
#define EMI_CTRL_WRITE   (0xE << 3 | 0b000)
#define SYNC_SNAP_WRITE  (0xB << 3 | 0b000)
#define LOCK_KEY_WRITE   (0xA << 3 | 0b000)

// miscelaneous bytes:
#define DUMMY_MSG        0x00               // Unused argument to SPI.Transfer()
#define LOCK_BYTE        0xCA
#define UNLOCK_BYTE      0x9C

// Definitions to do with 4MHz CLOCK:
const int mhz4 = 9;                 // Timer 2 "B" output: OC2B, in Arduino MEGA is pin 9, produce 4MHz on this pin
const long frequency = 4096000L;    // Target clock freqyency in Hz

// Set pin 3 as the slave select for the ADE7913
const int slaveSelectPin = 3;

// Set pin 2 as data-ready from ADE7913
const int dataReadyPin = 2;

// Setup signed 3-byte word to store ADC results, NB: use 4 bytes to match int32_t (signed 32bit integer)
union threeByteWord
{
  int32_t value;
  byte bytes[4];
};

// Settings for writing updates to serial, and ADE7913 syncing:
// define as volatile all variables to be updated from iterrupt service routine
const unsigned long writePeriodMillis = 2000;
const unsigned long syncPeriodMillis = 10000;
unsigned long previousWriteMillis = 0;
unsigned long previousSyncMillis = 0;
const int rdDelayMicros  = 0;
const int nMaxWriteTry = 100;
volatile unsigned long microsForBurstRead;
volatile unsigned long microsBetweenReads;
volatile unsigned long microsPreviousRead;

// Local copies of ADC readings, updated on dataReady interrupt
volatile long nReads = 0;
volatile threeByteWord IWV;
volatile threeByteWord V1WV;
volatile threeByteWord V2WV;
volatile byte ADC_CRC[2];
volatile byte STATUS0[1];
volatile byte CNT_SNAPSHOT[2];
volatile byte ADC_CRC_burst[2];
volatile byte CONFIG[1];
volatile byte TEMPOS[1];
volatile byte EMI_CTRL[1];

//======================//
//======  SETUP  =======//
//======================//
void setup() {
  // dettach interrupt to allow set-up to take place
  detachInterrupt(digitalPinToInterrupt(dataReadyPin));

  // Set 4MHz clock going (copied from elsewhere on internet, don't fully understand this)
  pinMode (mhz4, OUTPUT);
  TCCR2A = bit (WGM20) | bit (WGM21) | bit (COM2B1);    // fast PWM, clear OC2A on compare
  TCCR2B = bit (WGM22) | bit (CS20);                    // fast PWM, no prescaler
  OCR2A =  (F_CPU / frequency) - 1;                     // zero relative
  OCR2B = ((OCR2A + 1) / 2) - 1;                        // 50% duty cycle

  // Set slaveSelectPin as an output, and set high so chip not selected
  pinMode (slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);

  // Initialize SPI:
  SPI.begin();
  SPI.beginTransaction(spiSettings);

  // Initialize serial for reporting results
  Serial.begin(115200);

  // Read STATUS0 register, until Bit 0 (RESET_ON) is cleared:
  STATUS0[0] = 0b11111111;
  int nTry = 0;
  do {
    readMultBytesADE7913(STATUS0_READ, STATUS0, 1);
    nTry++;
  } while (bitRead(STATUS0[0], 0) && nTry < nMaxWriteTry);

  // Check if bit succusfully cleared
  if (bitRead(STATUS0[0], 0)) {
    Serial.print("ERROR: RESET_ON bit NOT cleared, nTry: "); Serial.println(nTry);
    Serial.print("STATUS0[0]: "); Serial.println(STATUS0[0], BIN);
    while (true) {};  // LOOP forever  on failure
  } else {
    Serial.print("RESET_ON bit cleared, nTry: "); Serial.println(nTry);
    Serial.print("STATUS0[0]: "); Serial.println(STATUS0[0], BIN);
  }

  // Unlock CONFIG registers, NB this cannot be read back
  writeADE7913(LOCK_KEY_WRITE, UNLOCK_BYTE);
  Serial.println("Registers unlocked!");

  // Initialize CONFIG register with bit 0 (CLKOUT_EN) cleared (to 0)
  // as CLKOUT unecessary (we provide it from ardiuno)
  // also SET TEMP_EN (bit 3) so temperature can be measured (we're not using V2P)
  // SET ADC_FREQ (bit 5:4) to 11 (1kHz for debugging), otherwise 00 (8kHx) for running
  boolean writeSuccess = writeADE7913_check(CONFIG_WRITE, 0b00001000, CONFIG_READ);
  delay(100);
  readMultBytesADE7913(CONFIG_READ, CONFIG, 1);
  if (writeSuccess) {
    Serial.println("CONFIG write success!");
    Serial.print("CONFIG[0]: "); Serial.println(CONFIG[0], BIN);
  } else {
    Serial.println("ERROR: CONFIG Write Failed");
    Serial.print("CONFIG[0]: "); Serial.println(CONFIG[0], BIN);
    while (true) {};  // LOOP forever  on failure
  }

  // Read temperature offset register:
  readMultBytesADE7913(TEMPOS_READ, TEMPOS, 1);
  Serial.print("TEMPOS: "); Serial.println((int8_t) TEMPOS[0], DEC);

  // Set the EMI_CTRL register; and check written correctly:
  writeSuccess = writeADE7913_check(EMI_CTRL_WRITE, 0b01010101, EMI_CTRL_READ);
  delay(100);
  readMultBytesADE7913(EMI_CTRL_READ, EMI_CTRL, 1);
  if (writeSuccess) {
    Serial.println("EMI_CTRL write success!");
    Serial.print("EMI_CTRL[0]: "); Serial.println(EMI_CTRL[0], BIN);
  } else {
    Serial.println("ERROR: EMI_CTRL Write Failed");
    Serial.print("EMI_CTRL[0]: "); Serial.println(EMI_CTRL[0], BIN);
    while (true) {};  // LOOP forever  on failure
  }

  // Execute a SYNC_SNAP = 0x01 write broadcast, NB will be cleared to 0x00 after 1 CLK cycle
  writeADE7913(SYNC_SNAP_WRITE, 0b00000001);
  Serial.println("SYNC_SNAP Register Set!");

  // Execute a LOCK_KEY = 0xCA (to lock CONFIG registers). This cannot be read back
  writeADE7913(LOCK_KEY_WRITE, LOCK_BYTE);
  Serial.println("Registers locked!");

  SPI.endTransaction();


  Serial.println();
  Serial.println(" --- SETUP COMPLETE ---");
  Serial.println();

  // TEST MIN/MAX VALUE OF 3-BYTE WORD AS EXPECTED:
  volatile threeByteWord test3bw;
  test3bw.bytes[2] = 0b10000000;
  test3bw.bytes[1] = 0b00000000;
  test3bw.bytes[0] = 0b00000000;
  extendSignBit(&test3bw);
  Serial.print("Three byte word min: "); Serial.println(test3bw.value, DEC);

  test3bw.bytes[2] = 0b01111111;
  test3bw.bytes[1] = 0b11111111;
  test3bw.bytes[0] = 0b11111111;
  extendSignBit(&test3bw);
  Serial.print("Three byte word max: "); Serial.println(test3bw.value, DEC);
  for (int i = 0; i < 4; i++) {
    Serial.print("bytes["); Serial.print(i); Serial.print("]: "); Serial.println(test3bw.bytes[i], BIN);
  }
  for (int i = 0; i < 8; i++) {
    Serial.print("bit "); Serial.print(i); Serial.print(" of bytes[2]: "); Serial.println(bitRead(test3bw.bytes[2], i));
  }
  delay(250);
  // Set !dataReady interupt as PULL_UP input, and attach interupt:
  pinMode(dataReadyPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(dataReadyPin), dataReady_ISR, FALLING); // LOW
}


//======================//
//======  LOOP  ========//
//======================//
void loop() {
  // check if writePeriod passed, and output to Serial if so:
  unsigned long currentMillis = millis();

  if ((currentMillis - previousWriteMillis) > writePeriodMillis) {
    // dettach interrupt so write won't be messed up
    detachInterrupt(digitalPinToInterrupt(dataReadyPin));
    previousWriteMillis = currentMillis;
    writeToSerial();
    // re-attach interrupt
    attachInterrupt(digitalPinToInterrupt(dataReadyPin), dataReady_ISR, FALLING); // LOW
  }

  if ((currentMillis - previousSyncMillis) > syncPeriodMillis) {
    // dettach interrupt so sync won't be messed up
    detachInterrupt(digitalPinToInterrupt(dataReadyPin));
    previousSyncMillis = currentMillis;
    SPI.beginTransaction(spiSettings);
    syncADE7913();
    SPI.endTransaction();
    Serial.println("ADE7913 Synced");
    // re-attach interrupt
    attachInterrupt(digitalPinToInterrupt(dataReadyPin), dataReady_ISR, FALLING);  // LOW
  }
}

// Write a register to ADE7913, assume SPI.beginTransaction already called
void writeADE7913(byte writeTo, byte writeMsg) {
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(writeTo);
  SPI.transfer(writeMsg);
  digitalWrite(slaveSelectPin, HIGH);
}

// Write a register to ADE7913, assume SPI.beginTransaction already called
// include read-back test, and loop until correctly set (or nMaxWriteTry reached)!
boolean writeADE7913_check(byte writeTo, byte writeMsg, byte readFrom) {
  boolean success = false;
  int nTry = 0;
  do {
    digitalWrite(slaveSelectPin, LOW);
    SPI.transfer(writeTo);
    SPI.transfer(writeMsg);
    digitalWrite(slaveSelectPin, HIGH);
    delay(1);
    // Read-back register to confirm write success
    byte readBack[1];
    readMultBytesADE7913(readFrom, readBack, 1);
    success = (readBack[0] == writeMsg);
    nTry++;
  } while ((!success) && nTry < nMaxWriteTry);

  return success;
}

// Read multiple bytes from ADE7913, assume SPI.beginTransaction already called
// COMMENTED OUT: (try multiple times till a non-all-ones answer found)
void readMultBytesADE7913(byte readFrom, volatile byte readTo[], int nBytes) {
  int idx = nBytes - 1;                  // fill up bytes from end first
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(readFrom);
  while (idx >= 0) {
    readTo[idx] = SPI.transfer(DUMMY_MSG);
    idx--;
  }
  digitalWrite(slaveSelectPin, HIGH);
}

// Sync the ADE7913 (NB: this will need to be made fancier once I'm using multiple chips)
// assumes SPI.beginTransaction already called
byte syncADE7913() {
  // Unlock the config registers:
  writeADE7913(LOCK_KEY_WRITE, UNLOCK_BYTE);

  // Broadcast Sync (write 0x01 to SYNC_SNAP):
  // (this step will need to be cleverer when using multiple chips)
  writeADE7913(SYNC_SNAP_WRITE, 0b00000001);

  // Relock the registers:
  writeADE7913(LOCK_KEY_WRITE, LOCK_BYTE);
}

// Interupt Service Routing to run when "dataReadyPin" pin goes low:
void dataReady_ISR() {
  //  Serial.println("Interupt!");
  microsBetweenReads = micros() - microsPreviousRead;
  microsPreviousRead = micros();
  nReads++;       // keep a track of No. of reads

  // We have ADC data! => make the read:
  SPI.beginTransaction(spiSettings);


  // Test re-read registers of 'known' value:
  // readMultBytesADE7913(CONFIG_READ, CONFIG, 1); delayMicroseconds(rdDelay);
  //  readMultBytesADE7913(TEMPOS_READ, TEMPOS, 1); delayMicroseconds(rdDelay);
  //  readMultBytesADE7913(CONFIG_READ, CONFIG, 1); delayMicroseconds(rdDelay);
  //  readMultBytesADE7913(TEMPOS_READ, TEMPOS, 1); delayMicroseconds(rdDelay);
  /*
      // READ VALUES INDIVIDUALLY
      readMultBytesADE7913(IWV_READ, IWV.bytes, 3); delayMicroseconds(rdDelay);
      readMultBytesADE7913(V1WV_READ, V1WV.bytes, 3); delayMicroseconds(rdDelay);
      readMultBytesADE7913(V2WV_READ, V2WV.bytes, 3); delayMicroseconds(rdDelay);
      readMultBytesADE7913(ADC_CRC_READ, ADC_CRC, 2); delayMicroseconds(rdDelay);
      readMultBytesADE7913(STATUS0_READ, STATUS0, 1); delayMicroseconds(rdDelay);
      readMultBytesADE7913(CNT_SNAPSHOT_READ, CNT_SNAPSHOT, 2); delayMicroseconds(rdDelay);
  */
  // READ VALUES IN BURST MODE:
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(IWV_READ);
  IWV.bytes[2] = SPI.transfer(DUMMY_MSG);
  IWV.bytes[1] = SPI.transfer(DUMMY_MSG);
  IWV.bytes[0] = SPI.transfer(DUMMY_MSG);

  V1WV.bytes[2] = SPI.transfer(DUMMY_MSG);
  V1WV.bytes[1] = SPI.transfer(DUMMY_MSG);
  V1WV.bytes[0] = SPI.transfer(DUMMY_MSG);

  V2WV.bytes[2] = SPI.transfer(DUMMY_MSG);
  V2WV.bytes[1] = SPI.transfer(DUMMY_MSG);
  V2WV.bytes[0] = SPI.transfer(DUMMY_MSG);

  ADC_CRC[1] = SPI.transfer(DUMMY_MSG);
  ADC_CRC[0] = SPI.transfer(DUMMY_MSG);

  STATUS0[0] = SPI.transfer(DUMMY_MSG);

  //CNT_SNAPSHOT_burst[1] = SPI.transfer(DUMMY_MSG);
  //CNT_SNAPSHOT_burst[0] = SPI.transfer(DUMMY_MSG);

  digitalWrite(slaveSelectPin, HIGH);

  SPI.endTransaction();

  microsForBurstRead = micros() - microsPreviousRead;
  return;
}

// Write current values to Serial:
void writeToSerial() {

  // extend the sign bits of each 3-byte word:
  extendSignBit(&IWV);
  extendSignBit(&V1WV);
  extendSignBit(&V2WV);

  // Output the No. of good reads made since last serial write:
  Serial.print("nReads: "); Serial.println(nReads);
  // reset number of reads to zero:
  nReads = 0;
  Serial.print("Expected: "); Serial.println(8 * writePeriodMillis);
  Serial.print("Burst read took [us]: "); Serial.println(microsForBurstRead);
  Serial.print("Time between reads [us]: "); Serial.println(microsBetweenReads);

  // Calculate Temperature in Deg C:
  double gain = 8.72101e-5;
  double temp = gain * ((int32_t)V2WV.value) + 8.72101e-5 * ((int8_t) TEMPOS[0]) * pow(2, 11) - 306.47;

  // Write out current values in registers:
  Serial.print("CONFIG= "); Serial.println(CONFIG[0], BIN);
  Serial.println("========== Burst Reads ==========");
  Serial.print("IWV= "); Serial.print(IWV.value, DEC); Serial.print(" "); Serial.println(IWV.value, BIN);
  Serial.print("V1WV= ");  Serial.print(V1WV.value, DEC); Serial.print(" "); Serial.println(V1WV.value, BIN);
  Serial.print("V2WV= ");  Serial.print(V2WV.value, DEC); Serial.print(" "); Serial.println(V2WV.value, BIN);
  Serial.print("Temp= "); Serial.print(temp); Serial.println(" deg C");
  Serial.print("ADC_CRC= ");  Serial.print(ADC_CRC[1], BIN); Serial.print(" "); Serial.println(ADC_CRC[0], BIN);
  Serial.print("STATUS0= ");  Serial.println(STATUS0[0], BIN);
  Serial.print("CNT_SNAPSHOT= ");  Serial.print(CNT_SNAPSHOT[1], BIN); Serial.print(" "); Serial.println(CNT_SNAPSHOT[0], BIN);
  Serial.println("=================================");
  Serial.print("TEMPOS= ");  Serial.print((int8_t) TEMPOS[0], DEC);
  Serial.println(" "); Serial.println(" ");
}

// Extend sign byte of 3-byte word to create signed 4-byte word
void extendSignBit(volatile threeByteWord *wordIn) {
  boolean signBit = bitRead(wordIn->bytes[2], 7);
  if (signBit == 0) {
    wordIn->bytes[3] = 0b00000000;
  } else {
    wordIn->bytes[3] = 0b11111111;
  }
}

