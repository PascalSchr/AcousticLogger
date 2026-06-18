#include <SdFat.h>
#include <TimeLib.h>



// ------------------------------------------
// -- ADC Pins --
const int PIN_CONVST = 2;
const int PIN_WR = 3;
const int PIN_RESET = 4;
const int PIN_RD = 5;
const int PIN_CS = 6;
const int PIN_BUSY = 7;
const int PIN_FIRSTDATA = 8;

// -- Pins --
const int ledPin = 13;

// ------------------------------------------
// -- ADC-Pins -> Teensy-Pins -> GPIO6 BIT --
// DB0  → pin 19 → GPIO6 bit 16  
// DB1  → pin 18 → GPIO6 bit 17  
// DB2  → pin 14 → GPIO6 bit 18
// DB3  → pin 15 → GPIO6 bit 19
// DB4  → pin 40 → GPIO6 bit 20
// DB5  → pin 41 → GPIO6 bit 21
// DB6  → pin 17 → GPIO6 bit 22
// DB7  → pin 16 → GPIO6 bit 23
// DB8  → pin 22 → GPIO6 bit 24
// DB9  → pin 23 → GPIO6 bit 25
// DB10 → pin 20 → GPIO6 bit 26
// DB11 → pin 21 → GPIO6 bit 27
// DB12 → pin 38 → GPIO6 bit 28
// DB13 → pin 39 → GPIO6 bit 29
// DB14 → pin 26 → GPIO6 bit 30
// DB15 → pin 27 → GPIO6 bit 31

const uint32_t GPIO6_DATA_MASK = 0xFFFF0000;

// All data bits sit in the upper 16 bits — one shift extracts the sample
inline uint16_t FASTRUN extractBits(uint32_t raw){
  return (uint16_t)(raw >> 16);
}


// ------------------------------------------
// -- SD --
SdFs sd;
FsFile file;
char filename[100] = "";


// ------------------------------------------
// -- Buffers --
const uint32_t SAMPLES_PER_BUFFER = 4096;
const uint32_t BUF_BYTES = SAMPLES_PER_BUFFER * 2 * sizeof(uint16_t);
const int dataPins[16] = {19,18,14,15,40,41,17,16,22,23,20,21,38,39,26,27};

DMAMEM uint16_t buf[2][SAMPLES_PER_BUFFER * 2];

uint32_t swapTime = 0;
uint32_t fileTimeLength = 60*1000*10;   //10 minutes

volatile uint8_t writeBuf = 0;
volatile uint32_t bufPos = 0;
volatile bool flushReady = false;
volatile bool overrun = false;

volatile uint32_t sampleCounter    = 0;    // current sample index
volatile uint16_t fileCounter = 0;



// ------------------------------------------
// -- stats --
uint32_t totalSamples = 0;
uint32_t lastStatsTime = 0;  
uint32_t filestarttime = 0;
uint32_t runtime = 0;


// ------------------------------------------
// -- sample rate --
const uint32_t SAMPLE_INTERVAL_US = 5;

IntervalTimer sampleTimer;


// ------------------------------------------
// -- RTC --
time_t RTCTime;
time_t get_TeensyTime(void) {
  return Teensy3Clock.get();
}


void FASTRUN sampleISR() {
    digitalWriteFast(PIN_CONVST, HIGH);
    delayNanoseconds(25);
    digitalWriteFast(PIN_CONVST, LOW);

    // Replace micros() timeout with a counter (~5000 iterations ≈ 50µs at 600MHz)
    uint32_t timeout = 5000;
    while (digitalReadFast(PIN_BUSY)) {
        if (--timeout == 0) return;
    }

    digitalWriteFast(PIN_CS, LOW);

    digitalWriteFast(PIN_RD, LOW);
    delayNanoseconds(25);
    uint16_t ch0 = extractBits(GPIO6_DR);
    digitalWriteFast(PIN_RD, HIGH);

    delayNanoseconds(10);
    digitalWriteFast(PIN_RD, LOW);
    delayNanoseconds(25);
    uint16_t ch1 = extractBits(GPIO6_DR);
    digitalWriteFast(PIN_RD, HIGH);

    digitalWriteFast(PIN_CS, HIGH);

    uint8_t b = writeBuf;
    buf[b][bufPos * 2]     = ch0;
    buf[b][bufPos * 2 + 1] = ch1;
    if (++bufPos >= SAMPLES_PER_BUFFER){
        if (flushReady) overrun = true;
        bufPos     = 0;
        flushReady = true;
        writeBuf  ^= 1;
    }
}


void enterRegisterMode() {
    IMXRT_GPIO6.GDIR |= GPIO6_DATA_MASK;

    // DB15=1 (read bit), address=0x00 — single RD pulse enters register mode
    uint32_t dr = IMXRT_GPIO6.DR & ~GPIO6_DATA_MASK;
    IMXRT_GPIO6.DR = dr | (1UL << 31);

    digitalWriteFast(PIN_CS, LOW);
    digitalWriteFast(PIN_RD, LOW);
    delayNanoseconds(25);
    digitalWriteFast(PIN_RD, HIGH);
    digitalWriteFast(PIN_CS, HIGH);

    IMXRT_GPIO6.GDIR &= ~GPIO6_DATA_MASK;
    IOMUXC_GPR_GPR26 |= GPIO6_DATA_MASK;
    delayMicroseconds(1);
}






void exitRegisterMode() {

    IMXRT_GPIO6.GDIR |= GPIO6_DATA_MASK;

    // Now clear all DB pins — this actually drives zeros onto the bus
    IMXRT_GPIO6.DR &= ~GPIO6_DATA_MASK;

    Serial.println(IMXRT_GPIO6.DR,BIN);
    delayNanoseconds(25);

    digitalWriteFast(PIN_CS, LOW);
    digitalWriteFast(PIN_WR, LOW);
    delay(25);
    digitalWriteFast(PIN_WR, HIGH);
    digitalWriteFast(PIN_CS, HIGH);

    IMXRT_GPIO6.GDIR &= ~GPIO6_DATA_MASK;
    delayMicroseconds(1);
}

void writeRegister(uint8_t address, uint8_t value) {
    IMXRT_GPIO6.GDIR |= GPIO6_DATA_MASK;
    //Serial.println(IMXRT_GPIO6.GDIR, BIN);

    // DB15=0 (write), DB[14:8]=address, DB[7:0]=data — all in one WR pulse
    // DB15 maps to GPIO6 bit 31, DB[14:8] to bits 30:24, DB[7:0] to bits 23:16
    uint32_t word = ((uint32_t)(address & 0x7F) << 24)  // DB[14:8]
                  | ((uint32_t)value << 16);              // DB[7:0]
    //Serial.println(word, BIN);
    uint32_t dr = IMXRT_GPIO6.DR & ~GPIO6_DATA_MASK;
    IMXRT_GPIO6.DR = dr | word;
    //Serial.println(IMXRT_GPIO6.DR,BIN);
    delay(25);
    digitalWriteFast(PIN_CS, LOW);
    digitalWriteFast(PIN_WR, LOW);
    delay(25);
    digitalWriteFast(PIN_WR, HIGH);
    digitalWriteFast(PIN_CS, HIGH);
    //Serial.println(IMXRT_GPIO6.DR, BIN);
    IMXRT_GPIO6.GDIR &= ~GPIO6_DATA_MASK;
}

uint8_t readRegister(uint8_t address) {
    IMXRT_GPIO6.GDIR |= GPIO6_DATA_MASK;
    //Serial.println(IMXRT_GPIO6.GDIR, BIN);

    // DB15=1 (read), DB[14:8]=address, DB[7:0]=don't care
    uint32_t word = (1UL << 31)                          // DB15=1
                  | ((uint32_t)(address & 0x7F) << 24);  // DB[14:8]

    //Serial.println(word,BIN);
    uint32_t dr = IMXRT_GPIO6.DR & ~GPIO6_DATA_MASK;
    //Serial.println(dr|word,BIN);
    //Serial.println(IMXRT_GPIO6.DR, BIN);

    delay(25);
    digitalWriteFast(PIN_CS, LOW);
    digitalWriteFast(PIN_WR, LOW);
    delay(25);
    IMXRT_GPIO6.DR = dr | word;
    delay(25);
    digitalWriteFast(PIN_WR, HIGH);
    digitalWriteFast(PIN_CS, HIGH);
    delay(25);




        // --- Switch to input so ADC can drive the bus ---
    IMXRT_GPIO6.GDIR &= ~GPIO6_DATA_MASK;

    digitalWriteFast(PIN_CS, LOW);
    digitalWriteFast(PIN_RD, LOW);
    delay(25);
    uint32_t raw = GPIO6_DR; // read data on the bus
    delay(25);
    digitalWriteFast(PIN_RD, HIGH);
    digitalWriteFast(PIN_CS, HIGH);
    IMXRT_GPIO6.GDIR &= ~GPIO6_DATA_MASK;

    // DB[7:0] sit in GPIO6 bits 23:16
    return (uint8_t)((raw >> 16) & 0xFF);
}
// ------------------------------------------
// -- setup function --
void setup(){
  pinMode(ledPin, OUTPUT); // onboard status LED 
  setSyncProvider(get_TeensyTime);


  Serial.begin(115200); // <- 115200 highest stable Baud-rate
  while (!Serial && millis()<2000); //wait for serial and at least 2s 

  // -- set control pins --
  pinMode(PIN_CONVST, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RD, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_WR, OUTPUT);


  // -- reset ADC --
  digitalWriteFast(PIN_RESET, HIGH);
  Serial.println("resetting...");
  delay(25);
  digitalWriteFast(PIN_RESET, LOW);
  delay(1000);
  Serial.println("resetting done!");

  // -- set up default for output pins --
  digitalWriteFast(PIN_CONVST, LOW);
  digitalWriteFast(PIN_CS, HIGH);
  digitalWriteFast(PIN_RD, HIGH);
  digitalWriteFast(PIN_WR, HIGH);

  // -- set register as GPIO6 (fast GPIO) --
  Serial.printf("GPR26 before: 0x%08X\n", IOMUXC_GPR_GPR26);
  IOMUXC_GPR_GPR26 |= GPIO6_DATA_MASK;
  IOMUXC_GPR_GPR26 &= GPIO6_DATA_MASK;
  Serial.printf("GPR26 after:  0x%08X\n", IOMUXC_GPR_GPR26);  //https://www.pjrc.com/teensy/IMXRT1060RM_rev3_annotations.pdf teensy documentation, 11.3.27 GPR26 
  
  readRegister(0x07);

  writeRegister(0x03, 0x00);
  delay(25);
  writeRegister(0x07, 0x03);
  delay(25);
  writeRegister(0x08, 0x01);
  delay(25);

  Serial.printf("Range reg:      0x%02X (expect 0x00)\n", readRegister(0x03));  // 0x00 for +-2.5 V single ended,  0x88 for +-5 V differential  0x55 for 0 - 5 V single ended
  delay(25);
  Serial.printf("Bandwidth reg:  0x%02X (expect 0x03)\n", readRegister(0x07));  // 0x03 for high bandwidth mode on channel 0 and 1
  delay(25);
  Serial.printf("Oversample reg: 0x%02X (expect 0x01)\n", readRegister(0x08));  // 0x01 for oversampling = 2

  exitRegisterMode();
  delay(25);


  // -- set up data bus as inputs -- 
  const int dataPins[16] = {19,18,14,15,40,41,17,16,22,23,20,21,38,39,26,27};
  for (int i = 0; i>16; i++){
    pinMode(dataPins[i], INPUT);
  }


  // -- set data pins of GPIO6 as inputs --
  IMXRT_GPIO6.GDIR &= (~GPIO6_DATA_MASK); //& bitwise and, ~ invert, 0:input(default), 1:output


  // -- reset ADC --

  // -- initialize SD card --
  if (!sd.begin(SdioConfig(FIFO_SDIO))){
    Serial.println("SD init failed!"); 
    while(1){            
      digitalWrite(ledPin, HIGH);   // set the LED on
      delay(500);                  // wait for a second
      digitalWrite(ledPin, LOW);    // set the LED off
      delay(1000); 
    }
  }
  sprintf(filename, "File_%d-%d_%d-%d-%d.bin", month(),day(),hour(),minute(),second()); //filename example File_1-1_10-30-45.bin
  file = sd.open(filename, O_WRITE | O_CREAT | O_TRUNC);
  
  if (!file){
    Serial.println("File open failed!"); 
    while(1){            
      digitalWrite(ledPin, HIGH);   // set the LED on
      delay(100);                  // wait for a second
      digitalWrite(ledPin, LOW);    // set the LED off
      delay(1000); 
    }
  }
  file.preAllocate(uint64_t (10*200000*4));
  Serial.println("Starting...");
  filestarttime = now();
  runtime = now();
  sampleTimer.begin(sampleISR, SAMPLE_INTERVAL_US);   //ISR starting for polling the ADC
  sampleTimer.priority(0);          // ← just add this line
}



// ------------------------------------------
// -- Main Loop --

void loop(){
  if (overrun){
    Serial.println("OVERRUN - SD write too slow!");
    overrun = false;
  }

  if (flushReady){
    flushReady = false;
    uint8_t readBuf = writeBuf ^ 1;
    file.write(buf[readBuf], BUF_BYTES);
    totalSamples += SAMPLES_PER_BUFFER;
  }


  uint32_t now = millis();
  /*
  if (now - lastStatsTime>3000){
    float khz = totalSamples / 3000.0f;
    Serial.printf("Logging rate: %.2f kHz  buf0[0]=%d buf1[0]=%d\n", khz, (int16_t)buf[0][0], (int16_t)buf[1][0]);
    totalSamples = 0;
    lastStatsTime = now;
  }*/



  if (now - filestarttime>fileTimeLength) {
    fileCounter++;
    if (fileCounter>500)  {
        Serial.println("all files full!");
        Serial.print("finished...");
        file.close();
        while(1){
            digitalWrite(ledPin, HIGH);   // set the LED on
            delay(500);                  // wait for a second
            digitalWrite(ledPin, LOW);    // set the LED off
            delay(500);   
        }
    }
    /*Serial.println("file full! ...new file...");
    /*file.truncate();
    file.sync();*/
    file.close();
    sprintf(filename, "File_%d-%d_%d-%d-%d.bin", month(),day(),hour(),minute(),second());
    file = sd.open(filename, O_WRITE | O_CREAT | O_TRUNC);
    /*if (!file) {
      Serial.println("File2 open failed!"); while (1);
    }
    file.preAllocate(uint64_t (10*200000*4)); // pre-alloc 256 MB*/
    filestarttime = now;
  } 
  
  
}









































