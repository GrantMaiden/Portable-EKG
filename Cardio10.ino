//Grant Maiden  1640958
//Dang Le       1570906
//Lab 7

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include <Wire.h>
#include <font_AwesomeF000.h>
#include <font_ArialBlack.h>
#include <font_ArialBoldItalic.h>

#include "ILI9341_t3.h"
#include <SD.h>
//#include "SdFat.h"
#include <SerialFlash.h>
//#define PDB_CONFIG (PDB_SC_TRGSEL(15)|PDB_SC_PDBEN|PDB_SC_PDBIE|PDB_SC_CONT|PDB_SC_PRESCALER(7)|PDB_SC_MULT(1))

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10
#define SD_CS 4

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

//Sd SD;

File myFile;
int firstDigit = 0;
int secondDigit = 0;

// Pin definitions
int intPin = 12;                  // This can be changed, 2 and 3 are the Arduinos ext int pins
int data = 0;                     // Takes the heartbeat input
int yPos[320];                    // Store 320 heartbeat inputs as an array 
int yOldPos[320];                 // Store old 320 heartbeat inputs as an array 
int recordSD[7500];               // Store all heartbeat input throughout the whole recording
int hb_counter = 0;               // Increment everytime a heartbeat is detected
int hb_timer;                     // A timer to calculate heart rate
int idx = 0;                      // index for recordSD[] array
int stopTimer = 0;                // A variable used to stop timer when the recording is paused
int stopEcg = 0;                  // Timer to stop the recording after 30s mark
int startTimerOld = 0;            // Another variable used to stop timer when the recording is paused
int oldData = 120;                // Store previous heartbeat data
int xPosition = 330;              // x position on the LCD
int menuSelect = 2;               // Continue/Exit menu select
int chooseFileCurrentNum = 0;     // Current file number in the file choosing menu
int hb_delta_t = 0;               // Timer to implement the beating heart

boolean trig = true;                   // Flag to pause the recording
boolean oldTrig = true;               
boolean oldTrig2 = true;
boolean timerTrig = true;
boolean bool_heartBeatFlag = false;
boolean bool_hbTimerFlag = false;
boolean sdFlag = false;                // True when the recording is done and we need to store the data to the SD
boolean resetFlag = false;             // True when we need to reset the recording after 30s
boolean milliFlag = false;             // Used to start the timer at the beginning of the recording
boolean stopTimerFlag = false;         // Used to stop the timer during the pause
boolean startStabilize = true;         // Used to draw the Stablizing screen
boolean whiteFlag = true;              // Used to switch from the Stablizing screen to the recording screen
boolean sdCardFull = false;            // True when the number of SD files goes to 100
boolean writeChoice = true;            // True when "Record new" is chosen, false when "Read from SD" is chosen
boolean chose = false;                 // True when you made a choice in the menu
boolean SDplaybackFlag = false;        // True after choosing the file to playback
boolean setupVariablesFlag = false;    // Flag used to re-initialize all variables after 30s recording
boolean initializeDrawMenu = false;
boolean playBackMenuFlag = false;
boolean chooseFileInitialize = false;
boolean redrawFlag = false;
boolean changeADCPort = false;
boolean ADC_ECG_Read = false;
boolean displayFlag = false;
boolean hb_flash = false;

boolean StableTrue = false;  // True after 10 heartbeats are detected
int millisStart = 0;         // Start the timer

int diff = 4095;
int sensorMin = 4095;        // minimum sensor value
int sensorMax = 0;           // maximum sensor value

char filename[] = "rec00.txt";
char readFilename[] = "rec00.txt";
char headername[] = "SDECG00";
int num = 0;                 // Keep track of the number of files in SD card

// For 30s timer
int count = 0;
int deltat = 0;

int PotentiometerValue = 0;
int chooseVal = 0;
int fileNum = -1;
int SDidx = 0;               // Used to keep track of the index in recordSD[] array in playback

/* Used to check to see of those values changed */
int oldInput = 0;
int oldNum = 0;
int oldSDidx = 0;
int oldHB = 0;

boolean numFlag = false;    // Flag to make sure to save the number of a full number is read from the SD card
int mem = 0;                // Used to store the number everytime we finish reading a whole number from the sd card
int byteCount = 0;          // Keep track of the number of bytes that we've read from the SD card

//ADC initialize setup BEGIN +++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

uint8_t ledOn = 0;
uint16_t samples[16];

static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

/*
  ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(2)         Single ended 10 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

/*
  PDB_SC_TRGSEL(15)        Select software trigger
  PDB_SC_PDBEN             PDB enable
  PDB_SC_PDBIE             Interrupt enable
  PDB_SC_CONT              Continuous mode
  PDB_SC_PRESCALER(7)      Prescaler = 128
  PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
                    | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

// 48 MHz / 128 / 10 / 1 Hz = 37500
#define PDB_PERIOD (F_BUS / 128 / 10 / 1 / 250)

//ADC initialize END ------------------------------------------------------------------------------------

//Filter initialize Start +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <Filters.h>
FilterOnePole filterOneLowpass( LOWPASS, 10 );   // create a one pole (RC) lowpass filter
FilterOnePole highPassFilter( HIGHPASS, .1);

//Filter initialize END ------------------------------------------------------------------------------------

//HB detection initialize Start +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int PRI_time = 0;
int QRS_time = 0;
int HB_avg = 0;
int HB_add_tally = 0;
int PRI_avg = 0;
int PRI_add_tally = 0;
int PRI_add_tally2 = 0;
int QRS_avg = 0;
int QRS_add_tally = 0;
int PAC_counter = 0;
int previous_HB_length = 0;

boolean bool_PAC_detected = false;
boolean bool_BRADY = false;
boolean bool_TACHY = false;

//HB detection initialize END ------------------------------------------------------------------------------------


void setup()
{
  Wire.begin();
  Serial.begin(38400);
  //while (!Serial);
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  //pinMode(14, OUTPUT);
  adcInit();
  pdbInit();
  dmaInit();

  tft.begin();         // Initialize the display
  tft.setRotation(1);  //  0 or 2) width = width, 1 or 3) width = height, swapped etc.

  boolean success;

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ) {
    error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'DanGrant HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=DanGrant HRM")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
  if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();

  for (int i = 0; i < 320; i++)
    yPos[i] = 0;

  //Count number of files on SD card
  while (SD.exists(filename)) {
    num = num + 1;
    firstDigit = num / 10;
    secondDigit = num % 10;
    filename[3] = firstDigit + '0';
    filename[4] = secondDigit + '0';
    headername[5] = firstDigit + '0';
    headername[6] = secondDigit + '0';
  }

  pinMode(14, INPUT);
  pinMode(17, INPUT);

  // Start device display with ID of sensor
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE); // Set pixel color; 1 on the monochrome screen
  tft.setTextSize(10);
  tft.setFont(ArialBlack_20);
  
  delay(1000);

  tft.fillScreen(ILI9341_WHITE);      // clears the screen and buffer

  delay(1000);

  //analogReadResolution(12);
  count = millis();
}

void loop()
{
  // Setup Variables for proper program operation
  if (setupVariablesFlag == false) {
    setupVariables();
    setupVariablesFlag = true;
  }

  // Read PotentionMeterValue
  if (changeADCPort == false) {
    ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[4];
    delay(5);
    changeADCPort = true;
  }
  PotentiometerValue = ADC0_RA;

  if (!chose) {
    drawTopLevelMenu(PotentiometerValue / 2048);
    chooseVal = digitalRead(17);
    if (chooseVal == LOW) {
      chose = true;
      delay(250);
    }
  } else {
    // Show recording
    if (SDplaybackFlag) {
      readDataModule();
    }

    // Goto ChooseFile funciton (If Read Has been selected at top level)
    if (!writeChoice && !SDplaybackFlag) {
      chooseFile(PotentiometerValue);
      int fileVal = digitalRead(17);
      if (fileVal == LOW && fileNum < 0) {
        fileNum = chooseFileCurrentNum;
        SDplaybackFlag = !SDplaybackFlag;
        for (int i = 0; i < 7500; i++)
          recordSD[i] = 0;
        firstDigit = fileNum / 10;
        secondDigit = fileNum % 10;
        readFilename[3] = firstDigit + '0';
        readFilename[4] = secondDigit + '0';
        idx = 0;
        readfromSD();

        tft.fillRect(0, 90 , 320, 150, ILI9341_DARKGREEN);
        tft.fillRect(0, 0 , 320, 90, ILI9341_WHITE);

        tft.setRotation(3);
        // Draw Border
        tft.fillRect(0, 150, 320, 5, ILI9341_BLACK);
        tft.fillRect(0, 150, 5, 90, ILI9341_BLACK);
        tft.fillRect(0, 235, 320, 5, ILI9341_BLACK);
        tft.fillRect(315, 150, 5, 90, ILI9341_BLACK);

        // Draw checkbox
        tft.fillRect(0, 210, 320, 2, ILI9341_BLACK);
        for (int i = 1; i <= 3; i++) {
          tft.drawLine(80 * i, 210, 80 * i, 240, ILI9341_BLACK);
        }

        // Draw all the texts
        tft.setTextColor(ILI9341_BLACK);
        tft.setFont(AwesomeF000_14);
        tft.setCursor(7, 165); tft.print((char)4);
        tft.setTextColor(ILI9341_BLACK);
        tft.setFont(ArialBlack_12);
        tft.setTextColor(ILI9341_BLUE);
        tft.setCursor(30, 170); tft.print(HB_avg);
        tft.setFont(Arial_10_Bold_Italic);
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(65, 172); tft.print("bpm");
        tft.setFont(ArialBlack_12);
        tft.setCursor(105, 170); tft.print("QRS");
        tft.setTextColor(ILI9341_BLUE);
        tft.setCursor(148, 170); tft.print(QRS_avg);
        tft.setFont(Arial_10_Bold_Italic);
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(185, 172); tft.print("ms");
        tft.setFont(ArialBlack_12);
        tft.setCursor(215, 170); tft.print("PRI");
        tft.setTextColor(ILI9341_BLUE);
        tft.setCursor(252, 170); tft.print(PRI_avg);
        tft.setFont(Arial_10_Bold_Italic);
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(289, 172); tft.print("ms");

        tft.setFont(ArialBlack_14);
        tft.setCursor(20, 192); tft.print("PVC");
        tft.setCursor(100, 192); tft.print("PAC");
        tft.setCursor(170, 192); tft.print("BRDY");
        tft.setCursor(250, 192); tft.print("TCHY");

        // draw X if PAC is detected
        if (bool_PAC_detected) {
          tft.setFont(ArialBlack_12);
          tft.setTextColor(ILI9341_BLUE);
          tft.setCursor(115, 215); tft.print("X");
        }

        // draw X if Bradycardia is detected
        if (bool_BRADY) {
          tft.setFont(ArialBlack_12);
          tft.setTextColor(ILI9341_BLUE);
          tft.setCursor(195, 215); tft.print("X");
        }

        // draw X if Tachycardia is detected
        if (bool_TACHY) {
          tft.setFont(ArialBlack_12);
          tft.setTextColor(ILI9341_BLUE);
          tft.setCursor(275, 215); tft.print("X");
        }
        delay(200);
      }

      // Goto record ECG (Write to SD)
    } else if (writeChoice) {
      if (ADC_ECG_Read == false) {
        ADC_ECG_Read = true;
        ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[6];
        delay(5);
      }

      // Start the timer
      if (StableTrue && !milliFlag) {
        Serial.println("running");
        millisStart = millis();
        milliFlag = true;
      }

      // Draw the Stablizing screen
      if (startStabilize) {
        tft.setRotation(3);
        tft.fillScreen(ILI9341_DARKGREEN);
        tft.setTextColor(ILI9341_BLACK);
        tft.fillRect(0, 0, 320, 5, ILI9341_WHITE);
        tft.fillRect(0, 0, 5, 240, ILI9341_WHITE);
        tft.fillRect(0, 235, 320, 5, ILI9341_WHITE);
        tft.fillRect(315, 0, 5, 240, ILI9341_WHITE);
        tft.setTextSize(3);
        tft.setCursor(40, 90);
        tft.setFont(ArialBlack_16);
        tft.println("INPUT STABILIZING");
        tft.setCursor(60, 110);
        tft.println("PLEASE WAIT...");
        tft.setRotation(1);
        startStabilize = false;
        whiteFlag = true;
      }

      // Write current data to display (Write Mode)
      if (StableTrue && !sdCardFull) {
        displayECG();
        noInterrupts();
        updateInfo();
        interrupts();
      }

      // Store the data to SD card
      if (sdFlag) {
        storeToSD();
      }
    }
  }
}

// Draw Top-level read/write menu
void drawTopLevelMenu(int input) {
  tft.setRotation(3);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  int tempVal = input - oldInput;
  // Draw the menu
  if (initializeDrawMenu == false) { // Initialize Screen
    tft.setFont(ArialBlack_20);
    tft.fillRect(0, 0, 320, 120, ILI9341_DARKGREEN);
    tft.fillRect(0, 120, 320, 120, ILI9341_DARKGREEN);
    tft.fillRect(0, 110, 320, 20, ILI9341_WHITE);
    tft.fillRect(0, 0, 320, 5, ILI9341_WHITE);
    tft.fillRect(0, 235, 320, 5, ILI9341_WHITE);
    tft.fillRect(0, 115, 320, 10, ILI9341_BLACK);
    tft.setCursor(70, 50); tft.println("RECORD NEW");
    tft.setCursor(70, 170); tft.println("READ FROM SD");
    tft.setTextColor(ILI9341_BLUE);
    initializeDrawMenu = true;
    if (input == 0) { // Initialize Screen Select (Record)
      writeChoice = true;
      tft.fillRect(0, 10, 60, 100, ILI9341_DARKGREEN);
      tft.fillRect(0, 130, 60, 100, ILI9341_DARKGREEN);
      tft.setFont(AwesomeF000_32);
      tft.setCursor(10, 38);
      tft.print((char)88);
    } else { // Initialize Screen Select (Read)
      tft.fillRect(0, 10, 60, 100, ILI9341_DARKGREEN);
      tft.fillRect(0, 130, 60, 100, ILI9341_DARKGREEN);
      writeChoice = false;
      tft.setFont(AwesomeF000_32);
      tft.setCursor(10, 153);
      tft.print((char)88);
    }
  }

  // Draw the check symbol
  tft.setTextColor(ILI9341_BLUE);
  if (tempVal != 0) {
    switch (input) {
      case 0:
        writeChoice = true;
        tft.fillRect(0, 10, 60, 100, ILI9341_DARKGREEN);
        tft.fillRect(0, 130, 60, 100, ILI9341_DARKGREEN);
        tft.setFont(AwesomeF000_32);
        tft.setCursor(10, 38);
        tft.print((char)88);
        break;
      case 1:
        tft.fillRect(0, 10, 60, 100, ILI9341_DARKGREEN);
        tft.fillRect(0, 130, 60, 100, ILI9341_DARKGREEN);
        writeChoice = false;
        tft.setFont(AwesomeF000_32);
        tft.setCursor(10, 153);
        tft.print((char)88);
        break;
    }
  }
  tft.setFont(ArialBlack_20);
  oldInput  = input;
  tft.setRotation(1);
}

// Draw the choosing file menu
void chooseFile(int input) { // Controls and initializes choose file screen
  int potVal = input;
  if (1600 <= input && input < 2500) {
    potVal = 0;
  }
  if (input < 1600) {
    potVal = (5 - (potVal / 320)) * -1;
  }
  if (2500 <= input) {
    potVal = (potVal - 2495) / 319;
  }

  chooseFileCurrentNum = chooseFileCurrentNum + potVal;
  if (chooseFileCurrentNum < 1) {
    chooseFileCurrentNum = 0;
  }
  if (chooseFileCurrentNum > num - 1) {
    chooseFileCurrentNum = num - 1;
  }

  tft.setRotation(3);
  int tempVal = oldNum - chooseFileCurrentNum;
  if (chooseFileInitialize == false) {
    chooseFileInitialize = true;
    tft.fillScreen(ILI9341_WHITE);
    tft.setTextColor(ILI9341_BLACK);
    tft.fillRect(0, 40, 320, 60, ILI9341_DARKGREEN);
    tft.fillRect(0, 40, 320, 5, ILI9341_BLACK);
    tft.fillRect(0, 95, 320, 5, ILI9341_BLACK);

    tft.fillRect(70, 130, 180, 60, ILI9341_DARKGREEN);
    tft.fillRect(70, 130, 180, 5, ILI9341_BLACK);
    tft.fillRect(70, 130, 5, 60, ILI9341_BLACK);
    tft.fillRect(70, 185, 180, 5, ILI9341_BLACK);
    tft.fillRect(245, 130, 5, 60, ILI9341_BLACK);

    tft.setTextSize(5);
    tft.setCursor(5, 60);
    tft.print("Please Select A File");
    tft.setCursor(80, 150);
    tft.setTextColor(ILI9341_BLACK);
    tft.print("REC");
    tft.setTextColor(ILI9341_BLUE);
    tft.print("00");
    tft.setTextColor(ILI9341_BLACK);
    tft.print(" .txt");
  }
  if (tempVal != 0) {
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(5);
    tft.setCursor(142, 150);
    tft.setTextColor(ILI9341_DARKGREEN);
    tft.setTextColor(ILI9341_BLUE);
    tft.fillRect(142, 150, 40, 20, ILI9341_DARKGREEN);
    tft.print(chooseFileCurrentNum);
    delay(250);
  }
  oldNum = chooseFileCurrentNum;
  tft.setRotation(1);
}

// Functions to read from SD and store the data to recordSD[] array
void readfromSD() {
  String numRead = "";
  myFile = SD.open(readFilename);
  if (myFile) {
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      char in = myFile.read();
      if (byteCount > 12) {
        int temp = (int) in;
        if ((47 < temp && temp < 58) || temp == 45) {
          numRead += in;
        } else {
          if (numRead.length() != 0) {
            mem = numRead.toInt();
            numFlag = true;
          }
          numRead = "";
        }
        if (numFlag) {
          if (idx < 7493)
            recordSD[idx] = 0.83 * (240 - mem);
          else
            recordSD[idx] = mem;
          idx++;
          numFlag = false;
        }
      }
      byteCount++;
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
  HB_avg = recordSD[7493];
  QRS_avg = recordSD[7494];
  PRI_avg = recordSD[7495];

  bool_PAC_detected = recordSD[7497];
  bool_BRADY = recordSD[7498];
  bool_TACHY = recordSD[7499];
}

// Re-initilaize all variables
void setupVariables() {
  data = 0;
  hb_counter = 0;
  idx = 0;
  stopTimer = 0;
  stopEcg = 0;
  startTimerOld = 0;
  oldData = 120;
  xPosition = 330;
  menuSelect = 2;
  chooseFileCurrentNum = 0;
  HB_avg = 0;
  HB_add_tally = 0;
  PRI_avg = 0;
  PRI_add_tally = 0;
  QRS_avg = 0;
  QRS_add_tally = 0;
  PAC_counter = 0;
  previous_HB_length = 0;
  PRI_add_tally2 = 0;
  hb_delta_t = 0;

  trig = true;
  oldTrig = true;
  oldTrig2 = true;
  timerTrig = true;
  bool_heartBeatFlag = false;
  bool_hbTimerFlag = false;
  sdFlag = false;
  resetFlag = false;
  milliFlag = false;
  stopTimerFlag = false;
  startStabilize = true;
  whiteFlag = true;
  sdCardFull = false;
  writeChoice = true;
  chose = false;
  SDplaybackFlag = false;
  setupVariablesFlag = false;
  initializeDrawMenu = false;
  playBackMenuFlag = false;
  chooseFileInitialize = false;
  redrawFlag = false;
  changeADCPort = false;
  ADC_ECG_Read = false;
  displayFlag = false;
  bool_PAC_detected = false;
  bool_BRADY = false;
  bool_TACHY = false;
  hb_flash = false;

  StableTrue = false;
  millisStart = 0;

  diff = 4095;
  sensorMin = 4095;        // minimum sensor value
  sensorMax = 0;           // maximum sensor value

  count = 0;
  deltat = 0;

  PotentiometerValue = 0;
  chooseVal = 0;
  fileNum = -1;
  SDidx = 0;

  oldInput = 0;
  oldNum = 0;
  oldSDidx = 0;
  oldHB = 0;

  numFlag = false;
  mem = 0;
  byteCount = 0;

  for (int i = 0; i < 320; i++) {
    yPos[i] = 0;
  }

}

// Function to draw the ECG
void displayECG() {
  int val = digitalRead(17);
  if (val == LOW) {
    if (!resetFlag) {
      trig = !trig;
      delay(150);
    } else {
      trig = !trig;
      resetFlag = false;
      StableTrue = false;
      startStabilize = true;
      milliFlag = false;
      delay(150);
    }
  }

  if (xPosition > 319 || whiteFlag) {
    if (whiteFlag == true) {
      whiteFlag = false;
      tft.fillRect(0, 90 , 320, 150, ILI9341_DARKGREEN);
      tft.fillRect(0, 0 , 320, 90, ILI9341_WHITE);
      tft.setRotation(3);
      // Draw Border
      tft.fillRect(0, 150, 320, 5, ILI9341_BLACK);
      tft.fillRect(0, 150, 5, 90, ILI9341_BLACK);
      tft.fillRect(0, 235, 320, 5, ILI9341_BLACK);
      tft.fillRect(315, 150, 5, 90, ILI9341_BLACK);

      // Draw checkbox
      tft.fillRect(0, 210, 320, 2, ILI9341_BLACK);
      for (int i = 1; i <= 3; i++) {
        tft.drawLine(80 * i, 210, 80 * i, 240, ILI9341_BLACK);
      }

      // Draw the texts
      tft.setTextColor(ILI9341_BLACK);
      tft.setFont(AwesomeF000_14);
      tft.setCursor(7, 165); tft.print((char)4);
      tft.setTextColor(ILI9341_BLACK);
      tft.setFont(ArialBlack_12);
      tft.setTextColor(ILI9341_BLUE);
      tft.setCursor(30, 170); tft.print(HB_avg);
      tft.setFont(Arial_10_Bold_Italic);
      tft.setTextColor(ILI9341_BLACK);
      tft.setCursor(65, 172); tft.print("bpm");
      tft.setFont(ArialBlack_12);
      tft.setCursor(105, 170); tft.print("QRS");
      tft.setTextColor(ILI9341_BLUE);
      tft.setCursor(148, 170); tft.print(QRS_avg);
      tft.setFont(Arial_10_Bold_Italic);
      tft.setTextColor(ILI9341_BLACK);
      tft.setCursor(185, 172); tft.print("ms");
      tft.setFont(ArialBlack_12);
      tft.setCursor(215, 170); tft.print("PRI");
      tft.setTextColor(ILI9341_BLUE);
      tft.setCursor(252, 170); tft.print(PRI_avg);
      tft.setFont(Arial_10_Bold_Italic);
      tft.setTextColor(ILI9341_BLACK);
      tft.setCursor(289, 172); tft.print("ms");

      tft.setFont(ArialBlack_14);
      tft.setCursor(20, 192); tft.print("PVC");
      tft.setCursor(100, 192); tft.print("PAC");
      tft.setCursor(170, 192); tft.print("BRDY");
      tft.setCursor(250, 192); tft.print("TCHY");
      tft.setRotation(1);
    }
    displayFlag = true;

    // Send heartrate to Bluetooth device
    HRMdisplay();

    // Draw white the old data
    for (int i = 0; i < 319; i++) {
      if (yOldPos[i] < 150 && yOldPos[i + 1] < 150) {
        tft.setRotation(3);
        tft.drawLine(i, yOldPos[i], i + 1, yOldPos[i + 1], ILI9341_DARKGREEN);
        tft.drawLine(i, yOldPos[i] + 1, i + 1, yOldPos[i + 1] + 1, ILI9341_DARKGREEN);
        tft.drawLine(i, yOldPos[i] - 1, i + 1, yOldPos[i + 1] - 1, ILI9341_DARKGREEN);
        tft.drawLine(i, yOldPos[i] + 2, i + 1, yOldPos[i + 1] + 2, ILI9341_DARKGREEN);
        tft.drawLine(i, yOldPos[i] - 2, i + 1, yOldPos[i + 1] - 2, ILI9341_DARKGREEN);
        tft.setRotation(1);
      }
    }
    xPosition = 0;
    
    // Draw grid
    tft.setRotation(3);
    for (int i = 0; i < 52; i++) {
      if (i > 8 && i < 32) {
        if (i % 5 != 0)
          tft.drawLine(0, 240 - i * 10, 320, 240 - i * 10, ILI9341_RED);
        else
          tft.fillRect(0, 240 - (i * 10 + 2), 320, 3, ILI9341_RED);
      }
      if (i % 5 != 0)
        tft.drawLine(334 - i * 10, 0, 334 - i * 10, 150, ILI9341_RED);
      else
        tft.fillRect(334 - (i * 10 + 2), 0, 3, 150, ILI9341_RED);
    }
    tft.setRotation(1);
  }

  // Timer to stop the recording at 30s mark
  stopEcg = millis() - millisStart;
  if (stopEcg > 30000) {
    trig = false;
    sdFlag = true;
    millisStart = millis();
    tft.setRotation(1);
  }

  if (timerTrig && bool_hbTimerFlag) {
    timerTrig = false;
  }

  if (trig) {
    tft.setRotation(3);
    tft.fillRect(4 + idx * 311 / 7500, 156, 2, 7, ILI9341_BLUE);
    tft.setRotation(1);
    oldSDidx = idx;
  }

  // Draw the lines
  deltat = micros() - count; // Delay 4ms
  if (deltat >= 4000) {
    count = micros();
    if (trig) {
      int lineData = 0.83 * (240 - yPos[0]);
      if (lineData < 150 && oldData < 150) {
        tft.setRotation(3);
        tft.drawLine(xPosition, oldData, xPosition + 1, lineData, ILI9341_BLUE);
        tft.drawLine(xPosition, oldData + 1, xPosition + 1, lineData + 1, ILI9341_BLUE);
        tft.drawLine(xPosition, oldData - 1, xPosition + 1, lineData - 1, ILI9341_BLUE);
        tft.drawLine(xPosition, oldData + 2, xPosition + 1, lineData + 2, ILI9341_BLUE);
        tft.drawLine(xPosition, oldData - 2, xPosition + 1, lineData - 2, ILI9341_BLUE);
        tft.setRotation(1);
      }
      xPosition++;
      oldData = lineData;
      yOldPos[xPosition] = lineData;
    }
    //timer for heartbeat
  }
}

// Write all the heartbeat data to SD card
void storeToSD() {
  myFile = SD.open(filename, FILE_WRITE);
  myFile.print(headername);
  myFile.println(", 250");
  for (int w = 0; w < 7493; w++) {
    myFile.print(recordSD[w]);
    if ((w + 1) % 8 != 0)
      myFile.print(", ");
    else
      myFile.println();
  }
  myFile.println();
  myFile.print("Heart rate: ");
  myFile.println(HB_avg);
  myFile.print("QRS: ");
  myFile.println(QRS_avg);
  myFile.print("PRI: ");
  myFile.println(PRI_avg);
  myFile.print("PVC: ");
  myFile.println("0");
  myFile.print("PAC: ");
  myFile.println(bool_PAC_detected);
  myFile.print("Bradycardia: ");
  myFile.println(bool_BRADY);
  myFile.print("Tachycardia: ");
  myFile.println(bool_TACHY);
  myFile.println("EOF");
  myFile.close();
  num = num + 1;
  firstDigit = num / 10;
  secondDigit = num % 10;
  String oldFileName = String(filename);
  filename[3] = firstDigit + '0';
  filename[4] = secondDigit + '0';
  headername[5] = firstDigit + '0';
  headername[6] = secondDigit + '0';
  idx = 0;
  sdFlag = false;

  // Check to see if it reaches the 100 files mark
  if (num > 99) {
    tft.setRotation(3);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);
    tft.setCursor(0, 100);
    tft.println("SD CARD FULL");
    delay(4000);
    tft.setRotation(1);
    sdCardFull = true;
  } else {
    changeADCPort = false;
    tft.setRotation(3);
    tft.fillScreen(ILI9341_DARKGREEN);
    tft.setTextColor(ILI9341_BLACK);
    tft.fillRect(0, 0, 320, 5, ILI9341_WHITE);
    tft.fillRect(0, 0, 5, 240, ILI9341_WHITE);
    tft.fillRect(0, 235, 320, 5, ILI9341_WHITE);
    tft.fillRect(315, 0, 5, 240, ILI9341_WHITE);
    tft.setTextSize(3);
    tft.setCursor(15, 60);
    tft.setFont(ArialBlack_16);
    tft.println("DATA HAS BEEN SAVED");
    tft.setCursor(55, 80);
    tft.println("TO SD CARD AS:");
    tft.setCursor(95, 130);
    tft.setFont(ArialBlack_18);
    tft.setTextColor(ILI9341_RED);
    tft.print(oldFileName);
    tft.setFont(ArialBlack_16);
    tft.setTextColor(ILI9341_BLACK);
    tft.setRotation(1);
    delay(6500);
    setupVariablesFlag = false;
  }
}

void readDataModule() { //module that handles the playback from SD card
  int val = digitalRead(17);
  if (val == LOW) {
    trig = !trig;
    delay(250);
  }
  if (playBackMenuFlag == false && oldTrig == trig) { // Show recorded data
    playbackDraw();
    redrawFlag = true;
  }
  if (playBackMenuFlag == false && oldTrig != trig) {
    initializeDrawMenu = false;
    playBackMenuFlag = true;
  }
  if (menuSelect == 0 && oldTrig2 != trig) { // Go back to playback()
    tft.fillRect(0, 90 , 320, 150, ILI9341_DARKGREEN);
    tft.fillRect(0, 0 , 320, 90, ILI9341_WHITE);
    tft.setRotation(3);
    // Draw Border
    tft.fillRect(0, 150, 320, 5, ILI9341_BLACK);
    tft.fillRect(0, 150, 5, 90, ILI9341_BLACK);
    tft.fillRect(0, 235, 320, 5, ILI9341_BLACK);
    tft.fillRect(315, 150, 5, 90, ILI9341_BLACK);

    // Draw checkbox
    tft.fillRect(0, 210, 320, 2, ILI9341_BLACK);
    for (int i = 1; i <= 3; i++) {
      tft.drawLine(80 * i, 210, 80 * i, 240, ILI9341_BLACK);
    }

    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(AwesomeF000_14);
    tft.setCursor(7, 165); tft.print((char)4);
    tft.setTextColor(ILI9341_BLACK);
    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(30, 170); tft.print(HB_avg);
    tft.setFont(Arial_10_Bold_Italic);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(65, 172); tft.print("bpm");
    tft.setFont(ArialBlack_12);
    tft.setCursor(105, 170); tft.print("QRS");
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(148, 170); tft.print(QRS_avg);
    tft.setFont(Arial_10_Bold_Italic);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(185, 172); tft.print("ms");
    tft.setFont(ArialBlack_12);
    tft.setCursor(215, 170); tft.print("PRI");
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(252, 170); tft.print(PRI_avg);
    tft.setFont(Arial_10_Bold_Italic);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(289, 172); tft.print("ms");

    tft.setFont(ArialBlack_14);
    tft.setCursor(20, 192); tft.print("PVC");
    tft.setCursor(100, 192); tft.print("PAC");
    tft.setCursor(170, 192); tft.print("BRDY");
    tft.setCursor(250, 192); tft.print("TCHY");

    if (bool_PAC_detected) {
      tft.setFont(ArialBlack_12);
      tft.setTextColor(ILI9341_BLUE);
      tft.setCursor(115, 215); tft.print("X");
    }

    if (bool_BRADY) {
      tft.setFont(ArialBlack_12);
      tft.setTextColor(ILI9341_BLUE);
      tft.setCursor(195, 215); tft.print("X");
    }

    if (bool_TACHY) {
      tft.setFont(ArialBlack_12);
      tft.setTextColor(ILI9341_BLUE);
      tft.setCursor(275, 215); tft.print("X");
    }
    tft.setRotation(1);
    redrawFlag = false;
    playBackMenuFlag = false;
    menuSelect = 2;
  }
  if (menuSelect == 1 && oldTrig2 != trig) { // Exit to toplevel
    setupVariablesFlag = false;
  }
  if (playBackMenuFlag == true) { // Open menu and draw
    if (initializeDrawMenu == false) { //draw menu
      initializeDrawMenu = true;
      tft.setRotation(3);
      tft.setTextColor(ILI9341_BLACK);
      tft.fillRect(60, 40, 200, 160, ILI9341_DARKGREEN);
      tft.fillRect(60, 40, 200, 5, ILI9341_BLACK);
      tft.fillRect(60, 40, 5, 160, ILI9341_BLACK);
      tft.fillRect(255, 40, 5, 160, ILI9341_BLACK);
      tft.fillRect(60, 195, 200, 5, ILI9341_BLACK);
      tft.setTextSize(3);
      tft.setCursor(130, 80);
      tft.setFont(ArialBlack_14);
      tft.println("Continue");
      tft.setCursor(130, 140);
      tft.println("Exit");
      tft.setRotation(1);
      tft.setTextColor(ILI9341_BLUE);
      if (PotentiometerValue / 2048 == 0) {
        tft.setRotation(3);
        tft.setFont(AwesomeF000_32);
        tft.setCursor(75, 60);
        tft.print((char)88);
        tft.setRotation(1);
        menuSelect = 0;
      } else {
        tft.setRotation(3);
        tft.setFont(AwesomeF000_32);
        tft.setCursor(75, 120);
        tft.print((char)88);
        tft.setRotation(1);
        menuSelect = 1;
      }
    }
    if (PotentiometerValue / 2048 == 0 && oldInput != PotentiometerValue / 2048) {
      tft.setRotation(3);
      tft.fillRect(65, 45, 60, 145, ILI9341_DARKGREEN);
      tft.setFont(AwesomeF000_32);
      tft.setCursor(75, 60);
      tft.print((char)88);
      tft.setRotation(1);
      menuSelect = 0;
    }
    if (PotentiometerValue / 2048 == 1 && oldInput != PotentiometerValue / 2048) {
      tft.setRotation(3);
      tft.fillRect(65, 45, 60, 145, ILI9341_DARKGREEN);
      tft.setFont(AwesomeF000_32);
      tft.setCursor(75, 120);
      tft.print((char)88);
      tft.setRotation(1);
      menuSelect = 1;
    }
    oldTrig2 = trig;
  }
  oldInput  = PotentiometerValue / 2048;
  oldTrig = trig;
}

void playbackDraw() { // line creation and control part of readDataModule()

  // Case statements for iteration speed
  if (1900 <= PotentiometerValue && PotentiometerValue  < 2200) {
    SDidx = SDidx;
  } else if (1425 <= PotentiometerValue && PotentiometerValue < 1900) {
    SDidx = SDidx - 1;
  } else if (950 <= PotentiometerValue && PotentiometerValue < 1425) {
    SDidx = SDidx - 5;
  } else if (425 <= PotentiometerValue && PotentiometerValue < 950) {
    SDidx = SDidx - 20;
  } else if (PotentiometerValue < 425) {
    SDidx = SDidx - 50;
  } else if (2200 <= PotentiometerValue && PotentiometerValue < 2675) {
    SDidx = SDidx + 1;
  } else if (2675 <= PotentiometerValue && PotentiometerValue < 3150) {
    SDidx = SDidx + 5;
  } else if (3150 <= PotentiometerValue && PotentiometerValue < 3625) {
    SDidx = SDidx + 20 ;
  } else if (3625 <= PotentiometerValue) {
    SDidx = SDidx + 50;
  }
  // Case Statement for potentiometer Control
  if (SDidx <= 0 && PotentiometerValue <= 2200) {
    SDidx = 0;
  } else if (SDidx >= 7144 && PotentiometerValue >= 1900) {
    SDidx = 7144;
  }

  tft.setRotation(3);
  float period = 60000 / HB_avg;
  if (millis() - count > period && !hb_flash) {
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(7, 165); tft.print((char)4);
    hb_delta_t = millis();
    hb_flash = true;
    count = millis();
  }
  if (millis() - hb_delta_t > 300 && hb_flash) {
    tft.setFont(AwesomeF000_14);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(7, 165); tft.print((char)4);
    hb_flash = false;
  }
  tft.setRotation(1);

  if (oldSDidx != SDidx || redrawFlag == false) {
    tft.setRotation(3);
    if ((1900 >= PotentiometerValue && PotentiometerValue > 2200) == false) {
      // Draw grid
      for (int i = 0; i < 52; i++) {
        if (i > 8 && i < 32) {
          if (i % 5 != 0)
            tft.drawLine(0, 240 - i * 10, 320, 240 - i * 10, ILI9341_RED);
          else
            tft.fillRect(0, 240 - (i * 10 + 2), 320, 3, ILI9341_RED);
        }
        if (i % 5 != 0)
          tft.drawLine(334 - i * 10, 0, 334 - i * 10, 150, ILI9341_RED);
        else
          tft.fillRect(334 - (i * 10 + 2), 0, 3, 150, ILI9341_RED);
      }

      tft.fillCircle(10 + oldSDidx * 299 / 7144, 160, 5, ILI9341_WHITE);
      tft.fillCircle(10 + SDidx * 299 / 7144, 160, 5, ILI9341_BLUE);

      for (int j = 0; j < 319; j++) {
        if (recordSD[oldSDidx + j] < 150 && recordSD[oldSDidx + j + 1] < 150)
        {
          tft.drawLine(j, recordSD[oldSDidx + j], j + 1, recordSD[oldSDidx + j + 1], ILI9341_DARKGREEN);
          tft.drawLine(j, recordSD[oldSDidx + j] + 1, j + 1, recordSD[oldSDidx + j + 1] + 1, ILI9341_DARKGREEN);
          tft.drawLine(j, recordSD[oldSDidx + j] - 1, j + 1, recordSD[oldSDidx + j + 1] - 1, ILI9341_DARKGREEN);
          tft.drawLine(j, recordSD[oldSDidx + j] + 2, j + 1, recordSD[oldSDidx + j + 1] + 2, ILI9341_DARKGREEN);
          tft.drawLine(j, recordSD[oldSDidx + j] - 2, j + 1, recordSD[oldSDidx + j + 1] - 2, ILI9341_DARKGREEN);
        }
        if (recordSD[SDidx + j] < 150 && recordSD[SDidx + j + 1] < 150)
        {
          tft.drawLine(j, recordSD[SDidx + j], j + 1, recordSD[SDidx + j + 1], ILI9341_BLUE);
          tft.drawLine(j, recordSD[SDidx + j] + 1, j + 1, recordSD[SDidx + j + 1] + 1, ILI9341_BLUE);
          tft.drawLine(j, recordSD[SDidx + j] - 1, j + 1, recordSD[SDidx + j + 1] - 1, ILI9341_BLUE);
          tft.drawLine(j, recordSD[SDidx + j] + 2, j + 1, recordSD[SDidx + j + 1] + 2, ILI9341_BLUE);
          tft.drawLine(j, recordSD[SDidx + j] - 2, j + 1, recordSD[SDidx + j + 1] - 2, ILI9341_BLUE);
        }
      }
    }
  }
  tft.setRotation(1);
  oldSDidx = SDidx;
}

// Update the heartbeat information everytime a heartbeat is detected
void updateInfo() {
  int checkHB = hb_counter - oldHB;
  tft.setRotation(3);

  if (checkHB != 0) {
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(7, 165); tft.print((char)4);
    hb_delta_t = millis();
    hb_flash = true;

    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.fillRect(30, 168, 36, 16, ILI9341_WHITE);
    tft.setCursor(30, 170); tft.print(HB_avg);

    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.fillRect(148, 168, 36, 16, ILI9341_WHITE);
    tft.setCursor(148, 170); tft.print(QRS_avg);

    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.fillRect(252, 168, 36, 16, ILI9341_WHITE);
    tft.setCursor(254, 170); tft.print(PRI_avg);
  }

  if (bool_PAC_detected) {
    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(115, 215); tft.print("X");
  }

  if (bool_BRADY) {
    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(195, 215); tft.print("X");
  } else {
    tft.fillRect(195, 215, 13, 13, ILI9341_WHITE);
  }

  if (bool_TACHY) {
    tft.setFont(ArialBlack_12);
    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(275, 215); tft.print("X");
  } else {
    tft.fillRect(275, 215, 13, 13, ILI9341_WHITE);
  }


  if (millis() - hb_delta_t -500 < 300) {
    tft.setFont(AwesomeF000_14);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(7, 165); tft.print((char)4);
    hb_flash = false;
  } else{
    tft.setFont(AwesomeF000_14);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(7, 165); tft.print((char)4);
  }

  tft.setRotation(1);
  oldHB = hb_counter;
}

// Send heartrate to Bluetooth device
void HRMdisplay() {
  int heart_rate = HB_avg;
  Serial.print(F("Updating HRM value to "));
  Serial.print(heart_rate);
  Serial.println(F(" BPM"));
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00-") );
  ble.println(heart_rate, HEX);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }
}

// Initilaize ADC
void adcInit() {
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();
  Serial.println("calibrated");

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[6];
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}

// Initilaize PDB
void pdbInit() {
  //pinMode(13, OUTPUT);

  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Timer period
  PDB0_MOD = PDB_PERIOD;
  // Interrupt delay
  PDB0_IDLY = 0;
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

// Initilaize DMA
void dmaInit() {
  // Enable DMA, DMAMUX clocks
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // Use default configuration
  DMA_CR = 0;

  // Source address
  DMA_TCD1_SADDR = &ADC0_RA;
  // Don't change source address
  DMA_TCD1_SOFF = 0;
  DMA_TCD1_SLAST = 0;
  // Destination address
  DMA_TCD1_DADDR = samples;
  // Destination offset (2 byte)
  DMA_TCD1_DOFF = 2;
  // Restore destination address after major loop
  DMA_TCD1_DLASTSGA = -sizeof(samples);
  // Source and destination size 16 bit
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  // Number of bytes to transfer (in each service request)
  DMA_TCD1_NBYTES_MLNO = 2;
  // Set loop counts
  DMA_TCD1_CITER_ELINKNO = sizeof(samples) / 2;
  DMA_TCD1_BITER_ELINKNO = sizeof(samples) / 2;
  // Enable interrupt (end-of-major loop)
  DMA_TCD1_CSR = DMA_TCD_CSR_INTMAJOR;

  // Set ADC as source (CH 1), enable DMA MUX
  DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
  DMAMUX0_CHCFG1 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

  // Enable request input signal for channel 1
  DMA_SERQ = 1;

  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}

// ADC calibration
void adcCalibrate() {
  uint16_t sum;

  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);

  // Plus side gain
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}

// Take heartbeat data at interrupt
void adc0_isr() {
  boolean tempTrig = true;
  if (displayFlag) {
    tempTrig = trig;
  }
  if (tempTrig && writeChoice && chose) {

    if (millis() - hb_timer > 500) {
      bool_hbTimerFlag = true;
    }

    stopTimerFlag = false;
    data = ADC0_RA;

    for (int i = 319; i > 0; i--) { //create new line
      yPos[i] = yPos[i - 1];
      int baselineCheck = 0;
      if(bool_heartBeatFlag == false && bool_hbTimerFlag == true){
        for (int p = 0; p < 35; p++){
          baselineCheck = baselineCheck + yPos[125 + p];
        }
        baselineCheck = baselineCheck / 35;
      }
      if ((baselineCheck + 10 < yPos[120] && yPos[120] < baselineCheck + 100) && bool_heartBeatFlag == false && bool_hbTimerFlag == true) { // if QRS Peak is detected
        int j = 1;
        int storeBeatVoltage = yPos[120];
        boolean PRI_detected = false;
        boolean storeBeat = true;
        int Q_time = 0;
        int S_time = 0;
        while (j < 20 && !PRI_detected) { // Begin pri detection : look for PRI dip
          //Serial.println("TEST");
          if (PRI_detected == true)
            break;
          j = j + 1;
          if (yPos[120 + j + 1] > yPos[120 + j] && yPos[120 + j + 2] > yPos[120 + j] && !PRI_detected) { // Q detected
            if (storeBeat == true) {
              storeBeatVoltage = yPos[120 + j];
              storeBeat = false;
              Q_time = j;
            }
            j = j + 2;
            while (j < 45) { // Look for P peak
              if (PRI_detected == true)
                break;
              j = j + 1;
              if (yPos[120 + j] > storeBeatVoltage && yPos[120 + j + 1] > storeBeatVoltage && !PRI_detected) { // P peak detected
                j = j + 2;
                while (j < 60) {
                  if (PRI_detected == true)
                    break;
                  j = j + 1;
                  if (yPos[120 + j] < storeBeatVoltage && yPos[120 + j + 1] < storeBeatVoltage) {
                    //Serial.println("PRI DETECTED");
                    //Serial.println(j);
                    PRI_detected = true;
                    PRI_time = (j/* - Q_time*/) * 4;
                    break;
                  }
                }
              }
            }
          }
        }
        // begin QT inverval detection
        int k = 1;
        boolean QT_detected = false;
        storeBeat = true;
        storeBeatVoltage = yPos[120];
        while (k < 30 && !QT_detected) {
          //storeBeatVoltage = yPos[120 - k];
          k = k + 1;
          if (yPos[120 - k] < yPos[120 - k - 1] && yPos[120 - k] < yPos[120 - k - 2] && !QT_detected) { // S detected
            if (storeBeat == true) {
              storeBeat = false;
              storeBeatVoltage = yPos[120 - k];
              S_time = k;
            }

            while (k < 80) { // Look for P peak
              if (QT_detected)
                break;
              k = k + 1;
              if (yPos[120 - k] > storeBeatVoltage && !QT_detected) { // T peak detected
                while (k < 121) {
                  if (QT_detected)
                    break;
                  k = k + 1;
                  if (yPos[120 - k] <= storeBeatVoltage + 5) {
                    //Serial.println("QRI DETECTED");
                    QT_detected = true;
                    if (PRI_detected == true) {
                      QRS_time = (S_time + Q_time) * 4;
                    }
                    break;
                  }
                }
              }
            }
          }
        }

        if (PRI_detected && QT_detected) {
          if (hb_counter >= 9) {
            HB_add_tally = HB_add_tally +  60000000 / ((millis() - hb_timer));
            HB_avg = (HB_add_tally) / (hb_counter - 8) / 1000;
            if (HB_avg < 60)
              bool_BRADY = true;
            if ( 60 <= HB_avg && HB_avg < 100 ) {
              bool_BRADY = false;
              bool_TACHY = false;
            }
            if (100 <= HB_avg) {
              bool_TACHY = true;
            }
            
            PRI_add_tally2 = PRI_add_tally2 + PRI_time;
            PRI_avg = PRI_add_tally2 / (hb_counter - 8);
            QRS_add_tally = QRS_add_tally + QRS_time;
            QRS_avg = QRS_add_tally / (hb_counter - 8);

            if (PAC_counter < 7) {
              float HB_difference = ((float)previous_HB_length / (millis() - hb_timer));
              if (HB_difference < .833 || 1.200 < HB_difference) {
                PAC_counter = PAC_counter + 1;
              }
            } else {
              bool_PAC_detected = true;
            }
          }
          previous_HB_length = millis() - hb_timer;
          hb_timer = millis();
          hb_counter++;
          bool_heartBeatFlag = true;
          bool_hbTimerFlag = false;
          Serial.println("HEARTBEAT DETECTED");
          Serial.println(hb_counter);
          if (hb_counter > 10) {
            StableTrue = true;
          }
        }
      }
    }
    bool_heartBeatFlag = false;

    //LOW PASS
    filterOneLowpass.input( data );
    yPos[0] = 120 + 48 * highPassFilter.input(filterOneLowpass.output()) / 819;

    if (yPos[0] > 240)
      yPos[0] = 240;
    if (yPos[0] < 0)
      yPos[0] = 0;

    if (StableTrue) {
      if (idx < 7500) {
        recordSD[idx] = yPos[0];
        idx++;
      } else
        idx = 0;
    }
  } else {
    if (!stopTimerFlag) {
      stopTimer = millis();
      stopTimerFlag = true;
      startTimerOld = millisStart;
    } else
      millisStart = startTimerOld + millis() - stopTimer;
  }
  if (!StableTrue) {
    millisStart = millis();
  }
}

void PVC_reg() { //PVC function
  /*
          int k = 0;
          bool QT_detected = false;
          bool storeBeat = true;
          while (k < 40 && !QT_detected) {
          //storeBeatVoltage = yPos[120 - k];
          k = k + 1;
          if (yPos[120 - k] < yPos[120 - k - 1] && !QT_detected) { // S detected
            if (storeBeat == true) {
              storeBeat = false;
              storeBeatVoltage = yPos[120 - k];
              S_time = k;
            }

            while (k < 80) { // Look for P peak
              if (QT_detected)
                break;
              k = k + 1;
              if (yPos[120 - k] > storeBeatVoltage && !QT_detected) { // T peak detected
                while (k < 121) {
                  if (QT_detected)
                    break;
                  k = k + 1;
                  if (yPos[120 - k] <= storeBeatVoltage + 5) {
                    //Serial.println("QRI DETECTED");
                    QT_detected = true;
                    if (PRI_detected == true) {
                      QRS_time = (S_time + Q_time) * 4;
                    }
                    break;
                  }
                }
              }
            }
          }
        }
  */
}

void pdb_isr() {
  //digitalWrite(13, (ledOn = !ledOn));
  PDB0_SC &= ~PDB_SC_PDBIF; // (also clears interrupt flag)
}

void dma_ch1_isr() {
  // Clear interrupt request for channel 1
  DMA_CINT = 1;
}
