
/*********************************************************************
  SMA-Q2 / SMA-TIME Firmware v0.2
  2nd version for an Arduino-based SMA-Q2 firmware, this time based on the sandeepmistry core and the great work by Aaron Christophel


  Change the pins in variant.h
  (C:\Users\Kauzdesktop\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.8.5\variants\feather52) file accordingly
  (SPI depicted only)

  #define PIN_SPI_MISO         (11) //14
  #define PIN_SPI_MOSI         (3) //13
  #define PIN_SPI_SCK          (2) //12




  With the original firmware, you enter the DFU mode of the watch by pressing the "back" as well as the "up"
  and "down" buttons simultaneously (a little acrobatic) when the watch powered off.
  Alternatively press this button combo and reset the thing (by pulling the rst testpad (P0.21) to ground with a reed switch) or power-cycling it.
  You should see a "DFU mode" screen with a progress bar.

  With custom firmware the MCU also checks for the bootloader button combo as it gets out of systemOff(BTN_UP, LOW); //name any pin and wakeup logic here
   So in the above example you press up and down first, and the BTN_UP last to enter DFU mode.

  A. Jordan 2019.
*********************************************************************/

//nrfjprog --recover -f NRF52                   recovers locked chips

#include <SPI.h>
#include <stdint.h>
#include <BLEPeripheral.h>

#include <nrf_nvic.h>//interrupt controller stuff
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>
#include "Adafruit_GFX.h"
#include "ColorMemLCD.h"
#include <TimeLib.h>
#include <nrf.h>
#include "count_steps.h"
#include "count_steps.c"
#include "i2csoft.h"

//pin definitions for the SMA-Q2
#define SCK 2
#define MOSI 3
#define SS 5
#define EXTCOMIN 6
#define DISP 7
#define VBAT 4 //with a 1/4 divider
#define VIBRO 30
#define BCKLT 8
#define BTN_BCK 9
#define BTN_UP 27
#define BTN_OK 28
#define BTN_DOWN 29
#define FLASH_MISO 11
#define FLASH_MOSI 13
#define FLASH_CS 12
#define FLASH_CLK 14
#define ACCEL_PWR 16
#define ACCEL_INT 15
#define ACCEL_SDA 17
#define ACCEL_SCL 18
#define HRM_SDA 19
#define HRM_SCL 20
#define HRM_INT 22
#define HRM_RST 10
#define HRM_PWR 31
#define CHG_STAT 23
#define CHG_PGOOD 24
#define UART_RXD 25
#define UART_TXD 26

ColorMemLCD display(SCK, MOSI, SS, EXTCOMIN);
//SoftwareTimer timerDisp;
//RTCInt rtc;  //create an RTCInt type object

//vars
boolean extcominState, bltState = 0;
int adcvalue = 0;
float mv_per_lsb = 600.0F / 16384.0F; // 12-bit ADC with 3.6V input range
long buttonTimer = 0;

long accSampleTimer = 0;
const int longPressTime = 3000;
boolean buttonActive = false;
boolean longPressActive = false;

long debounceTimer, BCKdebounceTimer = 0;
int debounceInterval = 500;
boolean OKbuttonPressed = 0;
//*********************************************Aarons functions*********************************************
#define wdt_reset() NRF_WDT->RR[0] = WDT_RR_RR_Reload
#define wdt_enable(timeout) \
  NRF_WDT->CONFIG = NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos); \
  NRF_WDT->CRV = (32768*timeout)/1000; \
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  \
  NRF_WDT->TASKS_START = 1

boolean debug = false;

#define sleepDelay 7000
#define BUTTON_PIN              28
#define refreshRate 100

int menu;
volatile bool buttonPressed = false;
long startbutton;
unsigned long sleepTime, displayRefreshTime;
volatile bool sleeping = false;
int timezone;
int steps;
int steps1;
String serialNr = "235246472";
String versionNr = "110.200.051";
String btversionNr = "100.016.051";
String msgText;
boolean gotoBootloader = false;
boolean vibrationMode;

String bleSymbol = " ";
int contrast;

BLEPeripheral                   blePeripheral           = BLEPeripheral();
BLEService                      batteryLevelService     = BLEService("190A");
BLECharacteristic   TXchar        = BLECharacteristic("0002", BLENotify, 20);
BLECharacteristic   RXchar        = BLECharacteristic("0001", BLEWriteWithoutResponse, 20);

BLEService                      batteryLevelService1     = BLEService("190B");
BLECharacteristic   TXchar1        = BLECharacteristic("0004", BLENotify, 20);
BLECharacteristic   RXchar1        = BLECharacteristic("0003", BLEWriteWithoutResponse, 20);

#define N_GRAINS     253 // Number of grains of sand
#define WIDTH        127 // Display width in pixels
#define HEIGHT       63 // Display height in pixels
#define MAX_FPS      60 // Maximum redraw rate, frames/second

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
} grain[N_GRAINS];

uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint16_t         backbuffer = 0, img[WIDTH * HEIGHT]; // Internal 'map' of pixels

#ifdef __cplusplus
extern "C" {
#endif

#define LF_FREQUENCY 32768UL
#define SECONDS(x) ((uint32_t)((LF_FREQUENCY * x) + 0.5))
#define wakeUpSeconds 120
void RTC2_IRQHandler(void)
{
  volatile uint32_t dummy;
  if (NRF_RTC2->EVENTS_COMPARE[0] == 1)
  {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->CC[0] = NRF_RTC2->COUNTER +  SECONDS(wakeUpSeconds);
    dummy = NRF_RTC2->EVENTS_COMPARE[0];
    dummy;
    //powerUp();
  }
}

void initRTC2() {

  NVIC_SetPriority(RTC2_IRQn, 15);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);

  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->CC[0] = SECONDS(wakeUpSeconds);
  NRF_RTC2->INTENSET = RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos;
  NRF_RTC2->EVTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;
  NRF_RTC2->TASKS_START = 1;
}
#ifdef __cplusplus
}
#endif

void powerUp() {
  if (sleeping) {
    sleeping = false;
    //display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.refresh();
    if (debug)Serial.begin(115200);

    delay(5);
  }
  sleepTime = millis();
}

void powerDown() {
  //wdt_feed();
  if (!sleeping) {
    if (debug)NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;
    sleeping = true;

    //    digitalWrite(28, LOW);
    //    digitalWrite(5, LOW);
    //    digitalWrite(6, LOW);
    //    digitalWrite(29, LOW);
    //    digitalWrite(4, LOW);
    NRF_SAADC ->ENABLE = 0; //disable ADC
    NRF_PWM0  ->ENABLE = 0; //disable all pwm instance
    NRF_PWM1  ->ENABLE = 0;
    NRF_PWM2  ->ENABLE = 0;
  }
}

void charge() {
  if (sleeping)menu = 88;
  powerUp();
}

void buttonHandler() {
    // check if enough time has passed to consider it a switch press
    if ((millis() - debounceTimer) > debounceInterval) {   
      //do the Stuff the ISR is supposed to do
      if (!sleeping) buttonPressed = true;
      else menu = 77;
      powerUp();
      debounceTimer = millis();
    }
}

void acclHandler() {
  ReadRegister(0x17);
  if (sleeping) {
    menu = 77;
    powerUp();
  }
}

void blePeripheralConnectHandler(BLECentral& central) {
  if (debug)Serial.println("BLEconnected");
  menu = 0;
  powerUp();
  bleSymbol = "B";
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  if (debug)Serial.println("BLEdisconnected");
  menu = 0;
  powerUp();
  bleSymbol = " ";
}

String answer = "";
String tempCmd = "";
int tempLen = 0, tempLen1;
boolean syn;

void characteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  char remoteCharArray[21];
  tempLen1 = characteristic.valueLength();
  tempLen = tempLen + tempLen1;
  memset(remoteCharArray, 0, sizeof(remoteCharArray));
  memcpy(remoteCharArray, characteristic.value(), tempLen1);
  tempCmd = tempCmd + remoteCharArray;
  if (tempCmd[tempLen - 2] == '\r' && tempCmd[tempLen - 1] == '\n') {
    answer = tempCmd.substring(0, tempLen - 2);
    tempCmd = "";
    tempLen = 0;
    if (debug)Serial.print("RxBle: ");
    if (debug)Serial.println(answer);
    filterCmd(answer);
  }
}

void filterCmd(String Command) {
  if (Command == "AT+BOND") {
    sendBLEcmd("AT+BOND:OK");
  } else if (Command == "AT+ACT") {
    sendBLEcmd("AT+ACT:0");
  } else if (Command.substring(0, 7) == "BT+UPGB") {
    gotoBootloader = true;
  } else if (Command.substring(0, 8) == "BT+RESET") {
    if (gotoBootloader)NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  } else if (Command.substring(0, 7) == "AT+RUN=") {
    sendBLEcmd("AT+RUN:" + Command.substring(7));
  } else if (Command.substring(0, 8) == "AT+USER=") {
    sendBLEcmd("AT+USER:" + Command.substring(8));
  }  else if (Command.substring(0, 7) == "AT+REC=") {
    sendBLEcmd("AT+REC:" + Command.substring(7));
  } else if (Command.substring(0, 8) == "AT+PUSH=") {
    sendBLEcmd("AT+PUSH:OK");
    menu = 99;
    powerUp();
    handlePush(Command.substring(8));
  } else if (Command.substring(0, 9) == "AT+MOTOR=") {
    sendBLEcmd("AT+MOTOR:" + Command.substring(9));
  } else if (Command.substring(0, 8) == "AT+DEST=") {
    sendBLEcmd("AT+DEST:" + Command.substring(8));
  } else if (Command.substring(0, 9) == "AT+ALARM=") {
    sendBLEcmd("AT+ALARM:" + Command.substring(9));
  } else if (Command.substring(0, 13) == "AT+HRMONITOR=") {
    sendBLEcmd("AT+HRMONITOR:" + Command.substring(13));
  } else if (Command.substring(0, 13) == "AT+FINDPHONE=") {
    sendBLEcmd("AT+FINDPHONE:" + Command.substring(13));
  } else if (Command.substring(0, 13) == "AT+ANTI_LOST=") {
    sendBLEcmd("AT+ANTI_LOST:" + Command.substring(13));
  } else if (Command.substring(0, 9) == "AT+UNITS=") {
    sendBLEcmd("AT+UNITS:" + Command.substring(9));
  } else if (Command.substring(0, 11) == "AT+HANDSUP=") {
    sendBLEcmd("AT+HANDSUP:" + Command.substring(11));
  } else if (Command.substring(0, 7) == "AT+SIT=") {
    sendBLEcmd("AT+SIT:" + Command.substring(7));
  } else if (Command.substring(0, 7) == "AT+LAN=") {
    sendBLEcmd("AT+LAN:ERR");
  } else if (Command.substring(0, 14) == "AT+TIMEFORMAT=") {
    sendBLEcmd("AT+TIMEFORMAT:" + Command.substring(14));
  } else if (Command == "AT+BATT") {
    sendBLEcmd("AT+BATT:" + String(getBatteryLevel()));
  } else if (Command == "BT+VER") {
    sendBLEcmd("BT+VER:" + btversionNr);
  } else if (Command == "AT+VER") {
    sendBLEcmd("AT+VER:" + versionNr);
  } else if (Command == "AT+SN") {
    sendBLEcmd("AT+SN:" + serialNr);
  } else if (Command.substring(0, 10) == "AT+DISMOD=") {
    sendBLEcmd("AT+DISMOD:" + Command.substring(10));
  } else if (Command.substring(0, 7) == "AT+LAN=") {
    sendBLEcmd("AT+LAN:ERR");
  } else if (Command.substring(0, 10) == "AT+MOTOR=1") {
    sendBLEcmd("AT+MOTOR:1" + Command.substring(10));
    digitalWrite(VIBRO, HIGH);
    delay(300);
    digitalWrite(VIBRO, LOW);
  } else if (Command.substring(0, 12) == "AT+CONTRAST=") {
    contrast = Command.substring(12).toInt();
  } else if (Command.substring(0, 6) == "AT+DT=") {
    SetDateTimeString(Command.substring(6));
    sendBLEcmd("AT+DT:" + GetDateTimeString());
  } else if (Command.substring(0, 5) == "AT+DT") {
    sendBLEcmd("AT+DT:" + GetDateTimeString());
  } else if (Command.substring(0, 12) == "AT+TIMEZONE=") {
    timezone = Command.substring(12).toInt();
    sendBLEcmd("AT+TIMEZONE:" + String(timezone));
  } else if (Command.substring(0, 11) == "AT+TIMEZONE") {
    sendBLEcmd("AT+TIMEZONE:" + String(timezone));
  } else if (Command == "AT+STEPSTORE") {
    sendBLEcmd("AT+STEPSTORE:OK");
  } else if (Command == "AT+TOPACE=1") {
    sendBLEcmd("AT+TOPACE:OK");
    sendBLEcmd("NT+TOPACE:" + String(steps));
  } else if (Command == "AT+TOPACE=0") {
    sendBLEcmd("AT+TOPACE:" + String(steps));
  } else if (Command == "AT+DATA=0") {
    sendBLEcmd("AT+DATA:0,0,0,0");
  } else if (Command.substring(0, 8) == "AT+PACE=") {
    steps1 = Command.substring(8).toInt();
    sendBLEcmd("AT+PACE:" + String(steps1));
  } else if (Command == "AT+PACE") {
    sendBLEcmd("AT+PACE:" + String(steps1));
  } else if (Command == "AT+DATA=1") {
    sendBLEcmd("AT+DATA:0,0,0,0");
  } else if (Command.substring(0, 7) == "AT+SYN=") {
    if (Command.substring(7) == "1") {
      sendBLEcmd("AT+SYN:1");
      syn = true;
    } else {
      sendBLEcmd("AT+SYN:0");
      syn = false;
    }
  }

}

void sendBLEcmd(String Command) {
  if (debug)Serial.print("TxBle: ");
  if (debug)Serial.println(Command);
  Command = Command + "\r\n";
  while (Command.length() > 0) {
    const char* TempSendCmd;
    String TempCommand = Command.substring(0, 20);
    TempSendCmd = &TempCommand[0];
    TXchar.setValue(TempSendCmd);
    TXchar1.setValue(TempSendCmd);
    Command = Command.substring(20);
  }
}

String GetDateTimeString() {
  String datetime = String(year());
  if (month() < 10) datetime += "0";
  datetime += String(month());
  if (day() < 10) datetime += "0";
  datetime += String(day());
  if (hour() < 10) datetime += "0";
  datetime += String(hour());
  if (minute() < 10) datetime += "0";
  datetime += String(minute());
  return datetime;
}

void SetDateTimeString(String datetime) {
  int year = datetime.substring(0, 4).toInt();
  int month = datetime.substring(4, 6).toInt();
  int day = datetime.substring(6, 8).toInt();
  int hr = datetime.substring(8, 10).toInt();
  int min = datetime.substring(10, 12).toInt();
  int sec = datetime.substring(12, 14).toInt();
  setTime( hr, min, sec, day, month, year);
}

void handlePush(String pushMSG) {
  int commaIndex = pushMSG.indexOf(',');
  int secondCommaIndex = pushMSG.indexOf(',', commaIndex + 1);
  int lastCommaIndex = pushMSG.indexOf(',', secondCommaIndex + 1);
  String MsgText = pushMSG.substring(commaIndex + 1, secondCommaIndex);
  int timeShown = pushMSG.substring(secondCommaIndex + 1, lastCommaIndex).toInt();
  int SymbolNr = pushMSG.substring(lastCommaIndex + 1).toInt();
  msgText = MsgText;
  if (debug)Serial.println("MSGtext: " + msgText);
  if (debug)Serial.println("symbol: " + String(SymbolNr));
}

int getBatteryLevel() {
  return map(analogRead(VBAT), 306, 355, 0, 100); //adapted for SMA-Q2 306 at 3,6v and 350 at 4,1v
}
//*********************************************SETUP*********************************************
void setup(void)
{

  //************************************************************************
  //adafruit core stuff for ADC
  // Set the analog reference to 3.0V (default = 3.6V)
  //analogReference(AR_INTERNAL_3_0);
  // Set the resolution to 12-bit (0..4095)
  //analogReadResolution(12); // Can be 8, 10, 12 or 14
  //************************************************************************
  pinMode(BCKLT, OUTPUT);
  pinMode(VIBRO, OUTPUT);
  pinMode(ACCEL_PWR, OUTPUT);
  pinMode(HRM_PWR, OUTPUT);
  pinMode(BTN_BCK, INPUT);
  pinMode(BTN_UP, INPUT);
  pinMode(CHG_STAT, INPUT);
  //pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT);
  pinMode(DISP, OUTPUT);
  //attachInterrupt(BTN_UP, btn_up_isr, FALLING);
  digitalWrite(DISP, HIGH);
  digitalWrite(BCKLT, LOW);
  digitalWrite(VIBRO, LOW);
  digitalWrite(ACCEL_PWR, HIGH); //turn on the accelerometer
  digitalWrite(HRM_PWR, LOW);

  //*********AArons Button******************
  //pinMode(2, INPUT);
  //  pinMode(25, OUTPUT);
  //  digitalWrite(25, HIGH);
  //  pinMode(4, OUTPUT);
  //  digitalWrite(4, LOW);
  //  pinMode(15, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACCEL_INT, INPUT);
  //pinMode(3, INPUT);
  //  if (digitalRead(BUTTON_PIN) == LOW) {
  //    NRF_POWER->GPREGRET = 0x01;
  //    sd_nvic_SystemReset();
  //  }

  if (debug)Serial.begin(115200);
  // wdt_enable(5000);
  blePeripheral.setLocalName("DS-D6");
  blePeripheral.setAdvertisingInterval(555);
  blePeripheral.setAppearance(0x0000);
  blePeripheral.setConnectable(true);
  blePeripheral.setDeviceName("ATCDSD6");
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService.uuid());
  blePeripheral.addAttribute(batteryLevelService);
  blePeripheral.addAttribute(TXchar);
  blePeripheral.addAttribute(RXchar);
  RXchar.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService1.uuid());
  blePeripheral.addAttribute(batteryLevelService1);
  blePeripheral.addAttribute(TXchar1);
  blePeripheral.addAttribute(RXchar1);
  RXchar1.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  blePeripheral.begin();
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(ACCEL_INT), acclHandler, RISING);
  NRF_GPIO->PIN_CNF[ACCEL_INT] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[ACCEL_INT] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  attachInterrupt(digitalPinToInterrupt(CHG_STAT), charge, FALLING);
  NRF_GPIO->PIN_CNF[CHG_STAT] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[CHG_STAT] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  //display.begin(SSD1306_SWITCHCAPVCC);
  // start & clear the display
  display.begin();

  delay(100);
  display.clearDisplay();
  // display.setFont(&FreeSerifItalic9pt7b);
  display.refresh();
  display.setTextSize(1);
  display.setTextColor(LCD_COLOR_BLACK);
  display.setCursor(10, 0);
  display.println("D6 Emulator");
  display.refresh();
  digitalWrite(VIBRO, LOW);
  delay(2000);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  initRTC2();
  initi2c();
  initkx023();
  digitalWrite(VIBRO, HIGH);
  delay(200);
  digitalWrite(VIBRO, LOW);

  uint8_t i, j, bytes;
  memset(img, 0, sizeof(img)); // Clear the img[] array
  for (i = 0; i < N_GRAINS; i++) { // For each sand grain...
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for (j = 0; (j < i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                              ((grain[i].y / 256) != (grain[j].y / 256))); j++);
    } while (j < i); // Keep retrying until a clear spot is found
    img[(grain[i].y / 256) * WIDTH + (grain[i].x / 256)] = 255; // Mark it
    grain[i].pos = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
  }
  //setup a software timer that will feed the watchdog every 9 seconds
  //  timerDisp.begin(1000, disp_callback);
  //  timerDisp.start();

  //  rtc.begin(TIME_H12); //init RTC in 12 hour mode
  //  //
  //  //filling internal structure for time
  //  rtc.time.hour = 0;          //hour
  //  rtc.time.minute = 0;       //minute
  //  rtc.time.second = 0;        //second
  //  rtc.time.Tmode = ANTI_MERIDIAN;
  //
  //  //filling internal structure for date
  //  rtc.date.day = 13;        //day
  //  rtc.date.month = 8;       //month
  //  rtc.date.year = 15;       //year
  //
  //  rtc.setTime();  //setting time
  //  rtc.setDate();  //setting date


  //************************************************************************
  // text display tests
  //  display.setTextSize(1);
  //  display.setTextColor(LCD_COLOR_MAGENTA);
  //  display.setCursor(0, 0);
  //  display.println("MAGENTA");
  //  display.setTextColor(LCD_COLOR_YELLOW);
  //  display.println("YELLOW");
  //  display.setTextColor(LCD_COLOR_CYAN);
  //  display.println("CYAN");
  //  display.setTextColor(LCD_COLOR_GREEN);
  //  display.println("GREEN");
  //  display.setTextColor(LCD_COLOR_RED);
  //  display.println("RED");
  //  display.setTextColor(LCD_COLOR_BLUE);
  //  display.println("BLUE");
  //
  //  display.setTextColor(LCD_COLOR_LCD_COLOR_BLACK, LCD_COLOR_BLACK); // 'inverted' text
  //  adcvalue = analogRead(VBAT);
  //  display.println(((float)adcvalue * mv_per_lsb * 4.0) / 1000.0); //1/4 resistor divider
  //  display.refresh();
  //delay(2000);
  wdt_feed();
  //************************************************************************

}

void loop(void)
{
  
  // readBckBtn(); //keep this in your main loop


  blePeripheral.poll();
  //wdt_reset();
  if (sleeping) {

    sd_app_evt_wait();
    sd_nvic_ClearPendingIRQ(SD_EVT_IRQn);
  } else {

    switch (menu) {
      case 0:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu0();
        }
        break;
      case 1:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu1();
        }
        break;
      case 2:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu2();
        }
        break;
      case 3:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu3();
        }
        break;
      case 4:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu4();
        }
        break;
      case 5:
        displayMenu5();
        break;
      case 77:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu77();
        }
        break;
      case 88:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu88();
        }
        break;
      case 99:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu99();
        }
        break;
    }
    if (buttonPressed) {
      buttonPressed = false;
      switch (menu) {
        case 0:
          menu = 1;
          break;
        case 1:
          menu = 2;
          break;
        case 2:
          menu = 3;
          break;
        case 3:
          menu = 4;
          break;
        case 4:
          //startbutton = millis();
          //while (!digitalRead(BUTTON_PIN)) {}
          //while (digitalRead(BTN_UP) == LOW && digitalRead(BTN_DOWN) == LOW && digitalRead(BTN_BCK) == LOW) {}
          //if (millis() - startbutton > 1000) {
          // delay(5000);

          
          //enter this menu with the DFU button combo pressed to enable bootloader mode
          if (digitalRead(BTN_UP) == LOW && digitalRead(BTN_DOWN) == LOW && digitalRead(BTN_BCK) == LOW) {
            NRF_POWER->GPREGRET = 0x01;
            sd_nvic_SystemReset();

            //while (1) {};
          } else {
            menu = 5;
          }
          break;
        case 5:
          menu = 0;
          break;
        case 77:
          menu = 0;
          break;
        case 88:
          menu = 0;
          break;
        case 99:
          digitalWrite(VIBRO, LOW);
          menu = 0;
          break;
      }
    }
    switch (menu) {
      case 0:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 1:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 2:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 3:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 4:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 5:
        if (millis() - sleepTime > 20000 ) powerDown();
        break;
      case 77:
        if (millis() - sleepTime > 3000 ) powerDown();
        break;
      case 88:
        if (millis() - sleepTime > 3000 ) powerDown();
        break;
      case 99:
        if (millis() - sleepTime > 6000 ) {
          digitalWrite(VIBRO, LOW);
          powerDown();
        }
        break;
    }
  }

  
  wdt_feed(); //keep this in your main loop to reset automatically after 10s if something goes wrong within loop()
  disp_toggle(); // the EXTCOMIN pin of the screen has to be toggled at least once per second
} //*************LOOP******************

//***********************************************FUNCTIONS***********************************************
// reload the watchdog timer that has been started by the bootloader and set for 10 seconds
void wdt_feed() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

//This needs to be run at least once per second
void disp_toggle() {
  extcominState = !extcominState;
  digitalWrite(EXTCOMIN, extcominState);
}



//implements long press for power off and short press for backlight toggle
void readBckBtn() {
  if (digitalRead(BTN_BCK) == LOW) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
      //button has been pressed for a long time
      //set any pin and wakeup logic here.
      //systemOff(BTN_UP, LOW); //implemented here:
      //C:\Users\[...]\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\0.8.5\cores\nRF5\wiring.c
    }
  } else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      } else {
        //button has been pressed for a short time
        bltState = !bltState;
        digitalWrite(BCKLT, bltState);
      }
      buttonActive = false;
    }
  }
}



//*******************************AARONS Functions 2***********************************+
void displayMenu0() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(bleSymbol);
  display.println(" Time and Batt:");
  display.println(GetDateTimeString() + String(second()));
  display.print(getBatteryLevel());
  display.print("%  ");
  adcvalue = analogRead(VBAT);
  //display.print(((float)adcvalue * mv_per_lsb * 4.0) / 1000.0); //1/4 resistor divider
  //display.print("V  ");
  display.println(analogRead(VBAT));
  display.println(contrast);
  display.refresh();
}

void displayMenu1() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Menue1 Mac:");
  char tmp[16];
  sprintf(tmp, "%04X", NRF_FICR->DEVICEADDR[1] & 0xffff);
  String MyID = tmp;
  sprintf(tmp, "%08X", NRF_FICR->DEVICEADDR[0]);
  MyID += tmp;
  display.println(MyID);
  display.refresh();
}

void displayMenu2() {
  display.setRotation(0);
  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  res[0] = softread();
  res[1] = softread();
  res[2] = softread();
  res[3] = softread();
  res[4] = softread();
  res[5] = softread();
  byte x = (int16_t)((res[1] << 8) | res[0]) / 128;
  byte y = (int16_t)((res[3] << 8) | res[2]) / 128;
  byte z = (int16_t)((res[5] << 8) | res[4]) / 128;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Menue2 PushMSG:");
  display.println(msgText);
  display.print(x);
  display.print(",");
  display.print(y);
  display.print(",");
  display.println(z);
  display.refresh();
}

void displayMenu3() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("CMD Length: ");
  display.println(answer.length());
  display.println(answer);
  display.refresh();
}
void displayMenu4() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Hello From Arduino");
  display.println("  :)");
  display.println("Press the DFU button combo ant then the middle button to enable bootloader mode...");
  display.refresh();
}
void displayMenu77() {
  display.setRotation(3);
  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(10, 0);
  display.println(bleSymbol);
  display.setCursor(0, 11);
  display.print(getBatteryLevel());
  display.println("%");

  display.setTextSize(3);
  display.setTextColor(LCD_COLOR_BLUE);
  display.setCursor(10, 70);
  if (hour() < 10) display.print("0");
  display.print(hour());
  display.print(":");
  //display.setCursor(4, 50);
  if (minute() < 10) display.print("0");
  display.print(minute());
  display.print(":");
  //display.setCursor(4, 70);
  if (second() < 10) display.print("0");
  display.print(second());
  display.print(" ");

  display.setCursor(10, 111);
  display.setTextSize(2);
  display.setTextColor(LCD_COLOR_RED);
  if (day() < 10) display.print("0");
  display.print(day());
  display.print(".");
  if (month() < 10) display.print("0");
  display.print(month());
  display.print(".");
  display.println(year());
  display.setTextColor(LCD_COLOR_BLACK);
  display.setTextSize(1);
  display.refresh();
}
void displayMenu88() {
  display.setRotation(3);
  display.clearDisplay();
  display.setCursor(60, 80);
  display.println("Charging");
  display.refresh();
}
void displayMenu99() {
  display.setRotation(0);
  digitalWrite(VIBRO, vibrationMode);
  if (vibrationMode)vibrationMode = false; else vibrationMode = true;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(msgText);
  display.refresh();
}
void displayMenu5() {
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  res[0] = softread();
  res[1] = softread();
  res[2] = softread();
  res[3] = softread();
  res[4] = softread();
  res[5] = softread();
  int x = (int16_t)((res[1] << 8) | res[0]);
  int y = (int16_t)((res[3] << 8) | res[2]);
  int z = (int16_t)((res[5] << 8) | res[4]);

  float accelX = x;
  float accelY = y;
  float accelZ = z;
  int16_t ax = -accelY / 256,      // Transform accelerometer axes
          ay =  accelX / 256,      // to grain coordinate space
          az = abs(accelZ) / 2048; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    v2 = (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0 * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(256.0 * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  uint16_t        i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {              // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if (newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
    newidx = (newy      / 256) * WIDTH + (newx      / 256); // New pixel #
    if ((oldidx != newidx) && // If grain is moving to a new pixel...
        img[newidx]) {       // but if that pixel is already occupied...
      delta = abs(newidx - oldidx); // What direction when blocked?
      if (delta == 1) {           // 1 pixel left or right)
        newx         = grain[i].x;  // Cancel X motion
        grain[i].vx /= -2;          // and bounce X velocity (Y is OK)
        newidx       = oldidx;      // No pixel change
      } else if (delta == WIDTH) { // 1 pixel up or down
        newy         = grain[i].y;  // Cancel Y motion
        grain[i].vy /= -2;          // and bounce Y velocity (X is OK)
        newidx       = oldidx;      // No pixel change
      } else { // Diagonal intersection is more tricky...
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if (!img[newidx]) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            grain[i].vy /= -2;         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              grain[i].vx /= -2;         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if (!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            grain[i].vy /= -2;         // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 255;  // Set new spot
    grain[i].pos = newidx;
  }

  display.clearDisplay();
  for (i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    display.drawPixel(xPos , yPos, LCD_COLOR_BLACK);
  }
  display.refresh();
}
