// ultrasonic sensor on A0.  Also monitors battery usage.  Need two fields on thingspeak channel.
// Portions of this code were developed by the Sodaq team

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <RTCTimer.h>
#include <SPI.h>
#include <SD.h>
#include <GPRSbee.h>

//Network constants
#define APN "safaricom"
#define APN_USERNAME "saf"
#define APN_PASSWORD "data"

//battery constants
#define ADC_AREF 3.3
#define BATVOLTPIN A6
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

//SpeakThings constants
#define URL "api.thingspeak.com/update"
#define WRITE_API_KEY "XXXXXXXXXXXX" //Change to your channel's key

RTCTimer timer;           // Instantiate the timer

bool hz_flag;

const int chipSelect = 11;

File dataFile;

int arraysize = 9;  //quantity of values to find the median (sample size). Needs to be an odd number
int rangevalue[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};    //declare an array to store the samples. not necessary to zero the array values here, it just makes the code clearer

void setup()
{
  
  wdt_disable();
  
  Serial.begin(57600);
  Serial1.begin(57600);
  Wire.begin();
  rtc.begin();
  
  pinMode(LED2, OUTPUT);

  //Intialise the GPRSbee
  gprsbee.init(Serial1, BEECTS, BEEDTR);

  //This is required for the Switched Power method
  gprsbee.setPowerSwitchedOnOff(true); 

  // Make sure the GPRSbee is switched off
  doCheckGPRSoff(0);

  // Do the work every 2 minutes
  timer.every(120, takedata);

  // Instruct the RTCTimer how to get the current timestamp
  timer.setNowCallback(getNow);
  
  Serial.print("Initializing SD card...");
  pinMode(11, OUTPUT);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
  setupWatchdog();

  interrupts();
  
  Serial.println("Setup complete");
  
  wdt_reset();
  gprsbee.on();    

  FiveSecondDelay();
  FiveSecondDelay();
  FiveSecondDelay(); 

  Serial1.print(F("AT+CMGF=1\r"));                     //line for sms mode
  delay(100);
  Serial1.println(F("AT + CMGS = \"+XXXXXXXXXXXXXX\""));          //my number for testing
  delay(100);
  Serial1.print(F("Sodaq Mbili Water Station V4 has reinitialized"));
  Serial.println(F("Sodaq Mbili Water Station V4 has reinitialized"));
  Serial1.println((char)26);                        //the ASCII code of the ctrl+z is 26, which closes the sms
  wdt_reset();
  FiveSecondDelay();
  wdt_reset();  
  Serial1.println();
                          
  wdt_reset();
  gprsbee.off();  
}

void loop()
{
  if (hz_flag) {
    hz_flag = false;
    wdt_reset();
    WDTCSR |= _BV(WDIE);
  }
  
  timer.update();

  systemSleep();
}

void takedata(uint32_t ts)
{
  wdt_reset();
    
  DateTime now = rtc.now(); //get the current date-time   
  
  FiveSecondDelay(); // delay to stabilize
  
  for(int i = 0; i < arraysize; i++)
   {                                                    //array pointers go from 0 to 4
     rangevalue[i] = analogRead(A0);
     delay(100);  
    }  
  
    int midpoint = arraysize/2;   
      
    int depthvalue = 400 - rangevalue[midpoint];    // corrected distance for Emarti

//   int depthvalue = rangevalue[midpoint];  // raw distance
    
    delay(100);
    
   int mv = getRealBatteryVoltage() * 1000.0;
      
  String url = "";
    
  gprsbee.on(); 
 
  FiveSecondDelay();
  FiveSecondDelay();
  FiveSecondDelay();
    
  url += String("api.thingspeak.com/update");
  url += String("?key=");
  url += String(WRITE_API_KEY);
  
  url += String("&field1=");
  url += String(depthvalue);
  
  url += String("&field2=");
  url += String(mv);
  
  url += String("&created_at=") + String(now.year(), DEC) + String("/") + String(now.month(), DEC) + String("/") + String(now.date(), DEC) + String("T") + String(now.hour(), DEC) + String(":") + String(now.minute(), DEC) + String(":") + String(now.second(), DEC) + String("&timezone=Africa/Nairobi");
  
  Serial.println(url);
  
  char result[20] = "";
  
  gprsbee.doHTTPGET(APN, APN_USERNAME, APN_PASSWORD, url.c_str(), result, sizeof(result));
  
  wdt_reset();
  FiveSecondDelay();
  wdt_reset();
    
  Serial.println("Received: " + String(result));
  
  gprsbee.off();
  
   dataFile = SD.open("datalog.txt", FILE_WRITE);
   if (! dataFile) {
    Serial.println(F("error opening datalog.txt"));
    // Wait forever since we cant write data
    }
   
    dataFile.print(now.year(), DEC);
    dataFile.print('/');
    dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.date(), DEC);
    dataFile.print(',');
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.print(now.second(), DEC);
    dataFile.print(',');
    dataFile.print(depthvalue);
    dataFile.print(',');
    dataFile.println(mv);
    
    delay(100);

    dataFile.flush();
  
  wdt_reset();
  
  delay(500);
}

void FiveSecondDelay()
{
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);  
  wdt_reset();
}

/*
 * Return the current timestamp
 *
 * This is a general purpose wrapper function to get the current timestamp.
 * It can also be used for timer.setNowCallback
 */
uint32_t getNow()
{
  return rtc.now().getEpoch();
}

//######### watchdog and system sleep #############
void systemSleep()
{
  ADCSRA &= ~_BV(ADEN);         // ADC disabled

  /*
  * Possible sleep modes are (see sleep.h):
  #define SLEEP_MODE_IDLE         (0)
  #define SLEEP_MODE_ADC          _BV(SM0)
  #define SLEEP_MODE_PWR_DOWN     _BV(SM1)
  #define SLEEP_MODE_PWR_SAVE     (_BV(SM0) | _BV(SM1))
  #define SLEEP_MODE_STANDBY      (_BV(SM1) | _BV(SM2))
  #define SLEEP_MODE_EXT_STANDBY  (_BV(SM0) | _BV(SM1) | _BV(SM2))
  */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  /*
   * This code is from the documentation in avr/sleep.h
   */
  cli();
  // Only go to sleep if there was no watchdog interrupt.
  if (!hz_flag)
  {
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
  sei();

  ADCSRA |= _BV(ADEN);          // ADC enabled
}
// The watchdog timer is used to make timed interrupts
// This is a modified version of wdt_enable() from avr/wdt.h
// The avr/wdt.h version only sets WDE (not WDIE for interrupts)
#if 1
// Both WDE and WDIE are set!!
// Note from the doc: "Executing the corresponding interrupt
// vector will clear WDIE and WDIF automatically by hardware
// (the Watchdog goes to System Reset Mode)
#define my_wdt_enable(value)   \
__asm__ __volatile__ (  \
    "in __tmp_reg__,__SREG__" "\n\t"    \
    "cli" "\n\t"    \
    "wdr" "\n\t"    \
    "sts %0,%1" "\n\t"  \
    "out __SREG__,__tmp_reg__" "\n\t"   \
    "sts %0,%2" "\n\t" \
    : /* no outputs */  \
    : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
      "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
      "r" ((uint8_t) (((value & 0x08) ? _WD_PS3_MASK : 0x00) | \
          _BV(WDE) | _BV(WDIE) | (value & 0x07)) ) \
    : "r0"  \
)
#else
// Only WDIE is set!!
#define my_wdt_enable(value)   \
__asm__ __volatile__ (  \
    "in __tmp_reg__,__SREG__" "\n\t"    \
    "cli" "\n\t"    \
    "wdr" "\n\t"    \
    "sts %0,%1" "\n\t"  \
    "out __SREG__,__tmp_reg__" "\n\t"   \
    "sts %0,%2" "\n\t" \
    : /* no outputs */  \
    : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
      "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
      "r" ((uint8_t) (((value & 0x08) ? _WD_PS3_MASK : 0x00) | \
          _BV(WDIE) | (value & 0x07)) ) \
    : "r0"  \
)
#endif

void setupWatchdog()
{
  my_wdt_enable(WDTO_1S);
}

//################ interrupt ################
ISR(WDT_vect)
{
  hz_flag = true;
}

void doCheckGPRSoff(uint32_t now)
{
  // Maybe we shouldn't care about WDT. If it does not switch
  // off properly it could be a good thing to reset the system.
  // TODO We need to verify this somehow.
  gprsbee.off();
  for (uint8_t i = 0; i < 10; ++i) {
    delay(50);
    if (gprsbee.off()) {
      break;
    }
  }
}

float getRealBatteryVoltage()
{
   uint16_t batteryVoltage = analogRead(BATVOLTPIN);

   return (ADC_AREF / 1023.0) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage; 
}
