#define TINY_GSM_MODEM_SIM7600   //SIMA7670 Compatible with SIM7600 AT instructions
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

const char apn[]  = "YOUR-APN";     //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

// set GSM PIN, if any
#define GSM_PIN "1234"


#include "SMSmodem.h"
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
SMSTinyGsm modem(debugger);
#else
SMSTinyGsm modem(SerialAT);
#endif

#define UART_BAUD    115200
#define PIN_DTR      25
#define PIN_TX       26
#define PIN_RX       27
#define PWR_PIN      4
#define BAT_ADC      35
#define BAT_EN       12
#define PIN_RI       33
#define RESET        5

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13

void IRAM_ATTR sms_incomming(){
  //SerialMon.println("Incomming message");
}


void setup() {
  // put your setup code here, to run once:
    // Set console baud rate
    SerialMon.begin(UART_BAUD);
    delay(10);

    SerialMon.println("setup...");

    pinMode(BAT_EN, OUTPUT);
    digitalWrite(BAT_EN, HIGH);

    //A7670 Reset
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, LOW);
    delay(100);
    digitalWrite(RESET, HIGH);
    delay(3000);
    digitalWrite(RESET, LOW);

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_PIN, LOW);

    delay(1000);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    if (!modem.init()) {
        SerialMon.println("Failed to restart modem, attempting to continue without restarting");
    }

    String name = modem.getModemName();
    delay(500);
    SerialMon.println("Modem Name: " + name);

    String modemInfo = modem.getModemInfo();
    delay(500);

    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
      modem.simUnlock(GSM_PIN);
    }    

    attachInterrupt(PIN_RI,sms_incomming,FALLING);

    DBG("Waiting for network...");
    while(!modem.waitForNetwork()) {
        delay(1000);
    }

    if (modem.isNetworkConnected()) {
        DBG("Network connected");
    }

    DBG("Enabling GPS/GNSS/GLONASS and waiting 15s for warm-up");
    //modem.enableGPS();
    //delay(15000L);

    //Disable gnss
    modem.sendAT("+CGNSSPWR=0");
    modem.waitResponse(10000L);

    //Enable gnss
    modem.sendAT("+CGNSSPWR=1");
    modem.waitResponse(10000L);

    //Wait gnss start.
    SerialMon.print("\tWait GPS reday.");
    while (modem.waitResponse(1000UL, "+CGNSSPWR: READY!") != 1) {
        SerialMon.print(".");
    }
    SerialMon.println();

    //Set gnss mode use GPS.
    modem.sendAT("+CGNSSMODE=1");
    modem.waitResponse(10000L);

    //modem.sendAT("+CNMI=1,2,0,0,0");
    modem.sendAT(GF("+CMGF=1"));
    modem.waitResponse();

}

int incomingByte = 0;

char dat[30];



void loop() {
  // put your main code here, to run repeatedly:

  

  int index = modem.waitResponse(GF(GSM_NL "+CMT:"),GF(GSM_NL "RING"));

  if (index==1){

  String SenderNum;
  String Text;

  modem.waitResponse(1000,SenderNum,GF(","));
  SenderNum.remove(SenderNum.length()-2,2);
  SenderNum.remove(0,2);
  SerialMon.println(SenderNum);

  modem.waitResponse(GF(","));

  modem.waitResponse(GF("\r")); //GF(static_cast<char>(0x0D))

  modem.waitResponse(1000,Text,GF("\r"));

  DBG("TextSend: ",Text);

  bool res = modem.sendSMS(SenderNum, String("Request Received GPS pooling..."));
  DBG("SMS:", res ? "OK" : "fail");
  float parameter1,  parameter2;
  char buf[16];
  while (1) {
        if (modem.getGPS(&parameter1, &parameter2)) {
            modem.sendAT(GF("+CGNSSINFO"));
            if (modem.waitResponse(GF(GSM_NL "+CGNSSINFO:")) == 1) {
                String res = modem.stream.readStringUntil('\n');
                String lat = "";
                String n_s = "";
                String lon = "";
                String e_w = "";
                res.trim();
                lat = res.substring(8, res.indexOf(',', 8));
                n_s = res.substring(19, res.indexOf(',', res.indexOf(',', 19)));
                lon = res.substring(21, res.indexOf(',', res.indexOf(',', 21)));
                e_w = res.substring(33, res.indexOf(',', res.indexOf(',', 33)));
                delay(100);
                Serial.println("****************GNSS********************");
                Serial.printf("lat:%s %s\n", lat, n_s);
                Serial.printf("lon:%s %s\n", lon, e_w);
                float flat = atof(lat.c_str());
                float flon = atof(lon.c_str());
                flat = (floor(flat / 100) + fmod(flat, 100.) / 60) *
                       (n_s == "N" ? 1 : -1);
                flon = (floor(flon / 100) + fmod(flon, 100.) / 60) *
                       (e_w == "E" ? 1 : -1);
                Serial.print("Latitude:"); Serial.println(flat);
                Serial.print("Longitude:"); Serial.println(flon);

                String GpsSMS = "Coord : \n lat : ";
                GpsSMS += lat+ "\n long : "+lon + "\n" ;
                GpsSMS += "www.google.com/maps/search/"+lat+","+lon;
                bool resSms = modem.sendSMS(SenderNum, GpsSMS);
                DBG("SMS:", resSms ? "OK" : "fail");

            }
            break;
        } else {
            Serial.print("getGPS ");
            Serial.println(millis());
        }
        delay(2000);
    }

  // float lat2      = 0;
  // float lon2      = 0;
  // float speed2    = 0;
  // float alt2      = 0;
  // int   vsat2     = 0;
  // int   usat2     = 0;
  // float accuracy2 = 0;
  // int   year2     = 0;
  // int   month2    = 0;
  // int   day2      = 0;
  // int   hour2     = 0;
  // int   min2      = 0;
  // int   sec2      = 0;
  // for (int8_t i = 15; i; i--) {
  //   DBG("Requesting current GPS/GNSS/GLONASS location");
  //   if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
  //                    &year2, &month2, &day2, &hour2, &min2, &sec2)) {
  //     DBG("Latitude:", String(lat2, 8), "\tLongitude:", String(lon2, 8));
  //     DBG("Speed:", speed2, "\tAltitude:", alt2);
  //     DBG("Visible Satellites:", vsat2, "\tUsed Satellites:", usat2);
  //     DBG("Accuracy:", accuracy2);
  //     DBG("Year:", year2, "\tMonth:", month2, "\tDay:", day2);
  //     DBG("Hour:", hour2, "\tMinute:", min2, "\tSecond:", sec2);



  //     break;
  //   } else {
  //     DBG("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
  //     delay(15000L);
  //   }
  // }

  }

  // SerialMon.println(dat);

  // for(int i=0;i<sizeof(dat);i++){
  //     SerialMon.print(dat[i],HEX);
  // }
  // SerialMon.print("\n");
  // if(SerialAT.available()>1){
  // SerialMon.println(SerialAT.read(),HEX);}
  

}
