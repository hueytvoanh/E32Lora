/*
 * LoRa E32-TTL-100
 * Get configuration.
 * https://www.mischianti.org/2019/10/29/lora-e32-device-for-arduino-esp32-or-esp8266-configuration-part-3/
 *
 * E32-TTL-100----- Arduino UNO
 * M0         ----- 3.3v
 * M1         ----- 3.3v
 * TX         ----- RX PIN 2 (PullUP)
 * RX         ----- TX PIN 3 (PullUP & Voltage divider)
 * AUX        ----- Not connected
 * VCC        ----- 3.3v/5v
 * GND        ----- GND
 *
 */
#include "Arduino.h"
#include "LoRa_E32.h"
#include <Wire.h>
#include <U8g2lib.h>

#define M0_PIN         16
#define M1_PIN         17 
#define RX_PIN 19
#define TX_PIN 19

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SYSTEM_IDLE           0
#define SYSTEM_RUNNING        1
#define TIME_GO2_RUNNING      20000

#define SYSTEM_IDLE           2
#define SYSTEM_0              0
#define SYSTEM_1              1

#define SEND_NONE             2
#define SEND_OK               0
#define SEND_NOK              1
#define LED_PIN               25
#define SENSOR_PIN            4

#define SEND_PERIOD           60000
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//LoRa_E32 e32ttl100(2, 3); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
LoRa_E32 e32ttl100(&Serial1);
struct Message {
    char type[5];
    char message[8];
    byte temperature[4];
};




#ifndef PMU_WIRE_PORT
#define PMU_WIRE_PORT           Wire
#endif

#ifndef DISPLAY_ADDR
#define DISPLAY_ADDR            0x3C
#endif

#ifndef DISPLAY_MODEL
#define DISPLAY_MODEL           U8G2_SSD1306_128X64_NONAME_F_HW_I2C
#endif

U8G2_SSD1306_128X64_NONAME_F_HW_I2C* u8g2;




volatile int systemState, sendStatus, sendingIssue;
volatile boolean mainIssue;
unsigned long startRunningTime, checkRunningTime; 

TaskHandle_t e32ReceiveTask;
TaskHandle_t loraSendTask;
TaskHandle_t systemStateTask;
TaskHandle_t uartTask;
TaskHandle_t oledTask;
TaskHandle_t sensorTask;

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);






void oledTaskFunction( void * pvParameters ) {
  while (1) {
    if (u8g2 != NULL) {
      //Serial.println("Updating OLED");
      u8g2->clearBuffer();
      u8g2->drawStr(0, 12, "MASTER NODE");
      if(systemState == SYSTEM_0){
          u8g2->drawStr(0, 26, "MAIN IS OUT");
          digitalWrite(LED_PIN, LOW);  
      }else{
          u8g2->drawStr(0, 26, "MAIN IS OK");
          digitalWrite(LED_PIN, HIGH);
      }
      //u8g2->drawStr(0, 26, String(systemState).c_str());

      if (sendStatus == SEND_OK) {
        u8g2->drawStr(0, 40, "SENT OK");
      } else if (sendStatus == SEND_NOK) {
        u8g2->drawStr(0, 40, "SENT NOK");
      } else {
        u8g2->drawStr(0, 40, "-------");
      }
      u8g2->sendBuffer();
    } else {
      Serial.println("u8g2 is NULL");
    }
    delay(1000);
  }
}


void loraSendTaskFunction( void * pvParameters ) {
  int i = 0;
  while (1) {
    checkRunningTime = millis();

    if(((checkRunningTime-startRunningTime) > SEND_PERIOD)||(sendingIssue == true)){
        Serial.print("Begin Sending packet: ");
        if(sendingIssue == true){
          i = 0;
        }
        else{
          i = 2;
        }  
            for(; i <3; i++){
                #ifdef LILYGO
                LoRa.beginPacket();
                #endif
                if(systemState == SYSTEM_0){ 
                     #ifdef LILYGO
                     LoRa.print("MASTER 0");
                     #endif
                }
                if(systemState == SYSTEM_1){ 
                    #ifdef LILYGO
                     LoRa.print("MASTER 1");
                     #endif
                }

                #ifdef LILYGO
                if (LoRa.endPacket() == 1) {
                    Serial.println("Packet sent successfully");
                    sendStatus = SEND_OK;
                    sendingIssue = false;
                } else {
                    Serial.println("Failed to send");
                    sendStatus = SEND_NOK;
                }
                #endif
             }
        
        startRunningTime = millis();
    }
    
  }
}

void systemStateTaskFunction( void * pvParameters ) {
  while (1) {      
      switch(systemState){
          case SYSTEM_IDLE:
              if(mainIssue == true){
                   Serial.println("Main OUT");            
                   systemState = SYSTEM_0;
              }
              else{
                   Serial.println("Main IS OK");
                   systemState = SYSTEM_1;
              }              
              break;                       
          case SYSTEM_0:                
              if(mainIssue == false){
                   systemState = SYSTEM_1;
                   sendingIssue = true;            
              }
              break;
          case SYSTEM_1:                
              if(mainIssue == true){
                   systemState = SYSTEM_0;
                   sendingIssue = true;            
              }
              break;
          default:
              break;
             
      }
      delay(500);
  }
}

void sensorTaskFunction( void * pvParameters ) {
   while (1) {
       if(digitalRead(SENSOR_PIN) == LOW){
           mainIssue = false; 
       }
       else{
           mainIssue = true;
       }
       delay(100);
   }
}











void configIo(void){
    pinMode(M0_PIN , OUTPUT); 
    pinMode(M1_PIN , OUTPUT);   
    digitalWrite(M0_PIN, HIGH);
    digitalWrite(M1_PIN, HIGH);
    delay(5000);
}

void setup() {

  systemState = SYSTEM_IDLE;
  mainIssue = true;
  sendingIssue = false;
  startRunningTime = millis();
    
  configIo();
	Serial.begin(115200);
	delay(500);
  Wire.begin();

  
  
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  //Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("UART1 initialized on pins 38 (TX) and 39 (RX)");

  //LoRa_E32 e32ttl100(&Serial1,41, M0_PIN, M1_PIN); 
  //LoRa_E32 e32ttl100(&Serial1); 

	beginDisplay();
 
	e32ttl100.begin();

	ResponseStructContainer c;
	c = e32ttl100.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);

	ResponseStructContainer cMi;
	cMi = e32ttl100.getModuleInformation();
	// It's important get information pointer before all other operation
	ModuleInformation mi = *(ModuleInformation*)cMi.data;

	Serial.println(cMi.status.getResponseDescription());
	Serial.println(cMi.status.code);

	printModuleInformation(mi);

	c.close();
	cMi.close();

  setConfigE32();


  xTaskCreatePinnedToCore(
   e32ReceiveTaskFunction,
   "e32ReceiveTaskFunction",
   10000,
   NULL,
   1,
   &e32ReceiveTask,
   1);


  xTaskCreatePinnedToCore(
    loraSendTaskFunction,
    "loraSendTaskFunction",
    10000,
    NULL,
    1,
    &loraSendTask,
    1);

    xTaskCreatePinnedToCore(
    systemStateTaskFunction,
    "systemStateTaskFunction",
    10000,
    NULL,
    1,
    &systemStateTask,
    1);

    xTaskCreatePinnedToCore(
    oledTaskFunction,
    "oledTaskFunction",
    10000,
    NULL,
    1,
    &oledTask,
    1);

    xTaskCreatePinnedToCore(
    sensorTaskFunction,
    "sensorTaskFunction",
    10000,
    NULL,
    1,
    &sensorTask,
    1);




   
}

void e32ReceiveTaskFunction( void * pvParameters ){

  while(1==1){
      if (e32ttl100.available()  > 1){
          ResponseStructContainer rsc = e32ttl100.receiveMessage(sizeof(Message));
          struct Message message = *(Message*) rsc.data;
          Serial.println(message.type);

          Serial.println(*(float*)(message.temperature));
          Serial.println(message.message);
          //    free(rsc.data);
          rsc.close();
      }
    
  }
}


void setConfigE32(){
  ResponseStructContainer c;
  c = e32ttl100.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*) c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);
  configuration.ADDL = 0x0;
  configuration.ADDH = 0x1;
  configuration.CHAN = 0x17;

  configuration.OPTION.fec = FEC_0_OFF;
  configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  configuration.OPTION.transmissionPower = POWER_17;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_1250;

  configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;
  configuration.SPED.uartBaudRate = UART_BPS_115200;
  configuration.SPED.uartParity = MODE_00_8N1;

  // Set configuration changed and set to not hold the configuration
  ResponseStatus rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);
  printParameters(configuration);
  c.close();
}

void sendMessageE32(){
     Serial.println("Send message to 00 03 04");
     ResponseStatus rs = e32ttl100.sendFixedMessage(0, 3, 0x04, "Message to 00 03 04 device");
     Serial.println(rs.getResponseDescription()); 
     Serial.println("End Sending");
}

void loop() {
     
     sendMessageE32();
     delay(5000);
}
void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD BIN: "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH BIN: "));  Serial.println(configuration.ADDH, BIN);
	Serial.print(F("AddL BIN: "));  Serial.println(configuration.ADDL, BIN);
	Serial.print(F("Chan BIN: "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit BIN    : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDataRate BIN : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate BIN  : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans BIN       : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup BIN      : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup BIN      : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC BIN         : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower BIN       : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

	Serial.println("----------------------------------------");

}
void printModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	Serial.print(F("HEAD BIN: "));  Serial.print(moduleInformation.HEAD, BIN);Serial.print(" ");Serial.print(moduleInformation.HEAD, DEC);Serial.print(" ");Serial.println(moduleInformation.HEAD, HEX);

	Serial.print(F("Freq.: "));  Serial.println(moduleInformation.frequency, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");

}


bool beginDisplay()
{
    Wire.beginTransmission(DISPLAY_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.printf("=================================================================================================Find Display model at 0x%X address\n", DISPLAY_ADDR);
        //u8g2 = new DISPLAY_MODEL(U8G2_R0, U8X8_PIN_NONE);
        u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
        u8g2->begin();
        u8g2->clearBuffer();
        u8g2->setFont(u8g2_font_inb19_mr);
        u8g2->drawStr(0, 30, "SUNNY");
        u8g2->drawHLine(2, 35, 47);
        u8g2->drawHLine(3, 36, 47);
        u8g2->drawVLine(45, 32, 12);
        u8g2->drawVLine(46, 33, 12);
        u8g2->setFont(u8g2_font_inb19_mf);
        u8g2->drawStr(58, 60, "Elec");
        u8g2->sendBuffer();
        u8g2->setFont(u8g2_font_fur11_tf);
        delay(3000);
        return true;
    }

    Serial.printf("Warning: Failed to find Display at 0x%0X address\n", DISPLAY_ADDR);
    return false;
}
