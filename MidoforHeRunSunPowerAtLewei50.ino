/*************************************************** 
 * This is an example for the DFRobot Wido - Wifi Integrated IoT lite sensor and control node
 * Product Page & More info: http://www.dfrobot.com.cn/goods-997.html
 * Designed specifically to work with the DFRobot Wido products:
 * 
 * The library is forked from Adafruit
 * 
 * Written by Lauren
 * BSD license, all text above must be included in any redistribution
 * 
 * Modfiy by bill deng
 ****************************************************/
 
/*
This example code is used to connect the Lewei50 cloud service (Official homepage: www.lewei50.com).

 The device required is just:
 
 1.
 2. And Wido

Note: Please don't forget to change the setting below before using!
 1. WLAN_SSID & WlAN_PASS
 2. userkey
 3. GATE ID

 */


#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <avr/wdt.h>
//#include <SHT1x.h>
//#include "utility/debug.h"

#define Wido_IRQ   7
#define Wido_VBAT  5
#define Wido_CS    10
#define LED_PIN  13  

Adafruit_CC3000 Wido = Adafruit_CC3000(Wido_CS, Wido_IRQ, Wido_VBAT,
SPI_CLOCK_DIVIDER); // you can change this clock speed
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2


#define WLAN_SSID       "herunsun1"           // cannot be longer than 32 characters!
#define WLAN_PASS       "qwertyuiop"          // For connecting router or AP, don't forget to set the SSID and password here!!


#define TCP_TIMEOUT      30000
//#define CC3000_TINY_DRIVER

#define WEBSITE  "www.lewei50.com"
#define userkey  "dab6a862154c4ebfab05b845eb4b5652"  // Update Your API Key. To get your API Key, please check the link below
                                                     //https://www.lewei50.com/user/clientindex
#define analogInPin  A0  // Analog input pin that the potentiometer is attached to
// Specify data and clock connections and instantiate SHT1x object
//#define dataPin  3
//#define clockPin 2
//SHT1x sht1x(dataPin, clockPin);

static unsigned long widoruntimeStamp = 0;



void softReset(){
asm volatile ("jmp 0");
}

void setup(){
//  wdt_enable(WDTO_8S); //看门狗使能 并设置超时时间 8s
  wdt_disable(); //看门狗禁止
//  wdt_reset(); //看门狗复位
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // sets the LED off
  Serial.begin(115200);    //USB ON Serial
  Serial1.setTimeout(3000);  //set Serial1 Timeout value=3000ms
  Serial1.begin(19200);    //concet to UPS
  
  widoruntimeStamp = millis() ;

  if (!Wido.begin())
  {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    digitalWrite(LED_PIN, HIGH);   // sets the LED on
    delay(1000);
    digitalWrite(LED_PIN, LOW);   // sets the LED off
    delay(500);
    softReset();
  }
  for (byte i=0;i<10;i++){
    digitalWrite(LED_PIN, HIGH);   // sets the LED on
    delay(400);
    digitalWrite(LED_PIN, LOW);   // sets the LED off
    delay(200);
  }
  
//  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
//  Serial.print(F("\nAttempting to connect to ")); 
  Serial.println(ssid);

  /* NOTE: Secure connections are not available in 'Tiny' mode!
   By default connectToAP will retry indefinitely, however you can pass an
   optional maximum number of retries (greater than zero) as the fourth parameter.
   */
  if (!Wido.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY,3)) {
    Serial.println(F("Connect To AP Failed!"));
      for (byte i=0;i<10;i++){
      digitalWrite(LED_PIN, HIGH);   // sets the LED on
      delay(100);
      digitalWrite(LED_PIN, LOW);   // sets the LED off
      delay(300);
    }
    softReset();
  }
  
  Serial.println(F("Connected!"));
  wdt_enable(WDTO_8S); //看门狗使能 并设置超时时间 8s
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!Wido.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

}

uint32_t ip = 0;
unsigned int  BatteryVoltage = 0;
unsigned int  OutACVoltage = 0;
unsigned int  InACVoltage = 0;
int  BatteryAmp = 0;
unsigned int  LOAD = 0;
unsigned int  UPState = 0;
boolean Updata = false ;  //数据更新标志：=0无更新数据
//float Temperature = 0;
//float Humidity = 0;


void loop(){
  static Adafruit_CC3000_Client WidoClient;
  static unsigned long RetryMillis = 0;
  static unsigned long uploadtStamp = 0;
  static unsigned long sensortStamp = 0;
  static unsigned long CnnectedStamp = 0;

  
  unsigned int IncominDataLength = 0;
  char IncominBuffer[15] = "";  //{"/x01/x10/x80/x00/x00/x08/xB2/x02/x1F/x00/x64/xF4/x0D/xA8"} ;

  wdt_reset();   //看门狗复位
  
  if((millis() - widoruntimeStamp) > 3600000){
    Serial.print(F("Time To Reset......"));
    Serial.println(millis());
    delay(1000);
    softReset();
  }

  if(millis() - sensortStamp > 3000){    //DATA I/O SCAN
    sensortStamp = millis();
    
//    Temperature = sht1x.readTemperatureC();
//    Humidity = sht1x.readHumidity();
    
    digitalWrite(LED_PIN, HIGH);   // sets the LED on
    Serial1.print("\x01\x10\x01\xEC");    //Poll UPS StateCommand
    IncominDataLength =Serial1.readBytes(IncominBuffer,14);
    if  ((IncominDataLength == 14 ) && (IncominBuffer[0] == 1))  {
      BatteryVoltage = (unsigned char)(IncominBuffer[7]) ;
      BatteryVoltage <<= 8;
      BatteryVoltage += (unsigned char)(IncominBuffer[8]);

      OutACVoltage = (unsigned char)(IncominBuffer[5]);
      OutACVoltage <<= 8;
      OutACVoltage += (unsigned char)(IncominBuffer[6]);
     
      InACVoltage = (unsigned char)(IncominBuffer[3]);
      InACVoltage <<= 8;
      InACVoltage += (unsigned char)(IncominBuffer[4]);
      
      UPState  = (unsigned char)(IncominBuffer[2]) ;
      
      // read the analog in value:
      BatteryAmp = analogRead(analogInPin);
      
      Updata = true ;
    }
    Serial1.print("\x59\x0D");    //Poll UPS StateCommand
    IncominDataLength =Serial1.readBytes(IncominBuffer,8);
    if  ((IncominDataLength == 8 ) && (IncominBuffer[7] == 0x0D))  {
      LOAD = (unsigned char)(IncominBuffer[4]) * 6 ;  //(LOAD% * 6KW )
    }
    digitalWrite(LED_PIN, LOW);   // sets the LED off
  }
  
  wdt_reset();   //看门狗复位
  
  if(!WidoClient.connected() && (millis() - RetryMillis > TCP_TIMEOUT)){  
    // Update the time stamp
    RetryMillis = millis();
    
    Serial.println(F("Try to connect the cloud server"));
    WidoClient.close();

    // Get Lewei50 IP address
    Serial.print(F("www.lewei50.com -> "));
    while  (ip  ==  0)  {
      if  (!Wido.getHostByName(WEBSITE, &ip))  {    //  Get the server IP address based on the domain name
        Serial.println(F("Couldn't resolve!"));
      }
      delay(500);
    }  
    Wido.printIPdotsRev(ip);
    Serial.println(F(""));
    // Connect to the Lewei50 Server
    WidoClient = Wido.connectTCP(ip, 80);          // Try to connect cloud server
    if (WidoClient.connected()){
      CnnectedStamp = millis();
    }
    if ((millis() - CnnectedStamp)>180000 ){
      Serial.print(F("Time To Reset......"));
      Serial.println(millis());
      delay(1000);
      softReset();
    }
  }
  else{        //检查连接丢失是否超过了3分钟
    WidoClient.close();      // Close the service connection
  }
  wdt_reset();   //看门狗复位
  
  if(WidoClient.connected() && (millis() - uploadtStamp > 10000) && Updata ){
    Updata = false;
    uploadtStamp = millis();
    // If the device is connected to the cloud server, upload the data every 10000ms.
    wdt_reset();   //看门狗复位
    // Prepare Http Package for Lewei50 & get length
    int length = 0;
    char lengthstr[4] = "";
    char BVChar[6] = "",OVChar[4] ="",BIChar[7] ="",LAChar[5] ="",USChar[4] ="";    
    
    itoa(BatteryVoltage/10,BVChar,10);  // push the data to the http data package
    strcat(BVChar,".");
    itoa(BatteryVoltage%10,BVChar+strlen(BVChar),10);
    length += strlen(BVChar);

    itoa(OutACVoltage/10,OVChar,10);  // push the data to the http data package
    length += strlen(OVChar);
    
    BatteryAmp = BatteryAmp -517;      // Zero Ponint cut
    if (abs(BatteryAmp) < 6){
      BatteryAmp = 0;
    }
    BatteryAmp = BatteryAmp * 2;    //0.02v = 1A
    itoa(BatteryAmp/10,BIChar,10);  // push the data to the http data package
    strcat(BIChar,".");
    itoa(abs(BatteryAmp)%10,BIChar+strlen(BIChar),10);
    length += strlen(BIChar);
    
    itoa(LOAD/100,LAChar,10);  // push the data to the http data package
    strcat(LAChar,".");
    itoa(LOAD%100,LAChar+strlen(LAChar),10);
    length += strlen(LAChar);
    
    itoa(UPState,USChar,10);  // push the data to the http data package
    length += strlen(USChar);
    
    Serial.println(F("Connected to Lewei50 server."));
    // Send headers
    Serial.print(F("Sending headers"));
    
    WidoClient.fastrprint(F("POST /api/V1/gateway/UpdateSensors/01")); 
                                                            //Please change your Gateway ID  here, after creating
                                                           //Please check the link:https://www.lewei50.com/user/clientindex
    WidoClient.fastrprintln(F(" HTTP/1.1"));
    Serial.print(F("."));
    
    WidoClient.fastrprintln(F("Host: www.lewei50.com"));
    Serial.print(F("."));
    
    WidoClient.fastrprint(F("userkey: "));
    WidoClient.fastrprintln(userkey);
    Serial.print(F("."));

    length += 126;                           // get the length of data package
    itoa(length,lengthstr,10);               // convert int to char array for posting
    Serial.print(F("Length = "));
    Serial.println(length);
    
    WidoClient.fastrprint("Content-Length: "); 
    WidoClient.fastrprintln(lengthstr);
    WidoClient.fastrprintln("");
    Serial.print(F("."));
    
    Serial.println(F(" done."));
    
    // Send data
/***************************************************************************************************
*  make a DataPakg:
* [
*    {"Name":"BV","Value":"55.6"},
*    {"Name":"OV","Value":"220"},
*    {"Name":"IV","Value":"220"},
*    {"Name":"LA","Value":"1.58"},
*    {"Name":"SV","Value":"160"}
*  ]
***************************************************************************************************/
    WidoClient.fastrprint(F("[{\"Name\":\"BV\",\"Value\":\""));
    WidoClient.fastrprint(BVChar);  //23
    
    WidoClient.fastrprint(F("\"},{\"Name\":\"OV\",\"Value\":\""));
    WidoClient.fastrprint(OVChar);  //25
    
    WidoClient.fastrprint(F("\"},{\"Name\":\"A1\",\"Value\":\""));
    WidoClient.fastrprint(BIChar);  //25
    
    WidoClient.fastrprint(F("\"},{\"Name\":\"LA\",\"Value\":\""));
    WidoClient.fastrprint(LAChar);    //25
    
    WidoClient.fastrprint(F("\"},{\"Name\":\"SV\",\"Value\":\""));
    WidoClient.fastrprint(USChar);
    WidoClient.fastrprintln(F("\"}]"));    //28
        
//    strcat(httpPackage,"{\"Name\":\"T1\",\"Value\":\"");
//    dtostrf(Temperature,5,2,httpPackage+strlen(httpPackage));
//    strcat(httpPackage,"\"},");
//    strcat(httpPackage,"{\"Name\":\"H1\",\"Value\":\"");
//    dtostrf(Humidity,5,2,httpPackage+strlen(httpPackage));          // push the data to the http data package
//    strcat(httpPackage,"\"}]");    
    Serial.println(F("Sending data"));
    Serial.println(F(" done."));
    
    /********** Get the http page feedback ***********/
    
    unsigned long rTimer = millis();
    Serial.println(F("Reading Cloud Response!!!\r\n"));
    while (millis() - rTimer < 2000) {
      while (WidoClient.connected() && WidoClient.available()) {
        char c = WidoClient.read();
        Serial.print(c);
      }
    }
    Serial.println(F(" done."));
    wdt_reset();   //看门狗复位
    delay(1000);             // Wait for 1s to finish posting the data stream

    WidoClient.close();      // Close the service connection
    digitalWrite(LED_PIN, HIGH);   // sets the LED on
    delay(200);
    digitalWrite(LED_PIN, LOW);   // sets the LED off
    delay(100);
    digitalWrite(LED_PIN, HIGH);   // sets the LED on
    delay(200);
    digitalWrite(LED_PIN, LOW);   // sets the LED off
    delay(100);
   digitalWrite(LED_PIN, HIGH);   // sets the LED on
    delay(200);
    digitalWrite(LED_PIN, LOW);   // sets the LED off
    
    RetryMillis = millis();  // Reset the timer stamp for applying the connection with the service
  }

}


