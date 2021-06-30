#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
#include <SoftwareSerial.h>
#include <Wire.h>
#include <WireSlave.h>

// #define debug

uint16_t pwr = 0;
uint16_t eTime= 0; 
uint16_t start = 0;
uint16_t tagFoundCount=0;
bool rfidStartup = true;
#define Led_esp 2 // GPIO 2 ESP32


uint8_t RFIDEN = 4;
String tagInfo="";
uint8_t M=0;
char allTags[89]={0};
unsigned long previousMillis;
unsigned long currentMillis;
typedef struct message // définir une structure message
{
int winnerRSSI[10]={-255,-255,-255,-255,-255,-255,-255,-255,-255,-255}; // initialisation des valeurs winners RSSI 
byte tagEPC[10][12]={{0x00},{0x00}};
}message;
message myData; // créer une structure message nommé myData
volatile bool rfidBusy=false;
volatile unsigned long sentStartTime;
bool receptionFlag = false;
bool requestFlag = false;
RFID nano; //Create instance
byte zeroEPC[12]={0x00};
byte mmyEPC[12]; //Most EPCs are 12 bytes
boolean ledState = false;

boolean  setupNano(long);
bool stop=false;
uint16_t i=0;
byte a=0;
String masterData = "";
uint8_t setupNanoCounter=0;

void sendRequest();
void readRequest();
void restartRFID();
bool setupNano();
bool nanoGetVersion();
bool nanoSetTagProtocol();
bool nanoSetAntennaPort();
bool nanoSetRegion(uint8_t nanoRegion);
bool nanoSetReadPower(uint16_t nanoPower);
bool nanoStopReading();
bool parseResponse(uint8_t ID, uint8_t msg);
void rfidON();
void rfidOFF();
void rfidRestart();
void blinkLed(uint16_t time_Out,uint16_t ms);

void setup()
{
  pinMode(RFIDEN,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  delay(200);
  digitalWrite(2,LOW);
  Serial.begin(9600);
  nano.disableDebugging();
  rfidON();
  if(!setupNano()){
    blinkLed(1000,25);
    esp_restart();
  }
}

void loop(){
  while(!Serial.available());
  masterData="";
  Serial.setTimeout(100);masterData=Serial.readStringUntil(';');//continuously read incoming data
  if (masterData.substring(0,1)=="R"){                           //we have an rfid read request
    digitalWrite(2,HIGH);
    delay(500);
    digitalWrite(2,LOW);
    M=0;tagFoundCount=0;
    uint8_t ind1 = masterData.indexOf(',');
    uint8_t ind2 = masterData.indexOf(',',ind1+1);
    start = masterData.substring(2,ind1).toInt();
    eTime= masterData.substring(ind1+1,ind2).toInt();
    pwr = masterData.substring(ind2+1,masterData.length()).toInt();
    nano.setReadPower(pwr);
    delay(10);
    nano.startReading();                                        //Begin scanning for tags
    previousMillis=millis();
    while((millis()-previousMillis)<eTime){
      if (nano.check() == true) //Check to see if any new data has come in from module
      { 
        byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp
        switch (responseType)
        {
        case RESPONSE_IS_KEEPALIVE:
          #ifdef debug
          Serial.println(F("Scanning"));
          #endif
          break;
        
        case RESPONSE_IS_TAGFOUND:
        {
          #ifdef debug
          Serial.print("TAG is found # ");Serial.println(tagFoundCount++);
          #endif
          int rssi = nano.getTagRSSI(); 
          byte tagEPCBytes = nano.getTagEPCBytes();
          for (byte x = 0 ; x < tagEPCBytes ; x++){
            if (nano.msg[31 + x] < 0x10) {
              mmyEPC[x]=0;
            }
            mmyEPC[x]=nano.msg[31+x];
          }
            for(byte j=0;j<10;j++){
              if (memcmp(mmyEPC,myData.tagEPC[j],12)==0){
                if (rssi>myData.winnerRSSI[j]){
                  myData.winnerRSSI[j]=rssi;
                }
                break;
              }
              else{
                if(memcmp(myData.tagEPC[j],zeroEPC,12)==0){
                  for(byte jj=0;jj<12;jj++){
                    myData.tagEPC[j][jj]=mmyEPC[jj];
                    myData.winnerRSSI[j]=rssi;
                  }
                break;
                }
              }
            }
        }
          break;
        
        case ERROR_CORRUPT_RESPONSE:
          #ifdef debug
          Serial.println("Bad CRC");
          #endif
          break;
        
        default:
          #ifdef debug
          Serial.print("Unknown error");
          #endif
          break;
        }
      }
    }
    nano.stopReading();
    delay(100);
  }
  if (masterData.substring(0,1)=="S"){sendRequest();}
}
void sendRequest(){
  while ((M<10))
  {
    tagInfo="";
    for (uint8_t i = 0; i < 12; i++){
      if (myData.tagEPC[M][i]==0){
        tagInfo+="00";
      }else if(myData.tagEPC[M][i]<=15){
        tagInfo+="0";tagInfo+=String(myData.tagEPC[M][i],HEX);
      }else tagInfo+=String(myData.tagEPC[M][i],HEX);
    }
    if (tagInfo!="000000000000000000000000")
    {
      tagInfo+=",";
      if (myData.winnerRSSI[M]>-10){
        tagInfo+="-";tagInfo+="00";tagInfo+=abs(myData.winnerRSSI[M]);
      }else if ((myData.winnerRSSI[M]<-10)&&(myData.winnerRSSI[M]>-100)){
        tagInfo+="-";tagInfo+="0";tagInfo+=abs(myData.winnerRSSI[M]);
      }else if (myData.winnerRSSI[M]<-100){
        tagInfo+=String(myData.winnerRSSI[M]);
        myData.winnerRSSI[M]=0;
      }
      tagInfo+="*";
      if (M==9){tagInfo+="&";}
      Serial.print(tagInfo);
      
      delay(5);
    }else
    {
      tagInfo="#";
      if (M==9){tagInfo+="&";}
      Serial.print(tagInfo);
      delay(5);
    }
    M++;
  }
    for (uint8_t j = 0; j < 12; j++){
      for (uint8_t i = 0; i < M; i++)
      {
        myData.tagEPC[i][j]={0x00};
      }
    }
}
void readRequest() {
    M=0;tagFoundCount=0;
    uint8_t ind1 = masterData.indexOf(',');
    uint8_t ind2 = masterData.indexOf(',',ind1+1);
    start = masterData.substring(2,ind1).toInt();
    eTime= masterData.substring(ind1+1,ind2).toInt();
    pwr = masterData.substring(ind2+1,masterData.length()).toInt();
}
void restartRFID(){
  rfidBusy=true;
  #ifdef debug
  Serial.println("restarting the RFID module");
  #endif
  pinMode(RFIDEN,OUTPUT);
  digitalWrite(RFIDEN,LOW);
  delay(1000);
  digitalWrite(RFIDEN,HIGH);
  rfidBusy=false;
}
bool setupNano(){
    nano.msg[0]=1;
    nano.begin(Serial2);
    Serial2.begin(115200);
    while(!Serial2);
    while (Serial2.available()) Serial2.read();
    delay(250);
  if (nanoSetTagProtocol()){//1
    if (nanoSetAntennaPort()){//2
      if (nanoSetRegion(REGION_EUROPE)){//3
        if (nanoSetReadPower(pwr)){//4
          return true;
        }else {return false;}
      }else {return false;}
    }else {return false;}
  }else {return false;}
}
bool nanoGetVersion(){
  nano.getVersion();
  if (parseResponse(0, nano.msg[0])){return true;i=3;}else {return false;}
}
bool nanoSetTagProtocol(){
  for (uint8_t i = 0; i < 3; i++){
    nano.setTagProtocol();
    if (parseResponse(1, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetAntennaPort(){
  for(uint8_t i = 0; i < 3; i++){
    nano.setAntennaPort();
        if (parseResponse(2, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetRegion(uint8_t nanoRegion){
  for(uint8_t i = 0; i < 3; i++){
    nano.setRegion(nanoRegion);
        if (parseResponse(3, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoSetReadPower(uint16_t nanoPower){
  for(uint8_t i = 0; i < 3; i++){
    nano.setReadPower(nanoPower);
        if (parseResponse(4, nano.msg[0])){return true;i=3;break;}else {return false;}
  }
}
bool nanoStopReading(){
  for(uint8_t i = 0; i < 3; i++){
    nano.stopReading();
    if (nano.msg[0]==ALL_GOOD){
      #ifdef debug
      Serial.println(F("nano.msg[0].stopReading = ALL GOOD"));
      #endif
      return true;
    }else{i=3;
      return false;
      break;
    }    
  }
}
bool parseResponse(uint8_t ID, uint8_t msg){
  #ifdef debug
  Serial.print(" nano.msg[0].");Serial.print(ID);Serial.print(" = ");Serial.println(nano.msg[0],HEX);
  #endif
  switch (msg)
  {
    case ALL_GOOD:
    #ifdef DEBUG
      Serial.print(ID);Serial.println(": OK");
    #endif
      rfidBusy=false;
      // nanoStartupOK=true;
      nano.msg[0]=1;
      return true;
      break;
    case ERROR_WRONG_OPCODE_RESPONSE:
      #ifdef debug
      Serial.print(ID);Serial.println(F("Continuous reading. Stopping..."));
      #endif
      nanoStopReading();
      delay(1500);
      break;
    case ERROR_COMMAND_RESPONSE_TIMEOUT:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _COMMAND_RESPONSE_TIMEOUT"));
      #endif
      restartRFID();
      return false;
      break;
    case ERROR_CORRUPT_RESPONSE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _CORRUPT_RESPONSE"));
      #endif
      delay(250);
      return false;
      break;
    case ERROR_UNKNOWN_OPCODE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _UNKNOWN_OPCODE"));
      #endif
      return false;
      break;
    case RESPONSE_IS_TEMPERATURE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_TEMPERATURE"));
      #endif
      return false;
      nanoStopReading();
      break;
    case RESPONSE_IS_KEEPALIVE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_KEEPALIVE"));
      #endif
      return false;
      break;
    case RESPONSE_IS_TEMPTHROTTLE:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_TEMPTHROTTLE"));
      #endif
      return false;
      nanoStopReading();
      break;
    case RESPONSE_IS_TAGFOUND:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_TAGFOUND"));
      #endif
      return false;
      break;
    case RESPONSE_IS_NOTAGFOUND:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_NOTAGFOUND"));
      #endif
      break;
    case RESPONSE_IS_UNKNOWN:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _IS_UNKNOWN"));
      #endif
      return false;
      restartRFID();
      break;
    case RESPONSE_SUCCESS:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _SUCCESS"));
      #endif
      return false;
      break;
    case RESPONSE_FAIL:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": _FAIL"));
      #endif
      restartRFID();
      return false;
      break;
    default:
      #ifdef debug
      Serial.print(ID);Serial.println(F(": ???"));
      #endif
      restartRFID();
      return false;
      break;
  }
}
void rfidON(){
  pinMode(RFIDEN,OUTPUT);
  digitalWrite(RFIDEN,HIGH);
  delay(40);
}
void rfidOFF(){
  // pinMode(RFIDEN,OUTPUT);
  // digitalWrite(RFIDEN,LOW);
  // delay(40);
  // pinMode(RFIDEN,OUTPUT);
  // digitalWrite(RFIDEN,LOW);
  // pinMode(RFIDEN,INPUT_PULLDOWN);
  // digitalWrite(RFIDEN,LOW);
  pinMode(RFIDEN,OUTPUT);
}
void rfidRestart(){
  rfidOFF();
  rfidON();
}

void blinkLed(uint16_t time_Out,uint16_t ms){
  previousMillis=millis();
  while((millis()-previousMillis)<time_Out){
    ledState = !ledState;
    digitalWrite(Led_esp,ledState);
    delay(ms);
  }
}