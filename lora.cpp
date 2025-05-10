
#include <AltSoftSerial.h>
#include <string.h>
#include <stdio.h>
#include "lora.h"

#define AT_TIMEOUT_MS 2000
#define TX_TIMEOUT_MS 5000
#define AT_RETRIES 3
#define RESPONSE_MAX_LENGTH 20
#define NEWLINE '\n'

//LoRa grove E5 AT COMMANDS
#define AT "AT\r\n"
#define AT_RESPONSE "+AT: OK"
#define TEST_MODE "AT+MODE=TEST\r\n"
#define TEST_MODE_RESPONSE "+MODE: TEST" 
#define TEST_CONFIG "AT+TEST=RFCFG,868,SF7,125,8,8,14,OFF,OFF,OFF\r\n"
#define TEST_CONFIG_RESPONSE "+TEST: RFCFG"
#define TEST_TRANSMIT_START "AT+TEST=TXLRSTR"
#define TEST_TRANSMIT "AT+TEST=TXLRSTR,\"Testing, Testing ....\"\r\n"
#define TEST_TRANSMIT_RESPONSE_START "+TEST: TXLRSTR"
#define TEST_TRANSMIT_DONE "+TEST: TX DONE"
#define TEST_DELAY_TRANSMIT "AT+TEST=TXLRSTR,\"Delay test\"\r\n"
#define LOW_POWER "AT+LOWPOWER\r\n"
#define LOW_POWER_RESPONSE "+LOWPOWER: SLEEP"
#define LOW_POWER_WAKEUP_RESPONSE "+LOWPOWER: WAKEUP"

//state machine to handle the transmit sequence.
typedef enum  
{

  idle,
  atSend,
  atSendRsp,
  testModeEnable,
  testModeEnableRsp,
  testModeConfig,
  testModeConfigRsp,
  testTransmit,
  testTransmitRsp,
  initTestTransmitSuccess,
  waitTestTransmitSuccess,
  lowPowerMode,
  lowPowerModeRsp,
  cleanUp,

}txSeqStg;

//state machine to handle the comms response.
typedef enum  
{
  wait,
  done,
  timeout,
  error,

}rspStat;

txSeqStg CurrentTxStage = idle;

AltSoftSerial altSerial;

unsigned long StartTime;
unsigned long Timeout;

char RespCopy[RESPONSE_MAX_LENGTH];
int ResponseLength;
int Retries;

static char buffer[RESPONSE_MAX_LENGTH];
int BufferCount = 0;
bool NewlineRx = false;

bool DebugFlag = false;

char Packet[PACKET_MAX_LENGTH];

void prepareResponseStuff()
{
  StartTime = millis();
  BufferCount = 0;
  NewlineRx = false;
}

void atSendCommand(const char* command, const char* resp)
{

  //get string lengths
  int sendLength = 0;
  sendLength = strlen(command);
  ResponseLength = strlen(resp);
  strcpy(RespCopy, resp);

  //clear out rx the buffer.
  while(altSerial.available()) altSerial.read();
 
  char* add = command;

  for(int n = 0; n < sendLength; n++)
  {
    altSerial.print(*add);

    if (DebugFlag == true)
    { 
      Serial.print(*add);
    }
    add++;
  }

  Timeout = AT_TIMEOUT_MS;
  prepareResponseStuff();
}


rspStat atRespServ()
{
  //guard against stupid values
  if(ResponseLength == 0)
  {
    return error;
  } 

  //Every response is terminated with newline. 
  while(altSerial.available() && (NewlineRx == false))
  {
    char nextChar = altSerial.read();

    //only copy part of the response typically.
    if(BufferCount < ResponseLength)
    {
      buffer[BufferCount] = nextChar;
      BufferCount++;
    }
      
    if(nextChar == NEWLINE)
    {
      NewlineRx = true;
    }

    if(DebugFlag == true)
    {
      Serial.print(nextChar);
    }
  }
 
  if((NewlineRx == false) &&
    ((millis() - StartTime) < Timeout))
  {
    return wait;
  }

  if(NewlineRx == false)
  {
    return timeout;
  }  

  if(ResponseLength != BufferCount)
  {
    return error;
  }
 
  if(strncmp(RespCopy, buffer, ResponseLength) != 0)
  {
     return error;
  }

  return done;
}

void processCmdRsp(rspStat rsp, txSeqStg next, txSeqStg prev)
{
  if(rsp == wait)
  {
    //do nothing
  }
  else if(rsp == done)
  {
    //next stage and refresh retries.
    CurrentTxStage = next;
    Retries = AT_RETRIES;
  }
  else if (((rsp == timeout) || (rsp == error)) && (Retries > 0)) 
  {
    CurrentTxStage = prev;
    Retries--;
  }
  else
  {
    //ran out of retries.
    CurrentTxStage = cleanUp;
  }	      
}

void LoRaInit()
{
  CurrentTxStage = idle;
  altSerial.begin(9600);
}

bool LoRaService()
{
  rspStat rspStatus; 

  switch (CurrentTxStage)
  {
    case idle:
      //do nothing
    break;
    case atSend:
      
      atSendCommand(AT, AT_RESPONSE);
      CurrentTxStage = atSendRsp;

    break;
    case atSendRsp:

      rspStatus = atRespServ();
      processCmdRsp(rspStatus, testModeEnable, atSend);

    break;
    case testModeEnable:

      atSendCommand(TEST_MODE, TEST_MODE_RESPONSE);
      CurrentTxStage = testModeEnableRsp;

    break;
    case testModeEnableRsp: 

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, testModeConfig, testModeEnable);

    break;
    case testModeConfig:

      atSendCommand(TEST_CONFIG, TEST_CONFIG_RESPONSE);
      CurrentTxStage = testModeConfigRsp;
  
    break;
    case testModeConfigRsp:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, testTransmit, testModeConfig);

    break;
    case testTransmit:

      //TODO 
      //build transmit packet here
      char message [PACKET_MAX_LENGTH];
      strcpy(message, TEST_TRANSMIT_START);
      strcat(message, ",\"");
      strcat(message, Packet);
      strcat(message, "\"\r\n");
    
      atSendCommand(message, TEST_TRANSMIT_RESPONSE_START);
      CurrentTxStage = testTransmitRsp;

    break;
    case testTransmitRsp:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, initTestTransmitSuccess, testTransmit);

    break;
    case initTestTransmitSuccess:
      //initialise for next response.
      
      ResponseLength = strlen(TEST_TRANSMIT_DONE);
      strcpy(RespCopy, TEST_TRANSMIT_DONE);
      Retries = AT_RETRIES;
      Timeout = TX_TIMEOUT_MS;
      prepareResponseStuff();
      CurrentTxStage = waitTestTransmitSuccess;

    break;
    case waitTestTransmitSuccess:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, lowPowerMode, testTransmit);

    break;
    case lowPowerMode:

      atSendCommand(LOW_POWER, LOW_POWER_RESPONSE);
      CurrentTxStage = lowPowerModeRsp;

    break;
    case lowPowerModeRsp:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, cleanUp, lowPowerMode);

    break;
    case cleanUp:

      CurrentTxStage = idle;

    break;
  }

  return (CurrentTxStage != idle); 
}

void LoRaSendPacket(const char* packet, bool debugFlag)
{
  //TODO 
  //load packet and initialise state machine.
  strcpy(Packet, packet);
  DebugFlag = debugFlag;
  CurrentTxStage = atSend;
  Retries = AT_RETRIES;

}


