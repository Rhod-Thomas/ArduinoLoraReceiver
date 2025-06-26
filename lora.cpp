
#include <AltSoftSerial.h>
#include <string.h>
#include <stdio.h>
#include "lora.h"

#define AT_TIMEOUT_MS 2000
#define TX_TIMEOUT_MS 5000
#define AT_RETRIES 3
#define RESPONSE_MAX_LENGTH 20
#define NEWLINE '\n'

//inactive timeout is ninety minutes
#define INACTIVE_TIMEOUT (unsigned long)1000 * 60 * 90

//LoRa grove E5 AT COMMANDS
#define AT "AT\r\n"
#define AT_RESPONSE "+AT: OK"
#define TEST_MODE "AT+MODE=TEST\r\n"
#define TEST_MODE_RESPONSE "+MODE: TEST" 
#define TEST_CONFIG "AT+TEST=RFCFG,868,SF7,125,8,8,14,OFF,OFF,OFF\r\n"
#define TEST_CONFIG_RESPONSE "+TEST: RFCFG"
#define TEST_RX_START "AT+TEST=RXLRPKT\r\n"
#define TEST_RX_START_RESPONSE "+TEST: RXLRPKT"
#define TEST_STOP "AT+TEST=STOP\r\n"
#define TEST_STOP_RSP "+TEST: STOP"
#define RESET "AT+RESET\r\n"
#define RESET_RSP "+RESET: OK"

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
  testReceive,
  testReceiveRsp,
  testReceiveWait,
  testStop,
  testStopRsp,
  reset,
  resetRsp,
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
  CurrentTxStage = atSend;
  DebugFlag = true;
  Retries = 3;
  altSerial.begin(9600);
}

bool LoRaService()
{
  rspStat rspStatus; 

  switch (CurrentTxStage)
  {
    case idle:
      //do nothing
      //this code shoukd never be idle
      //lock into an infinite loop to cause the watchdog to trigger
      while(true);

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
      processCmdRsp(rspStatus, testReceive, testModeConfig);

    break;
    case testReceive:

      atSendCommand(TEST_RX_START, TEST_RX_START_RESPONSE);
      CurrentTxStage = testReceiveRsp;

    break;
    case testReceiveRsp:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, testReceiveWait, testReceive);

    break;
    case testReceiveWait:
    
      //TODO   
      //need a way to check that the module is still working...
      //AT command maybe?
      //AT+TEST?
      //

      //pass through incoming data to serial.
      while(altSerial.available())
      {
         char nextChar = altSerial.read();
	 Serial.print(nextChar);
	 StartTime = millis();
      }

      if((millis() - StartTime) > INACTIVE_TIMEOUT)
      {
        CurrentTxStage = testStop;
      }
      
    break; 
    case testStop:

      atSendCommand(TEST_STOP, TEST_STOP_RSP);
      CurrentTxStage = testStopRsp;

    break;
    case testStopRsp:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, reset, testStop);

    break;
    case reset:

      atSendCommand(RESET, RESET_RSP);
      CurrentTxStage = resetRsp;

    break;
    case resetRsp:

      rspStatus= atRespServ();
      processCmdRsp(rspStatus, atSend, reset);

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


