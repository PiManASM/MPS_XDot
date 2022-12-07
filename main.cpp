/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/********************************************************************************************************==*
*                                      UART test client for NNTS
* Filename      : uartTest.c
* Version       : V1.2
* Programmers(s): Hank Yung, Zachary Rodenbucher
**********************************************************************************************************
* Notes         : See ...
*/

#define __MAIN_C

/* Includes ---------------------------------------------------------------------------------------------*/

#include "mbed-os\mbed.h"
#include "checksum.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* Defines ----------------------------------------------------------------------------------------------*/
#define DOBLUE      "\033[0;34;2m"
#define DORED       "\033[0;31;2m"
#define DONONE      "\033[0m"
/*
 * Conversion macros for switching between Little and Big Endian.
*/
#define FLAMMABLE
#define SWAP16(num)        (((num & 0xff00) >> 8) | (num << 8))
#define SWAP32(num)        (((num & 0xff000000) >> 24) | ((num & 0x00ff0000) >> 8) | ((num & 0x0000ff00) << 8) | (num << 24))

/* Command Status */
#define UART_SUCCESS           0x00
#define UART_CRC_ERROR         0x01
#define UART_BAD_PARAM         0x02
#define UART_EXE_FAILED        0x03
#define UART_NO_MEM            0x04
#define UART_UNKNOWN_CMD       0x05

#define UART_LOCAL_ERROR       0xFF   /* Error generated locally - not from sensor */

/* commands */
#define CMD_ANSWER       0x01
#define CMD_ENGDATA      0x09
#ifdef FLAMMABLE
#define CMD_CONC         0x03
#define CMD_ID           0x04
#endif

#define CMD_TEMP         0x21
#define CMD_PRES         0x22
#define CMD_REL_HUM      0x23
#define CMD_ABS_HUM      0x24

#define CMD_STATUS       0x41
#define CMD_VERSION      0x42
#define CMD_SENSOR_INFO  0x43

#define CMD_MEAS               0x61
#define CMD_SHUTDOWN           0x62

#define RQST_HDR_LENGTH     sizeof(uartRqstHeader_t)
#define REPLY_HDR_LENGTH    sizeof(uartReplyHeader_t)
#define NUM_OF_CMDS         (sizeof(uart_cmds) / sizeof(uart_cmd_t))
#define UART_MAX_DATA_SIZE  (1024*8)    /* maximum packet:  header + payload */
#define ENGDATA_CHUNKSIZE   512         /* size of each chunk of engineering data */
#define FINAL_PACKET        0x8000      /* bit to indicate last chunk of engineering data */

#define GAS_NAME_LENGTH     64

/* Structure definitions --------------------------------------------------------------------------------*/
typedef struct {
  uint16_t cmdID;
  uint16_t length;
  uint16_t reserved;
  uint16_t cksum;
} uartRqstHeader_t;

typedef struct {
  uint8_t cmdID;
  uint8_t status;
  uint16_t length;
  uint16_t cksum;
} uartReplyHeader_t;

typedef struct {
  uint8_t cmdID;
  uint16_t req_size;   /* Request size */
  uint16_t res_size;   /* Response size */
  uint32_t (*func)(uint8_t cmdID, uint8_t *data, uint16_t size);
} uart_cmd_t;

typedef struct {
  uint8_t sw_w;
  uint8_t sw_x;
  uint8_t sw_y;
  uint8_t sw_z;
  uint8_t hw_w;
  uint8_t hw_x;
  uint8_t proto_w;
  uint8_t proto_x;
} uart_version_t;

typedef struct {
  uint8_t sensorName[32];  /* Serial name (zero-padded ASCII string) */
  uint32_t sensorType;   /* Sensor Type/Model */
  uint8_t calDate[16];   /* Calibration date */
  uint8_t mfgDate[16];   /* Manufacturing  date */
} uart_sensor_info_t;

#ifdef FLAMMABLE
typedef struct {
  uint32_t cycleCount;
  float concentration;
  uint32_t flamID;
  float temp;
  float pressure;
  float relHumidity;
  float absHumidity;
} answer_t;

typedef struct {
  uint32_t length;
  uint8_t data[ENGDATA_CHUNKSIZE];
} uart_engdata_t;
#else
#error Need to define expected answer type!
#endif

typedef struct {
  float temp;
  float pressure;
  float humidity;
  float absHumidity;
  float humidAirDensity;
} enviro_reply_t;

/* Functions --------------------------------------------------------------------------------------------*/
static uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint8_t uartReSend(uint8_t cmdID);
static uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadSensorInfo(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size);
static void DumpRqstHdr(uartRqstHeader_t *);
static void DumpReplyHdr(uartReplyHeader_t *);
static void DumpHexa(uint8_t *p, uint32_t len);

/* Variables --------------------------------------------------------------------------------------------*/
int uartFP;
uint32_t verbose = 0, hexdump = 0;
uint32_t numOfRetries = 0;
uint32_t rxTimeout = 0, rxBytes = 0, uartState = 0;
static uartRqstHeader_t pktHdrCache;
static uint8_t payloadCache[256];
static uint32_t payloadCacheLen = 0;
char *filename = NULL;
uart_cmd_t uart_cmds[] = {
  {CMD_ANSWER, 0, sizeof(answer_t), ReadAnswer},
  {CMD_MEAS, 1, 0, WriteByte},
#ifdef FLAMMABLE
  {CMD_CONC, 0, 4, ReadFloat},
  {CMD_ID, 0, 4, ReadInteger},
#endif
  {CMD_ENGDATA, 0, sizeof(uart_engdata_t), ReadEngData},
  {CMD_TEMP, 0, 4, ReadFloat},
  {CMD_PRES, 0, 4, ReadFloat},
  {CMD_REL_HUM, 0, 4, ReadFloat},
  {CMD_ABS_HUM, 0, 4, ReadFloat},
  {CMD_STATUS, 0, 1, ReadByte},
  {CMD_VERSION, 0, 8, ReadVersion},
  {CMD_SENSOR_INFO, 0, sizeof(uart_sensor_info_t), ReadSensorInfo},
  {CMD_SHUTDOWN, 0, 0, WriteByte}
};


//*************************************************//
static BufferedSerial pc(USBTX, USBRX, 9600);
static BufferedSerial UART1(UART1_TX, UART1_RX, 38400);

DigitalOut led(LED1);
#define BLINKING_RATE     500ms
volatile int mutex;

static uint16_t crc_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue ) {
  uint16_t crc;
  uint8_t *p;
  int ii;
  crc = startValue;

  for(p = buffer, ii = 0; ii < length; ii++) {
    crc = (crc << 8) ^ crc_table[(crc >> 8) ^ *p];
    p++;
  }
  return crc;
}

static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  mutex = 1;
  uartRqstHeader_t header;
  uint16_t cksum;

  memset(&header, 0, RQST_HDR_LENGTH);
  header.cmdID = cmdID;
  header.length = payloadLen;

  cksum = crc_generate((uint8_t *) &header, RQST_HDR_LENGTH, 0xFFFF);
  header.cksum = cksum;

  if(payloadLen != 0) {
    if(payload == NULL) {
      printf("No payload given but payload lengh is non-zero\n");
      return 1;
    }
    cksum = crc_generate(payload, payloadLen, cksum);
  }
  header.cksum = cksum;

  if(verbose) {
    DumpRqstHdr(&header);
    if(hexdump)
      DumpHexa((uint8_t *) &header, RQST_HDR_LENGTH);
  }
  
  if(UART1.write((uint8_t *) &header, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to send header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }
  
  if(numOfRetries != 0) {
    pktHdrCache = header;
  }

  if(payloadLen) {
    if(hexdump) {
      printf("  Payload");
      DumpHexa(payload, payloadLen);
    }

    if(UART1.write(payload, payloadLen) != payloadLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }

    if(numOfRetries != 0) {
      memcpy(payloadCache, payload, payloadLen);
      payloadCacheLen = payloadLen;
    }
  }
  mutex = 0;
  return 0;
}

uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  
  int32_t retry = 1;
  uint32_t status;

  status = uartSingleRecv(cmdID, payload, payloadLen);
  if((status == UART_SUCCESS) || (numOfRetries == 0))
    return status;

  do {
    if((status = uartReSend(cmdID)) != 0) {
      break;
    }

    status = uartSingleRecv(cmdID, payload, payloadLen);
  } while ((retry++ < numOfRetries) && (status != UART_SUCCESS));

  return status;
}

static uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uint16_t rxCksum, cksum;
  int rxLen;
  uint32_t timeout;
  uartRqstHeader_t header;
  uartReplyHeader_t *reply;
  uint8_t buffer[UART_MAX_DATA_SIZE+1];

  memset(buffer, 0, sizeof(buffer));

  rxLen = UART1.read(buffer, sizeof(uartReplyHeader_t));
  if(rxLen <= 0) {
    printf("Failed to get reply: %s (%d)\n", strerror(errno),  errno);
    return UART_LOCAL_ERROR;
  }

  reply = (uartReplyHeader_t *) buffer;
  if(rxLen < REPLY_HDR_LENGTH) {
    printf("Incomplete header received: %d bytes\n", rxLen);
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  if(reply->length != 0) {  /* Is there a payload for this reply? */
    rxLen = UART1.read(&buffer[REPLY_HDR_LENGTH], reply->length);
    if(rxLen < reply->length) {
      printf("Failed to get reply payload: %s (%d)\n", strerror(errno),  errno);
      return UART_LOCAL_ERROR;
    }
  }

  rxCksum = reply->cksum;
  reply->cksum = 0;  /* zero out checksum field */
  cksum = crc_generate(buffer, REPLY_HDR_LENGTH + reply->length, 0xFFFF);
  if(rxCksum != cksum) {
    printf("Checksum failed: expected 0x%x, received 0x%x\n", cksum, rxCksum);
    reply->cksum = rxCksum;   /* restore received checksum */
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  reply->cksum = rxCksum;   /* restore received checksum */

  if(reply->status != UART_SUCCESS) {
    if(reply->status >= 0x20) {
      printf("Sensor hardware error: 0x%x\n", reply->status);
    } else {
      printf("Command returned error status: 0x%x\n", reply->status);
      DumpReplyHdr(reply);
      return (reply->status);  /* Sensor sent communication error */
    }
  }
  
  if(reply->cmdID != cmdID) {
    printf("cmdID mismatch: expected 0x%x, received 0x%x\n", cmdID, reply->cmdID);
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  if (reply->length == 0)
    return UART_SUCCESS;  /* No payload, we are done. */

  if(payloadLen < reply->length) {
    printf("Buffer too small for payload (%d < %d)\n", payloadLen, reply->length);
    return UART_LOCAL_ERROR;
  }

  memset(payload, 0, payloadLen);
  memcpy(payload, &buffer[REPLY_HDR_LENGTH], reply->length);
  return UART_SUCCESS;
}

static uint8_t uartReSend(uint8_t cmdID) {
  uartReplyHeader_t reply;
  uint16_t cksum, rxCksum, length;

  if(UART1.write((uint8_t *) &pktHdrCache, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to ff header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }

  if(payloadCacheLen) {
    if(UART1.write(payloadCache, payloadCacheLen) != payloadCacheLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }
  }

  return 0;
}

static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  float *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (float *) data;
  printf("Command[0x%02x]: %f\n", cmdID, *value);

  return 0;
}

static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (uint32_t *) data;
  printf("Command[0x%02x]: %lu\n", cmdID, *value);

  return 0;
}

static uint32_t ReadSensorInfo(uint8_t cmdID, uint8_t *data, uint16_t size) {
  ///uart_sensor_info_t *sensor;

  //if(uartSend(cmdID, NULL, 0) != 0)
  //  return 1; 

  //if(uartRecv(cmdID, data, size) != 0)
  ///  return 1;

  //sensor = (uart_sensor_info_t *) data;
  //printf("Sensor Name: %s\nSensor Type: %d\nCalibration Date: %s\nManufactured Date: %s\n",
  //       sensor->sensorName, sensor->sensorType, sensor->calDate, sensor->mfgDate);

  return 0;
}

static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;
  mutex = 1;
  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  //if(uartRecv(cmdID, data, size) != 0)
  //  return 1;

  uint16_t rxCksum, cksum;
  int rxLen;
  uint32_t timeout;
  uartRqstHeader_t header;
  uartReplyHeader_t *reply;
  uint8_t buffer[UART_MAX_DATA_SIZE+1];

  memset(buffer, 0, sizeof(buffer));
  
  ///rxLen = UART1.read(buffer, sizeof(uartReplyHeader_t));
  printf("y no read");
  ///if(rxLen <= 0) {
  ///  printf("Failed to get reply: %s (%d)\n", strerror(errno),  errno);
  ///  return UART_LOCAL_ERROR;
  //}
 
  //reply = (uartReplyHeader_t *) buffer;
  
  version = (uart_version_t *) data;
  printf("SW Version: %u.%u.%u.%u, HW Version: %u.%u, Protocol: %u.%u\n",
         version->sw_w, version->sw_x, version->sw_y, version->sw_z,
         version->hw_w, version->hw_x, version->proto_w, version->proto_x);
  return 0;
}

static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("%s\n", data);
  return 0;
}



static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size) {

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("Command[0x%02x]: 0x%x\n", cmdID, *data);

  return 0;
}

static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size) {
  if(uartSend(cmdID, data, size) != 0)
    return 1;
  
  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static uint32_t WriteFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t val;
  float fval;

  val = *((uint32_t *) data);
  fval = ((float) val) / 100.0;

  printf("%s: %d %f\n", __FUNCTION__, val, fval);
  if(uartSend(cmdID, (uint8_t *) &fval, sizeof(fval)) != 0)
    return 1;

  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static void DumpRqstHdr(uartRqstHeader_t *rqst) {
  printf("----\nREQUEST:\n");
  printf("  Hdr Size: %iu\n", sizeof(uartRqstHeader_t));
  printf("  CmdID: 0x%x\n", rqst->cmdID);
  printf("  Length: %d\n", rqst->length);
  printf("  Reserved: 0x%x\n", rqst->reserved);
  printf("  Checksum: 0x%x\n", rqst->cksum);
}

static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size) {
  return 0;
}

static void DumpReplyHdr(uartReplyHeader_t *reply) {
  printf("----\nREPLY:\n");
  printf("  CmdID: 0x%x\n", reply->cmdID);
  printf("  Status: 0x%x\n", reply->status);
  printf("  Length: %d\n", reply->length);
  printf("  Checksum: 0x%x\n", reply->cksum);
}

static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size) {
  answer_t *answer;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  answer = (answer_t *) data;
#ifdef FLAMMABLE
  printf("Cycle: %u\nGas: %d\nConcentration: %f\nTEMP: %f\nPRESS: %f\nREL_HUM: %f\nABS_HUM: %f\n",
         answer->cycleCount, answer->flamID, answer->concentration, answer->temp, answer->pressure, answer->relHumidity, answer->absHumidity);
#endif
  return 0;
}

static void DumpHexa(uint8_t  *p, uint32_t len) {
  int ii;

  for(ii = 0; ii < len; ii++) {
    if((ii % 8) == 0)
      printf("\n    [%02d]: ", ii);

    printf("0x%02x ", *p++);
  }
  printf("\n");
}

int main()
{

    led = true;
    uint8_t cmdID = CMD_VERSION;
    uint8_t *payload = 0x00;
    uint16_t payloadLen = 0x008;
    int status = 0;
    if(1==0){
      printf(
        "Mbed OS version %d.%d.%d\n",
        MBED_MAJOR_VERSION,
        MBED_MINOR_VERSION,
        MBED_PATCH_VERSION
      );
    }
    printf("\n Read version %i... \n\n", RQST_HDR_LENGTH);

    status = ReadVersion(cmdID, payload, payloadLen);
    printf("\n Status: %i \n", status);

    return status;
}

/*
int main()
{
      uint32_t oper = 0, value = 0, sts;
      uint8_t cmdID;
      cmdID = CMD_VERSION;
      
  uint8_t reply[UART_MAX_DATA_SIZE];
    
    crc = crc_generate(buffer, RQST_HDR_LENGTH, 0xFFFF);
    printf("%x\n", crc);
    


    buffer[7] = (uint8_t)(crc & 0xff);
    buffer[6] = (uint8_t)(crc >> 8);

    while (true) {
        printf("hello work");
        //sts = uart_cmds[10].func(cmdID, reply, uart_cmds[10].res_size);
        printf("CRC %x\n", crc); 
        printf("crc: Buf[6] %x\n", buffer[6]);
        printf("Buf[7] %x\n", buffer[7]);
        led = !led;
        //printf("blink LED...\n");
        serial_port.write(&buffer, 8);
        printf("write buffer...\n");
        ThisThread::sleep_for(5ms);
        UART1.write(&buffer, 8);

        printf("cmd: %hhu \n", buffer[1]);
        ThisThread::sleep_for(BLINKING_RATE);
    }
}

int main() {
  DigitalOut led(LED1);
  led = true;
  
  usage();
  
  
  uint8_t cmdID;
  int c, ii;
  int portNumber = 0;
  uint32_t oper = 0, value = 0, sts;
  uint8_t reply[UART_MAX_DATA_SIZE];

  cmdID = CMD_VERSION;
  hexdump = 1;
  verbose = 1;
  //value = strtol(optarg, NULL, 16);
 

  for(ii = 0; ii < NUM_OF_CMDS; ii++) {
    if(uart_cmds[ii].cmdID != cmdID)
      continue;

    if(uart_cmds[ii].req_size) {
      sts = uart_cmds[ii].func(cmdID, (uint8_t *) &value, uart_cmds[ii].req_size);
    } else if(uart_cmds[ii].res_size) {
      sts = uart_cmds[ii].func(cmdID, reply, uart_cmds[ii].res_size);
    } else {
      sts = uart_cmds[ii].func(cmdID, NULL, 0);
    }
    break;
  }

  if(ii == NUM_OF_CMDS) {
    printf("No such command: 0x%x\n", cmdID);
    sts = 1;
  }

  exit(sts);
}
*/

