#include <stdio.h>
#include <string.h>
#include <main.h>

#include "stm32f0xx_hal_uart.h"

typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
extern I2C_HandleTypeDef I2cHandle;

int BrcmConsolePrintf(const char * fmt, ...);
void f2string(float v, char * out, int * outlen);

#define NDEF_FLAG_MB  ((uint8_t)(1 << 7))
#define NDEF_FLAG_ME  ((uint8_t)(1 << 6))
#define NDEF_FLAG_CF  ((uint8_t)(1 << 5))
#define NDEF_FLAG_SR   ((uint8_t)(1 << 4))
#define NDEF_FLAG_IL  ((uint8_t)(1 << 3))
#define NDEF_FLAG_TNF ((uint8_t)(0x01))

#define M24LR16_EEPROM_ADDRESS_USER		(0x0A6 >> 1) /* I2C DeviceSelect */
#define M24LR16_EEPROM_ADDRESS_SYSTEM		0x0AE >> 1 /* I2C DeviceSelect */

// SINGLE RECORD, SHORT TYPE
#define NDED_SR_SINGLE_FLAG ((uint8_t)(NDEF_FLAG_MB \
                      | NDEF_FLAG_ME \
                      | NDEF_FLAG_SR \
                      | NDEF_FLAG_TNF))
typedef struct {
  uint8_t flag;
  uint8_t type_length;
  uint8_t payload_length;
  //uint8_t id_length; omited
  uint8_t type;
  //uint8_t ID;   // omited
  uint8_t payload[128];
}single_sr_header_t, *single_sr_header_p;

uint8_t * get_i2c_address(uint16_t addr, uint8_t * out)
{
  out[0] = addr >> 8;
  out[1] = addr;
  return out;
}

// D1 01, 07, 54, 02, 65, 6E, 32, 32, 33, 34
// type, T
// encoding UTF-8
// lang en
// text "1234"
// payload length 7
// 0x65, 0c6e,

single_sr_header_p make_single_sr_text(char * p, int length)
{
  static uint8_t raw_data[64];
  single_sr_header_p hdr = (single_sr_header_p)raw_data;

  hdr->flag = NDED_SR_SINGLE_FLAG;
  hdr->type_length = 1;
  hdr->payload_length = length + 3; // encoding length = 3
  hdr->type = 'T';

  hdr->payload[0] = 2;
  hdr->payload[1] = 'e';
  hdr->payload[2] = 'n';

  memcpy(hdr->payload + 3, p, length);
  return hdr;
}

char * make_ndef_TLV(single_sr_header_p ndef)
{
  static char mem[32];
  int length = 0;
  int index = 0;

  length = (ndef->payload_length + 4) & 0x00ff;
  mem[0] = 3;
  mem[1] = length;
  memcpy(mem + 2, ndef, mem[1] & 0x0ff);

  index = length + 2;
  mem[index] = 0;
  mem[index + 1] = 0;
  mem[index + 2] = 0xFE;

  return mem;
}

void delay_ms(uint16_t time) 
{     

   uint16_t i=0;   

   while(time--) 
   { 
      i=12000;  
      while(i--);
   } 
}  

//uint8_t writeData[32] = {0xe1, 0x40, 0xff, 0x05, 0x03, 0x11, 0xd1, 0x01, 0x0d, 0x54, 0x02, 0x65, 0x6e, 0x31, 0x32, 'A',
//                           'B',   'X', 'X' , 'E' , 'F' , 'G' , 'H' , 0x00, 0xfe, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff};

uint8_t writeData[24] = {0xe1, 0x40, 0xff, 0x05, 0x03, 0x0b, 0xd1, 0x01, 0x07, 0x54, 0x02, 0x65, 0x6e, '3', '6', '.',
                          '7', 0x00, 0x00, 0x00 ,0xfe, 0x00 ,0x00, 0x00};
/*
 * e1, 40, ff, 05, 03, 11, d1, 01, 0d, 54, 02, 65, 6e, 31, 32, 33,
 * 34, 35, 36, 37, 38, 39, 30, 00, fe, 00, 00, 00, ff, ff, ff, ff,
*/
void write_sample_ndef(float v)
{
  int i = 0;
  uint8_t tmp[16];
  HAL_StatusTypeDef status;
	
	f2string(v, (char *)&writeData[13], 0);
	BrcmConsolePrintf("write ndef\r\n");
  for(i = 0; i < sizeof(writeData); i += 4 )
  {
    get_i2c_address(i, tmp);
    memcpy(tmp + 2, writeData + i, 4);
		BrcmConsolePrintf(".");
    status = HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)I2C_ADDRESS, tmp, 6, 1000);

    delay_ms(100);
    if(status)
		{
			BrcmConsolePrintf("write failed\r\n");
		}		
  }
	BrcmConsolePrintf("\r\n");
}

void f2string(float v, char * out, int * outlen)
{
	if(outlen)
		*outlen = 4;
	
	if(v > 45 || v < 34)
	{
		memcpy(out, "99.9", 4);
		return;
	}else{
		sprintf(out, "%.1f", v);
		return;
	}
}

void dumpHex(uint8_t * data, int length)
{
  int i = 0;

  BrcmConsolePrintf("\r\n -- dump Hex --\r\n");
  for(i = 0; i < length; i++)
  {
    if((i != 0) && (i % 16 == 0))
    {
      BrcmConsolePrintf("\r\n");
    }

    BrcmConsolePrintf("%02x ", data[i]);
  }

  BrcmConsolePrintf("\r\n");
}

uint8_t * get_content(uint16_t addr, uint8_t * data, int length)
{
  HAL_StatusTypeDef status;
  uint8_t i2c_addr[2];

  get_i2c_address(addr, i2c_addr);
  
	status = HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)I2C_ADDRESS, i2c_addr, 2, 1000);

  if(status)
      BrcmConsolePrintf("I2C read failed \r\n");

	status = HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, data, length, 10000);
	
  return data;
}

void test_read()
{
  uint8_t data[32];
	
  get_content(0, data, 4);
  dumpHex(data, 4);

  get_content(4, data, 4);
  dumpHex(data, 4);

  get_content(8, data, 4);
  dumpHex(data, 4);

  get_content(12, data, 4);
  dumpHex(data, 4);

  get_content(16, data, 4);
  dumpHex(data, 4);

  get_content(20, data, 4);
  dumpHex(data, 4);  
}

void ndef_test(void)
{
		static float v = 36.0;
		v += 0.1;
    write_sample_ndef(v);

    delay_ms(100);
    test_read();

    BrcmConsolePrintf("\r\nNDEF test finished\r\n");
}
