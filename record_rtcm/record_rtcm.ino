#include <M5Stack.h>
#include "esp_system.h"
//#include "esp_timer.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

#define RTCM3PREAMB 0xD3

static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

unsigned int rtk_crc24q(const unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}

enum ParseState {
  WAIT_FOR_HEADER,
  READ_LENGTH1, READ_LENGTH2, 
  READ_PAYLOAD,
  READ_CRC1, READ_CRC2, READ_CRC3
};

HardwareSerial GNSSRaw(2);
uint8_t buffer[1200];
uint16_t buf_index = 0;
uint8_t len1, len2;
uint16_t len;
uint8_t crc1, crc2, crc3;
uint32_t crc_msg, crc;
ParseState state = WAIT_FOR_HEADER;
uint32_t r;
char file_name[128];
File rtcm_file;
uint32_t frames = 0;
void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.Power.begin();
  GNSSRaw.begin(115200);
  Serial.println("record gnss rtcm data");
  M5.Lcd.setTextFont(4);
  M5.Lcd.println("rtcm recorder");
//  termInit();
  if (!SD.begin()) {  // Initialize the SD card. 初始化SD卡
    M5.Lcd.println(
        "Card failed, or not present");  // Print a message if the SD card
                                         // initialization fails or if the
                                         // SD card does not exist
                                         // 如果SD卡初始化失败或者SD卡不存在，则打印消息
    while (1)
        ;
  }
  sprintf(file_name, "/rtcm_%u.bin", esp_random() % 128);
  rtcm_file = SD.open(file_name, FILE_WRITE);
  if(rtcm_file)
  {
    M5.Lcd.println("rtcm file created. name:");
    M5.Lcd.println(file_name);
    M5.Lcd.println("file created success!");
  }
  else
  {
    M5.Lcd.println("can not create file!");
    M5.Lcd.println(file_name);
  }
}

void reset_state()
{
  state = WAIT_FOR_HEADER;
  buf_index = 0;
  len1 = 0;
  len2 = 0;
  len = 0;
  crc1 = 0;
  crc2 = 0;
  crc3 = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  while (GNSSRaw.available()) {
    uint8_t b = GNSSRaw.read();
//    termPutchar(b);
    switch(state)
    {
      case WAIT_FOR_HEADER:
        if(b == RTCM3PREAMB)
        {
          state = READ_LENGTH1;
          buffer[buf_index++] = b;
//          Serial.println("got rtcm3 header");
        }
        
      break;

      case READ_LENGTH1:
        len1 = b;
//        Serial.printf("len1: 0x%02X\n", len1);
        buffer[buf_index++] = b;
        state = READ_LENGTH2;
      break;

      case READ_LENGTH2:
        len2 = b;
//        Serial.printf("len2: 0x%02X\n", len2);
        buffer[buf_index++] = b;
        len = ((len1 & 3u) << 8) + len2 + 3;
//        Serial.printf("rtcm len: %u\n", len);
        if(len > 1200)
        {
          Serial.printf("rtcm len not right!\n");
          reset_state();
        }
        else
        {
          state = READ_PAYLOAD;
        }
      break;

      case READ_PAYLOAD:
        buffer[buf_index++] = b;
        if(buf_index >= len)
        {
          state = READ_CRC1;
        }
        break;
      break;

      case READ_CRC1:
        crc1 = b;
        buffer[buf_index++] = b;

        state = READ_CRC2;
      break;

      case READ_CRC2:
        crc2 = b;
        buffer[buf_index++] = b;

        state = READ_CRC3;
      break;

      case READ_CRC3:
        crc3 = b;
        buffer[buf_index++] = b;

        // check crc!
        crc_msg = (crc1 << 16) + (crc2 << 8) + crc3;
        crc = rtk_crc24q(buffer, len);
        if(crc != crc_msg)
        {
          Serial.printf("wrong crc! crc is 0x%08X, should be 0x%08X\n", crc, crc_msg);
        }
        Serial.printf("rtcm frame end! len: %u\n", len);
        rtcm_file.write(buffer, len + 3);
        frames++;
        Serial.printf("writing %u frame to file\n", frames);
        reset_state();
      break;
      
      default:
      break;
    }
  }
}
