#include <M5Stack.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define FRAME_HEAD1 0x5A
#define FRAME_HEAD2 0xA5

#define IMU_DATA_LENGTH 82

HardwareSerial IMURaw(2);

struct __attribute__((__packed__)) HipNucIMU
{
    uint8_t tag;
    uint8_t id;
    uint8_t rev[2];
    float pressure;
    uint32_t timestamp;
    float acc[3];
    float gyr[3];
    float mag[3];
    float eul[3];
    float quat[4];
};

static void crc16_update(uint16_t *currectCrc, const
            uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

QueueHandle_t imu_q;
void process_task(void *param);

void setup() {
    M5.begin();
    M5.Power.begin();
    IMURaw.begin(115200);
    Serial.println("imu example");
    M5.Lcd.setTextFont(4);
    M5.Lcd.println("IMU Raw Example");
    imu_q = xQueueCreate(10, sizeof(HipNucIMU));
    if (!imu_q) {
      Serial.println("Queue create failed!");
    }
    xTaskCreatePinnedToCore(
      process_task,         // 任务函数
      "print_imu",    // 任务名
      1024,              // 栈大小（word 单位 = 1024*4 = 4KB）
      NULL,              // 传参
      1,                 // 优先级
      NULL,              // 不需要句柄
      0                  // 运行在 Core 0
     );
}

enum ParseState {
  WAIT_FOR_HEADER1, WAIT_FOR_HEADER2,
  READ_LENGTH1, READ_LENGTH2, 
  READ_CHECKSUM1, READ_CHECKSUM2,
  READ_PAYLOAD
};

ParseState state = WAIT_FOR_HEADER1;
uint8_t length1 = 0, length2 = 0;
uint16_t length = 0;
uint8_t buffer[100];
uint8_t buf_index = 0;
uint8_t crc1 = 0, crc2 = 0;
uint16_t crc = 0, checksum = 0;
HipNucIMU imu_data;
char msg_str[1024];
uint32_t wrong_frames = 0, right_frames = 0;

void loop() {
  while (IMURaw.available()) {
    uint8_t b = IMURaw.read();
    switch (state) {
      case WAIT_FOR_HEADER1:
        if (b == FRAME_HEAD1) 
        {
          state = WAIT_FOR_HEADER2;
          buffer[buf_index++] = b;
        }
        break;
      
      case WAIT_FOR_HEADER2:
        if (b == FRAME_HEAD2)
        {
          state = READ_LENGTH1;
          buffer[buf_index++] = b;
//          Serial.print("got imu header\n");
        }
        else{
          state = WAIT_FOR_HEADER1;
          buf_index = 0;
        }
        break;
      case READ_LENGTH1:
        length1 = b;
        state = READ_LENGTH2;
        buffer[buf_index++] = b;
        break;
      case READ_LENGTH2:
        length2 = b;
        buffer[buf_index++] = b;
        length = (length2 << 8) + length1;
        if (length != sizeof(HipNucIMU))
        {
          state = WAIT_FOR_HEADER1;
          Serial.printf("length is not right %u\n", length);
          buf_index = 0;
        }
        else
        {
          state = READ_CHECKSUM1;
//          Serial.printf("got len %u\n", length);
        }
        break;
      
      case READ_CHECKSUM1:
        crc1 = b;
        state = READ_CHECKSUM2;
        buffer[buf_index++] = b;
        break;
      case READ_CHECKSUM2:
        crc2 = b;
        crc = (crc2 << 8) + crc1;
        // check check sum
        state = READ_PAYLOAD;
        buffer[buf_index++] = b;
//        Serial.printf("crc: 0x%04X\n", crc);
        break;
      case READ_PAYLOAD:
        buffer[buf_index++] = b;
        if(buf_index == 7)
        {
//          Serial.printf("0x%02X\n", b);
        }
        if (buf_index >= IMU_DATA_LENGTH) 
        {
          //check the checksum!
          
          checksum = 0;
//          Serial.write("got imu frame\n");
          crc16_update(&checksum, buffer, 4);
          crc16_update(&checksum, buffer + 6, sizeof(HipNucIMU));
          if(crc == checksum)
          {
//            Serial.print("crc ok\n");
            memcpy(&imu_data, &buffer[6], sizeof(HipNucIMU));
            xQueueSend(imu_q, &imu_data, 0);
//          Serial.printf("size of imu data: %d\n", sizeof(HipNucIMU));
//            Serial.printf("id, tag, timestamp, gyr_x:0x%02X, %u, %u, %f\n", imu_data.tag, imu_data.id, imu_data.timestamp, imu_data.gyr[0]);
            
            right_frames++;
          }
          else
          {
            Serial.printf("crc wrong! 0x%04X\n", checksum);
            wrong_frames++;
          }


          if((right_frames + wrong_frames) % 100 == 0)
          {
            int64_t start = esp_timer_get_time();
            Serial.printf("%u right frames got. %u wrong frames got\n", right_frames, wrong_frames);
//            M5.Lcd.fillScreen(0);
//            M5.Lcd.setCursor(0, 0);
//            sprintf(msg_str, "id: 0x%02X\ttag: %u\ntimestamp: %u\ngyr: %f, %f, %f\nacc: %f, %f, %f\nmag: %f, %f, %f\n", imu_data.tag, imu_data.id, imu_data.timestamp, 
//              imu_data.gyr[0], imu_data.gyr[1], imu_data.gyr[2],
//              imu_data.acc[0], imu_data.acc[1], imu_data.acc[2],
//              imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]);
//            M5.Lcd.print(msg_str);
            int64_t end = esp_timer_get_time();
            
            printf("viz cost: %lld us\n", end - start);
//            int core = xPortGetCoreID();
//            printf("Running on core %d\n", core);
          }
          
          state = WAIT_FOR_HEADER1;
          buf_index = 0;
          length1 = 0;
          length2 = 0;
          length = 0;
          crc1 = 0;
          crc2 = 0;
          crc = 0;

        }
        break;
      default:
        break;
    }

    // termPutchar(ch);
  }
}

void process_task(void* param)
{
  HipNucIMU imu_data;
  static int imu_cnt = 0;
  while (1) {
    if(pdTRUE == xQueueReceive(imu_q, &imu_data, 0))
    {
      imu_cnt++;
      if(imu_cnt % 100 == 0)
      {
        printf("got imu %d, timestamp: %u\n", imu_cnt, imu_data.timestamp);
      }
      
    }
    vTaskDelay(1);
  }
}
