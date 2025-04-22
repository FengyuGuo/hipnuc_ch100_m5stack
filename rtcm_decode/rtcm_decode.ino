#include <M5Stack.h>
#include "esp_heap_caps.h"
#include "rtklib.h"

void printHeapDetails() {
  Serial.printf("8-bit heap:     %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
  Serial.printf("32-bit heap:    %d\n", heap_caps_get_free_size(MALLOC_CAP_32BIT));
  Serial.printf("Internal heap:  %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.printf("Largest 8-bit block:  %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.printf("Largest 32-bit block: %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
}

rtcm_t* rtcm_data;

void setup() {
  M5.begin();
  size_t maxAlloc = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  Serial.printf("Max malloc-able block in DRAM: %d bytes\n", maxAlloc);
  rtcm_data = (rtcm_t*)malloc(sizeof(rtcm_t));
  if (rtcm_data == NULL)
  {
    Serial.printf("can not malloc rtcm_t\n");
  }
  M5.Power.begin();
  Serial.println("decode rtcm in m5stack");
  M5.Lcd.setTextFont(4);
  M5.Lcd.println("rtcm decode example");
  Serial.printf("size of rtcm_t: %u, max sat: %d, size of obs_t: %u, size of nav_t: %u, size of sta_t: %u, size of ssr_t: %u\n", sizeof(rtcm_t), MAXSAT, sizeof(obs_t), sizeof(nav_t), sizeof(sta_t), sizeof(ssr_t) * MAXSAT);
  if (psramFound()) {
    Serial.println("✅ PSRAM detected!");
    Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM:  %d bytes\n", ESP.getFreePsram());
  } else {
    Serial.println("❌ PSRAM NOT detected.");
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  printHeapDetails();
  delay(1000);
}
