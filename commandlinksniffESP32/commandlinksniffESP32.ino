#include "driver/twai.h"

uint32_t getPGNFromExtendedId(uint32_t canId)
{
    uint32_t pgn = (canId >> 8) & 0x3FFFF;  // R, DP, PF, PS

    // If PF < 240, zero out PS because it is a destination address
    if (((canId >> 16) & 0xFF) < 240) {
        pgn &= 0x3FF00;
    }

    return pgn;
}



static const gpio_num_t CAN_TX = GPIO_NUM_21;
static const gpio_num_t CAN_RX = GPIO_NUM_22;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  // General config
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX,
      CAN_RX,
      TWAI_MODE_LISTEN_ONLY   // use TWAI_MODE_LISTEN_ONLY if you only want to sniff
  );

  // 250 kbps timing
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

  // Accept all frames
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", err);
    while (true) delay(1000);
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", err);
    while (true) delay(1000);
  }

  Serial.println("TWAI started at 250 kbps");

}

void loop() {
  delay(50);


  // Read any received frame
  twai_message_t rx_msg;
  esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(10));

  if(err == ESP_OK){
    if(rx_msg.extd == 1){
      Serial.println("Extended msg found");
      if(getPGNFromExtendedId(rx_msg.identifier) == 0x1F200){
        Serial.println("Rapid Found");
      }
      else if(getPGNFromExtendedId(rx_msg.identifier) == 0x1F201){
        Serial.println("Dynamic Found");
      }

    }
  }else{
    Serial.println("CAN ERROR");
  }





  // if (err == ESP_OK) {
  //   Serial.print("RX ID: 0x");
  //   Serial.print(rx_msg.identifier, HEX);
  //   Serial.print(rx_msg.extd ? " EXT" : " STD");
  //   Serial.print(" DLC=");
  //   Serial.print(rx_msg.data_length_code);
  //   Serial.print(" Data:");

  //   for (int i = 0; i < rx_msg.data_length_code; i++) {
  //     Serial.print(" ");
  //     if (rx_msg.data[i] < 16) Serial.print("0");
  //     Serial.print(rx_msg.data[i], HEX);
  //   }
  //   Serial.println();
  // }



}

