#include <CANSAME5x.h>

CANSAME5x CAN;

const int SWEEP_POINTS = 400;
const int LOOPBACK_POINTS = 5;  // CONFIG_LOOPBACK_NUM_POINTS
// Removed all chunking and sweep management - stream individual points immediately

uint32_t lastAmplitude[SWEEP_POINTS];  // Store last known values to avoid sending zeros
uint16_t currentTemp = 0;
uint16_t currentDivisor = 0;

void setup() {
  Serial.begin(2000000);  
  while (!Serial) delay(10);

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false);
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);

  if (!CAN.begin(500000)) {
    while (1) delay(10);
  }

  resetLastAmplitudes();
}

void loop() {
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    uint32_t packetId = CAN.packetId();

    // Process amplitude data packets (ID 0x13) - streaming mode
    if (packetId == 0x13 && packetSize >= 6) {
      uint8_t data[8];
      int index = 0;
      while (CAN.available() && index < 8) {
        data[index++] = CAN.read();
      }

      // Fast bit operations instead of shifts
      uint32_t amplitudeValue = (uint32_t)data[0] << 24 |
                                (uint32_t)data[1] << 16 |
                                (uint32_t)data[2] << 8  |
                                (uint32_t)data[3];
      uint16_t sweepIndex = (uint16_t)data[4] << 8 | data[5];
          
      sendSinglePointFast(sweepIndex, amplitudeValue);
    }
    // Process temp/divisor metadata (ID 0x01) - amplitude mode sends this
    else if (packetId == 0x01 && packetSize >= 8) {
      uint8_t data[8];
      int index = 0;
      while (CAN.available() && index < 8) {
        data[index++] = CAN.read();
      }
      
      // Extract temp and divisor from bytes 4-7
      currentTemp = (uint16_t)data[4] << 8 | data[5];
      currentDivisor = (uint16_t)data[6] << 8 | data[7];
      
      // Send metadata packet immediately
      sendMetadataFast(currentTemp, currentDivisor);
    }
    // Process loopback sweep data (ID 0x15)
    else if (packetId == 0x15 && packetSize >= 8) {
      uint8_t data[8];
      int index = 0;
      while (CAN.available() && index < 8) {
        data[index++] = CAN.read();
      }
      
      // Extract amplitude (bytes 0-3), point index (bytes 4-5), total length (bytes 6-7)
      uint32_t amplitude = (uint32_t)data[0] << 24 |
                           (uint32_t)data[1] << 16 |
                           (uint32_t)data[2] << 8  |
                           (uint32_t)data[3];
      uint16_t pointIndex = (uint16_t)data[4] << 8 | data[5];
      uint16_t totalLength = (uint16_t)data[6] << 8 | data[7];
      
      // Send loopback point immediately
      sendLoopbackPointFast(pointIndex, amplitude, totalLength);
    }
  }
  
  // No timeout checks needed - we stream immediately
}

void resetLastAmplitudes() {
  for (int i = 0; i < SWEEP_POINTS; i++) {
    lastAmplitude[i] = 0;
  }
}

// Ultra-fast single point transmission - minimal overhead
void sendSinglePointFast(uint16_t index, uint32_t amplitude) {
  uint8_t buffer[8];  // [0xAA][index(2)][amp(4)][0xA0]
  
  buffer[0] = 0xAA;  // start marker
  buffer[1] = index & 0xFF;
  buffer[2] = (index >> 8) & 0xFF;
  buffer[3] = amplitude & 0xFF;
  buffer[4] = (amplitude >> 8) & 0xFF;
  buffer[5] = (amplitude >> 16) & 0xFF;
  buffer[6] = (amplitude >> 24) & 0xFF;
  buffer[7] = 0xA0;  // end marker
  
  Serial.write(buffer, 8);
}

// Ultra-fast metadata transmission for temp and divisor
void sendMetadataFast(uint16_t temp, uint16_t divisor) {
  uint8_t buffer[7];  // [0xBB][temp(2)][div(2)][chk][0xA0]
  
  buffer[0] = 0xBB;  // metadata marker
  buffer[1] = (temp >> 8) & 0xFF;
  buffer[2] = temp & 0xFF;
  buffer[3] = (divisor >> 8) & 0xFF;
  buffer[4] = divisor & 0xFF;
  buffer[5] = buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4];  // Simple checksum
  buffer[6] = 0xA0;  // end marker
  
  Serial.write(buffer, 7);
}

// Ultra-fast loopback point transmission
void sendLoopbackPointFast(uint16_t index, uint32_t amplitude, uint16_t totalLength) {
  uint8_t buffer[10];  // [0xCC][index(2)][amp(4)][len(2)][0xA0]
  
  buffer[0] = 0xCC;  // loopback marker
  buffer[1] = index & 0xFF;
  buffer[2] = (index >> 8) & 0xFF;
  buffer[3] = amplitude & 0xFF;
  buffer[4] = (amplitude >> 8) & 0xFF;
  buffer[5] = (amplitude >> 16) & 0xFF;
  buffer[6] = (amplitude >> 24) & 0xFF;
  buffer[7] = totalLength & 0xFF;
  buffer[8] = (totalLength >> 8) & 0xFF;
  buffer[9] = 0xA0;  // end marker
  
  Serial.write(buffer, 10);
}
