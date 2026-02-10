#include <CANSAME5x.h>

CANSAME5x CAN;

unsigned long amplitudes[400]; // Array to store amplitude values for 400 points
bool amplitudeReceived[400] = {false}; // Array to track which indexes have been received
unsigned int lastReceivedIndex = 0; // Keep track of the last index to detect skipped packets
unsigned int currentSweepID = 0xFFFF; // Initial value for sweep ID

unsigned long index_errors = 0;
unsigned long id_errors = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("CAN Receiver");

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // start the CAN bus at 250 kbps
  if (!CAN.begin(500000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("Starting CAN!");
}
void loop() {
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    uint32_t packetId = CAN.packetId();  // Get CAN ID
    int data[8];
    int index = 0;
    while (CAN.available()) {
      if (index < 8) {
        data[index++] = CAN.read();
      } else {
        CAN.read(); // discard extra bytes
      }
    }

    // Branch based on CAN ID
    switch (packetId) {
      case 0x13: {  // Example: amplitude data frame
        if (index >= 8) {
          unsigned long amplitudeValue = ((unsigned long)data[0] << 24) |
                                         ((unsigned long)data[1] << 16) |
                                         ((unsigned long)data[2] << 8) |
                                         (unsigned long)data[3];

          unsigned int sweepIndex = ((unsigned int)data[4] << 8) |
                                    (unsigned int)data[5];

          if (sweepIndex == 0) {
            lastReceivedIndex = 0;
            for (int i = 0; i < 400; i++) {
              amplitudeReceived[i] = false;
            }
          }

          if (sweepIndex < 400) {
            if (sweepIndex > lastReceivedIndex + 1) {
              Serial.print("Warning: Missing packet(s) between index ");
              Serial.print(lastReceivedIndex);
              Serial.print(" and ");
              Serial.println(sweepIndex - 1);
            }

            lastReceivedIndex = sweepIndex;
            amplitudes[sweepIndex] = amplitudeValue;
            amplitudeReceived[sweepIndex] = true;

            bool sweepComplete = true;
            for (int i = 0; i < 400; i++) {
              if (!amplitudeReceived[i]) {
                sweepComplete = false;
               
                break;
              }
            }

            if (sweepComplete) {
              Serial.println("Full sweep received:");
              for (int i = 0; i < 400; i++) {
                Serial.print("Index ");
                Serial.print(i);
                Serial.print(": Amplitude ");
                Serial.println(amplitudes[i]);
                amplitudeReceived[i] = false;
              }
              Serial.println("End of sweep.");
              lastReceivedIndex = 0;
            }
          } else {
            Serial.println("Error: Index out of range.");
          }
        }
        break;
      }

      case 0x01: {
          unsigned int temp = ((unsigned long)data[4] << 8)
           | (unsigned long)data[5];

          unsigned int divisor = ((unsigned int)data[6] << 8) 
          | (unsigned int)data[7];
          Serial.print("Temp:");
          Serial.print(temp);
          Serial.print("<>");
          Serial.print("Divisor:");
          Serial.println(divisor);
        break;
      }
    }
  }
}
