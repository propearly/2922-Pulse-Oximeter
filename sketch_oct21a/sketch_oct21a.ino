/*
** Pulse sensor firmware — sends DATA packets:
**   DATA,<seq>,<bpm_1dp>,<50 raw samples...>
**
** Requirements implemented:
**  - Pulse rate calculation (BPM)
**  - Data packaging: seq number, BPM (1dp), 50 raw samples
**  - Debounce count = 5
**  - LED: RED = alert (high/low), BLUE = connection status (ON/FLASHING/OFF)
**  - Switch: manual connect/disconnect (toggle)
*/

#include "switch.h"
#include "BluetoothSerial.h"

// Pins (adjust for your board)
#define LED_RED_PIN 32
#define LED_BLUE_PIN 2
#define SWITCH_PIN 33
#define SENSOR_PIN 25

// Config
#define BAUD_RATE 115200
#define TICK_20MSEC 20000  // 20 ms tick
#define SAMPLE_FREQ 50     // 50 Hz
#define DEBOUNCE_CNT 5     // required: debounce = 5
#define PACKET_SAMPLES 50  // number of samples per packet

// BPM thresholds
#define BPM_LOW 60
#define BPM_HIGH 100

// Globals
BluetoothSerial SerialBT;

unsigned long lastTickTime = 0;
uint16_t samples_small[5];      // small smoothing buffer
uint8_t sampleIndexSmall = 0;

uint16_t packetSamples[PACKET_SAMPLES];
uint8_t packetIndex = 0;

uint16_t signalMax = 0;
uint16_t signalMin = 4095;
uint16_t threshold = 0;
uint16_t signalRange = 0;

bool beatDetected = false;
unsigned long lastBeatTime = 0;
float beatsPerMinute = 0.0;

Switch switch1(1, DEBOUNCE_CNT);
//bool bluetoothConnected = false; // manual connection toggle

bool latching_1 = true; // device connected by default
unsigned long seqNumber = 0;

unsigned long lastBlueBlink = 0;
bool blueState = false;

// smoothing helper (simple moving average of 5)
#define NUM_SMOOTH 5
uint16_t smoothBuf[NUM_SMOOTH] = {0};
uint8_t smoothIdx = 0;

uint16_t getSmoothed(uint16_t newSample) {
  smoothBuf[smoothIdx] = newSample;
  smoothIdx = (smoothIdx + 1) % NUM_SMOOTH;
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SMOOTH; ++i) sum += smoothBuf[i];
  return sum / NUM_SMOOTH;
}

void setup() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);

  Serial.begin(BAUD_RATE);
  SerialBT.begin("ESP32_PulseIt");  // Bluetooth device name

  delay(500);
  Serial.println("PULSE SENSOR ADAPTIVE THRESHOLD - START");
  lastTickTime = micros();
}

void updateLEDs() {
  //Blue: if "connected" (latching_1 true) -> ON, if false -> flash
  if (latching_1) {
    digitalWrite(LED_BLUE_PIN, HIGH);
  } else {
    if (micros() - lastBlueBlink > 500000) { // 500 ms
      lastBlueBlink = micros();
      blueState = !blueState;
      digitalWrite(LED_BLUE_PIN, blueState ? HIGH : LOW);
    }
  }
  //Red: show alert if BPM out of range
  if (beatsPerMinute > 0.0 && (beatsPerMinute < BPM_LOW || beatsPerMinute > BPM_HIGH)) {
    digitalWrite(LED_RED_PIN, HIGH);
  } else {
    digitalWrite(LED_RED_PIN, LOW);
  }
}


// -----------------------------------------------------------------------------
// Send Data Packet
// -----------------------------------------------------------------------------
void sendDataPacket() {
  //if (!bluetoothConnected) return;

  // Format: DATA,<seq>,<bpm>,<sample1>...<sample50>
  SerialBT.print("DATA,");
  SerialBT.print(seqNumber++);
  SerialBT.print(",");
  SerialBT.print(beatsPerMinute, 1);
  SerialBT.print(",");
  for (int i = 0; i < PACKET_SAMPLES; ++i) {
    SerialBT.print(packetSamples[i]);
    if (i < PACKET_SAMPLES - 1) SerialBT.print(",");
  }
  SerialBT.println();
}

void loop() {
  // 20 ms tick
  if ((micros() - lastTickTime) > TICK_20MSEC) {
    lastTickTime = micros();

    // Read sensor and smooth
    uint16_t raw = analogRead(SENSOR_PIN);
    uint16_t sensor_reading = getSmoothed(raw);

    // Maintain packet buffer
    packetSamples[packetIndex] = sensor_reading;
    packetIndex = (packetIndex + 1) % PACKET_SAMPLES;

    // Track max/min and range for thresholding
    if (sensor_reading > signalMax) signalMax = sensor_reading;
    if (sensor_reading < signalMin) signalMin = sensor_reading;
    signalRange = signalMax - signalMin;

    // Adaptive threshold (simple low-pass)
    uint16_t mid = (signalMax + signalMin) / 2;
    threshold = (uint16_t)(0.9 * threshold + 0.1 * mid);

    // Beat detection: detect rising edge above threshold
    if ((sensor_reading > threshold) && !beatDetected && signalRange > 25) {
      beatDetected = true;
      unsigned long currentTime = millis();
      unsigned long interval = currentTime - lastBeatTime;
      if (interval > 500 && interval < 2000) {
        float instantBPM = 60000.0 / (float)interval;
        if (beatsPerMinute < 1.0) beatsPerMinute = instantBPM;
        else beatsPerMinute = 0.8 * beatsPerMinute + 0.2 * instantBPM;
        lastBeatTime = currentTime;
      } else if (lastBeatTime == 0) {
        lastBeatTime = currentTime;
      }
    }
    if (sensor_reading < threshold) {
      beatDetected = false;
    }

    // Reset max/min every ~2 seconds (100 * 20ms)
    static uint16_t sampleCounter = 0;
    if (++sampleCounter >= 100) {
      sampleCounter = 0;
      signalMax = 0;
      signalMin = 4095;
    }

    // Handle switch (manual connect/disconnect)
       if (switch1.update(digitalRead(SWITCH_PIN))) {
         if (switch1.state()) {
           latching_1 = !latching_1;
      
           if (latching_1) {
             // Just reconnected: notify host and send an immediate data packet
             SerialBT.println("STATUS,CONNECTED");
             sendDataPacket();            // immediate update so host refreshes BPM
             // optional: reset sendCounter so next scheduled send is aligned
             // sendCounter = 0;          // uncomment if you prefer scheduling
             Serial.println("Manual connect: Bluetooth active");
           } else {
             // Just disconnected: notify host
             SerialBT.println("STATUS,DISCONNECTED");
             Serial.println("Manual disconnect: Bluetooth off");
           }
        }
      }


    // Process incoming messages
    // handleIncomingBluetooth();

    // Update LEDs each tick
    updateLEDs();

    // Send a packet once every PACKET_SAMPLES * 20ms = 1 second
    static uint16_t sendCounter = 0;
    if (++sendCounter >= PACKET_SAMPLES) {
      sendCounter = 0;
      // If not connected (latching_1 false) still send REPORT? We'll send only if connected to save bandwidth:
      if (latching_1) {
      sendDataPacket();
      } else {
        // Optional: send heartbeat when disconnected? We'll print a short status
        Serial.println("STATUS,DISCONNECTED");
      }
    }
  } // end tick
}
