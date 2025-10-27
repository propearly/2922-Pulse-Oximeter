 url=https://github.com/propearly/2922-Pulse-Oximeter/blob/9c5f82b2c3ac4bf7962c582f469e979742a4cb14/sketch_oct21a/sketch_oct21a.ino
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

// Blue blink period (microseconds)
#define BLUE_BLINK_US 300000UL  // 300 ms

// Globals
BluetoothSerial SerialBT;

unsigned long lastTickTime = 0;
uint16_t samples_small[5];      // small smoothing buffer (unused for now)
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

bool latching_1 = true; // device allows connections by default (advertising)
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
  SerialBT.begin("ESP32_PulseIt");  // Bluetooth device name (start advertising / SPP server)

  // Initialize smoothing buffer and threshold from a real reading to avoid startup artifacts
  uint16_t v = analogRead(SENSOR_PIN);
  for (int i = 0; i < NUM_SMOOTH; ++i) smoothBuf[i] = v;
  threshold = v;

  delay(500);
  Serial.println("PULSE SENSOR ADAPTIVE THRESHOLD - START");
  lastTickTime = micros();
}

// Update LEDs:
// - Blue LED:
//    * If a client is connected => FLASH (visible activity)
//    * Else if latching_1 (device advertising/available) => steady ON
//    * Else => OFF
// - Red LED: ON when BPM is out of range (alert), else OFF
void updateLEDs() {
  bool btConnected = SerialBT.connected(); // true when a remote client is connected

  if (btConnected) {
    // Flashing when connected (activity)
    if (micros() - lastBlueBlink > BLUE_BLINK_US) {
      lastBlueBlink = micros();
      blueState = !blueState;
      digitalWrite(LED_BLUE_PIN, blueState ? HIGH : LOW);
    }
  } else {
    if (latching_1) {
      // Advertising / available: steady ON so user knows it's connectable
      digitalWrite(LED_BLUE_PIN, HIGH);
    } else {
      // Manually disabled: OFF
      digitalWrite(LED_BLUE_PIN, LOW);
    }
  }

  // Red: show alert if BPM out of range
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
  // Format: DATA,<seq>,<bpm>,<sample1>...<sample50>
  // Send samples in chronological order (oldest -> newest) even though packetSamples is circular
  SerialBT.print("DATA,");
  SerialBT.print(seqNumber++);
  SerialBT.print(",");
  SerialBT.print(beatsPerMinute, 1);
  SerialBT.print(",");
  for (int i = 0; i < PACKET_SAMPLES; ++i) {
    int idx = (packetIndex + i) % PACKET_SAMPLES; // oldest sample is at packetIndex
    SerialBT.print(packetSamples[idx]);
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

    // Maintain packet buffer (circular)
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
    // NOTE: To improve reconnection speed we keep the Bluetooth server running (SerialBT.begin called in setup)
    // and use latching_1 to allow/deny data sends. This avoids calling SerialBT.end()/begin() which takes time.
    if (switch1.update(digitalRead(SWITCH_PIN))) {
      if (switch1.state()) {
        latching_1 = !latching_1;

        if (latching_1) {
          // Reconnect: don't restart BT; keep advertising and immediately notify host
          SerialBT.println("STATUS,CONNECTED");
          sendDataPacket(); // immediate update
          Serial.println("Manual connect: Bluetooth active (advertising kept)");
        } else {
          // Disconnect: mark as disabled, notify host. We don't call SerialBT.end() to keep device discoverable
          // (faster reconnection). If you want to forcibly drop an active client and your library supports it,
          // you can call SerialBT.disconnect(); // <-- optional, library dependent
          SerialBT.println("STATUS,DISCONNECTED");
          Serial.println("Manual disconnect: Bluetooth disabled for data (advertising kept)");
        }
      }
    }

    // Update LEDs each tick
    updateLEDs();

    // Send a packet once every PACKET_SAMPLES * 20ms = 1 second
    static uint16_t sendCounter = 0;
    if (++sendCounter >= PACKET_SAMPLES) {
      sendCounter = 0;
      // Only send data if latching_1 is true (device allowing transmissions)
      if (latching_1) {
        sendDataPacket();
      } else {
        // Optional: local serial status print to help debugging
        Serial.println("STATUS,DISCONNECTED");
      }
    }
  } // end tick
}