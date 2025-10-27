#include "switch.h"
#include "BluetoothSerial.h"

// -------------------- Pins --------------------
#define LED_RED_PIN    32
#define LED_BLUE_PIN   35
#define SWITCH_PIN     33
#define SENSOR_PIN     25

// -------------------- Config --------------------
#define BAUD_RATE      115200
#define TICK_20MS      20000    // 20 ms tick
#define SAMPLE_FREQ    50       // 50 Hz
#define DEBOUNCE_CNT   5
#define PACKET_SAMPLES 50

#define BPM_LOW        60
#define BPM_HIGH       100

// -------------------- Globals --------------------
BluetoothSerial SerialBT;

Switch switch1(1, DEBOUNCE_CNT);

unsigned long lastTick = 0;
unsigned long seqNumber = 0;
unsigned long lastBeatTime = 0;

bool beatDetected = false;
float beatsPerMinute = 0.0;
//bool connected = true;          // connection state
bool connected = false;

uint16_t packetSamples[PACKET_SAMPLES]; //temporary array to send via bluetooth
uint8_t packetIndex = 0;

// LED state
unsigned long lastBlueBlink = 0;
//bool blueState = false;
bool blueState = true;

// Adaptive threshold
uint16_t signalMax = 0;
uint16_t signalMin = 4095;
uint16_t threshold = 0;
uint16_t signalRange = 0;

// Smoothing (averaging)
#define NUM_SMOOTH 5
uint16_t smoothBuf[NUM_SMOOTH] = {0};
uint8_t smoothIdx = 0;

// -------------------- Helper Functions --------------------
uint16_t getSmoothed(uint16_t newSample) {
    smoothBuf[smoothIdx] = newSample;
    smoothIdx = (smoothIdx + 1) % NUM_SMOOTH;
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SMOOTH; ++i) sum += smoothBuf[i];
    return sum / NUM_SMOOTH;
}

void updateLEDs() {
    // Blue LED: connected = ON, disconnected = FLASHING
    if (connected) {
        digitalWrite(LED_BLUE_PIN, HIGH);
    } 
    else {
        // if (micros() - lastBlueBlink > 500000) { // 500ms
        //     lastBlueBlink = micros();
        //     blueState = !blueState;
        //     digitalWrite(LED_BLUE_PIN, blueState ? HIGH : LOW);
        // }
        digitalWrite(LED_BLUE_PIN, LOW);
    }

    // Red LED: BPM out of range = ON, bluetooth disconeccted
    if (beatsPerMinute > 0.0 && (beatsPerMinute < BPM_LOW || beatsPerMinute > BPM_HIGH)) {
        digitalWrite(LED_RED_PIN, HIGH);
    } else {
        digitalWrite(LED_RED_PIN, LOW);
    }
}

void sendDataPacket() { 
    if (!connected) return;

    SerialBT.print("DATA,");
    SerialBT.print(seqNumber++);
    SerialBT.print(",");
    SerialBT.print(beatsPerMinute, 1);

    for (int i = 0; i < PACKET_SAMPLES; ++i) {
        SerialBT.print(",");
        SerialBT.print(packetSamples[i]);
    }
    SerialBT.println();
}

// -------------------- Setup --------------------
void setup() {
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(SWITCH_PIN, INPUT);
    pinMode(SENSOR_PIN, INPUT);

    Serial.begin(BAUD_RATE);
    SerialBT.begin("ESP32_PulseIt");

    delay(500);
    Serial.println("Pulse Sensor - START");
    lastTick = micros();
}

// -------------------- Loop --------------------
void loop() {
    // code runs every 20 ms tick
    if ((micros() - lastTick) >= TICK_20MS) {
        lastTick = micros();

        // --- Read sensor and smooth ---
        uint16_t raw = analogRead(SENSOR_PIN);
        uint16_t sensor = getSmoothed(raw);

        // --- Print to Serial Monitor/Plotter ---
        // Format: sensor,threshold
        Serial.print(sensor);
        Serial.print(",");
        Serial.println(threshold);

        // --- Store packet samples ---
        packetSamples[packetIndex] = sensor;
        packetIndex = (packetIndex + 1) % PACKET_SAMPLES;

        // --- Update adaptive threshold ---
        if (sensor > signalMax) signalMax = sensor;
        if (sensor < signalMin) signalMin = sensor;
        signalRange = signalMax - signalMin;

        uint16_t mid = (signalMax + signalMin) / 2;
        threshold = (uint16_t)(0.9 * threshold + 0.1 * mid);

        // --- Beat detection ---
        if ((sensor > threshold) && !beatDetected && signalRange > 25) {
            beatDetected = true;
            unsigned long now = millis();
            unsigned long interval = now - lastBeatTime;
            if (interval > 500 && interval < 2000) {
                float instantBPM = 60000.0 / interval;
                if (beatsPerMinute < 1.0) beatsPerMinute = instantBPM;
                else beatsPerMinute = 0.8 * beatsPerMinute + 0.2 * instantBPM;
                lastBeatTime = now;

                // Optional: flash blue LED on heartbeat
                digitalWrite(LED_BLUE_PIN, HIGH);
                delay(20);
                digitalWrite(LED_BLUE_PIN, LOW);
            } else if (lastBeatTime == 0) {
                lastBeatTime = now;
            }
        }
        if (sensor < threshold) beatDetected = false;

        // --- Reset max/min every ~2 seconds ---
        static uint16_t sampleCounter = 0;
        if (++sampleCounter >= 100) {
            sampleCounter = 0;
            signalMax = 0;
            signalMin = 4095;
        }

        // --- Handle switch (manual connect/disconnect) ---
        if (switch1.update(digitalRead(SWITCH_PIN))) {
            if (switch1.state()) {
                connected = !connected;
            }
        }

        // --- Update LEDs ---
        updateLEDs();

        // --- Send data packet every 50 samples (~1 second) ---
        static uint16_t sendCounter = 0;
        if (++sendCounter >= PACKET_SAMPLES) {
            sendCounter = 0;
            sendDataPacket();
        }
    }
}
