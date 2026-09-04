// Drop-in Simon XT full-field, RAM-only state-machine build.
//
// Contact state comes from the observed debounced-level field, never from
// the rolling status prefix. Periodic supervisor packets may advance the
// trigger counter, but cannot overwrite an initialized OPEN/CLOSED state.
// ============================================================
// BLOCK 1: LIBRARIES, CONFIGURATIONS, AND STRUCTURES
// ============================================================
#include <Arduino.h>
#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <esp_wifi.h>
#include <time.h>

const char *ssid = "wifi ssid here";
const char *password = "wifi password";
const char *syslogServer = "syslog ip address";
const int syslogPort = 514;

#define CC1101_SCK 1
#define CC1101_MISO 2
#define CC1101_MOSI 39
#define CC1101_CS 5
#define CC1101_GDO0 6
#define INTERNAL_LED_PIN 35

#define BUFFER_SIZE 500
#define MAX_BURSTS 15
#define BASE_UNIT_US 125
#define BURST_GAP_US 45000
#define SESSION_END_GAP_US 200000
#define MIN_TRANSITIONS 100

// Set to 0 after field validation to reduce serial output.
#define RF_DEBUG 0

// Set to 1 only when supervisor-health messages are needed in syslog.
// Supervisor packets are still decoded when this is 0; only their logs
// are suppressed.
#define LOG_SUPERVISORS 0
#define SUPERVISOR_LOG_INTERVAL_MS 86400000UL // 24 hours

struct KnownSensor
{
    uint32_t id;
    const char *name;
};

const KnownSensor SENSOR_LIST_RAW[] = {
    {id here, "Rear Door Contact"},
    {id here, "Front Door Contact"},
    {id here, "Main Garage Door"},
    {id here, "Side Garage Door"},
    {id here, "Attic Heat Sensor 1"}, //future use code will be needed for battery check in as this only changes state in a fire this is not for premises monitoring
    {id here, "Attic Heat Sensor 2"}, //future use sensors like this are only for immediate alarm siren and not for premises monitoring outside of battery low and the panel does that 
    {id here, "Living Room Motion"},
    {id here, "Office Motion"}};
const KnownSensor *SENSOR_LIST = SENSOR_LIST_RAW;
#define SENSOR_COUNT (sizeof(SENSOR_LIST_RAW) / sizeof(KnownSensor))

struct RFBurst
{
    uint32_t duration[BUFFER_SIZE];
    uint8_t level[BUFFER_SIZE];
    uint16_t count;
    uint32_t startTime;
    uint32_t endTime;
};

struct LiveTracker
{
    uint32_t id;
    bool lastKnownState;
    bool initialized;
    unsigned long lastEventTime;
    unsigned long lastSupervisorLogTime;

    // Physical contact/garage events and motion events are intentionally
    // tracked independently. The on-air trigger counter is three bits and
    // bit-reversed in the captured status byte. OPEN and CLOSE may still
    // share one counter value on the garage transmitters.
    uint8_t lastPhysicalCounter;
    bool physicalCounterValid;
    uint8_t lastMotionCounter;
    bool motionCounterValid;

    // Candidate event must contain the SAME counter and SAME decoded state
    // in multiple physical frames before it can change the reported state.
    uint8_t candidateCounter;
    bool candidateState;
    uint8_t candidateConfirmCount;
    unsigned long candidateStartTime;

    // Per-sensor confirmation window.
    unsigned long confirmTimeout;
};

struct SensorState
{
    uint32_t id;
    bool isOpen;
    bool sawSupervisor;
    bool sawPhysicalEvent;
    bool initialized;
    unsigned long lastSupervisorLogTime;
};

// Isolated to stack buffers for local function security
struct BitStreamBuffer
{
    uint8_t data[128];
};
struct ByteFrameBuffer
{
    uint8_t data[16];
};
struct LogStringBuffer
{
    char text[128];
};
struct TimeBufBuffer
{
    char text[64];
};
struct PayloadBufBuffer
{
    uint8_t data[512];
};

struct LogItem
{
    LogStringBuffer message_storage;
};
// ============================================================
// BLOCK 2: MEMORY INSTANTIATIONS AND GLOBAL TRACKERS
// ============================================================
RFBurst sessionBurSTS[MAX_BURSTS];
volatile uint16_t completedBurSTS = 0;

volatile uint32_t currentDurations[BUFFER_SIZE];
volatile uint8_t currentLevels[BUFFER_SIZE];
volatile uint16_t currentPulseCount = 0;

volatile uint32_t lastEdgeMicros = 0;
volatile uint32_t currentBurstStartMicros = 0;
volatile uint32_t sessionStartMicros = 0;

volatile bool sessionActive = false;
volatile bool burstActive = false;
volatile bool previousLevel = false;

volatile unsigned long lastIsrTriggerTime = 0;
volatile bool interruptSquelchActive = false;
unsigned long squelchRecoveryTimer = 0;

#define TRACKER_MAX SENSOR_COUNT
LiveTracker liveCache[TRACKER_MAX];
uint8_t trackedSensorCount = 0;

WiFiUDP UdpClient;
unsigned long doorLedTimer = 0;
bool doorLedActive = false;

unsigned long motionLedTimer = 0;
bool motionLedActive = false;

unsigned long lastWiFiCheck = 0;
uint8_t baseR = 40, baseG = 20, baseB = 0;

#define LOG_QUEUE_SIZE 32
LogItem logQueue[LOG_QUEUE_SIZE];
volatile uint8_t logQueueHead = 0;
volatile uint8_t logQueueTail = 0;
portMUX_TYPE logQueueMux = portMUX_INITIALIZER_UNLOCKED;

// Forward Declarations
void checkLedTimeout();
void connectToWiFi();
void enqueueLog(const char *format, ...);
void processLogQueue();
void initCC1101();
void processUnifiedSessionRoll();
void printSession();
void endSession();
void saveCurrentBurst();
void gdo0ISR();
void logSupervisor(uint32_t sensorID, uint8_t statusByte);
int findCacheIndex(uint32_t sensorID);

void IRAM_ATTR gdo0ISR()
{
    uint32_t now = micros();

    if (completedBurSTS >= MAX_BURSTS)
    {
        interruptSquelchActive = true;
        return;
    }

    bool newLevel = digitalRead(CC1101_GDO0);
    if (!sessionActive)
    {
        sessionActive = true;
        sessionStartMicros = now;
    }
    if (!burstActive)
    {
        burstActive = true;
        currentBurstStartMicros = now;
        lastEdgeMicros = now;
        currentPulseCount = 0;
        previousLevel = newLevel;
        return;
    }
    uint32_t duration = now - lastEdgeMicros;
    lastEdgeMicros = now;
    if (currentPulseCount < BUFFER_SIZE)
    {
        currentDurations[currentPulseCount] = duration;
        currentLevels[currentPulseCount] = previousLevel ? 1 : 0;
        currentPulseCount++;
    }
    previousLevel = newLevel;
}

void saveCurrentBurst()
{
    noInterrupts();
    uint16_t count = currentPulseCount;
    if (count >= MIN_TRANSITIONS && completedBurSTS < MAX_BURSTS)
    {
        RFBurst &b = *(sessionBurSTS + completedBurSTS);
        b.count = count;
        b.startTime = currentBurstStartMicros;
        b.endTime = lastEdgeMicros;
        for (uint16_t i = 0; i < count; i++)
        {
            b.duration[i] = currentDurations[i];
            b.level[i] = currentLevels[i];
        }
        completedBurSTS++;
    }
    currentPulseCount = 0;
    burstActive = false;
    interrupts();
}

void printSession()
{
    processUnifiedSessionRoll();
}

void endSession()
{
    if (burstActive && currentPulseCount >= MIN_TRANSITIONS)
    {
        saveCurrentBurst();
    }
    noInterrupts();
    sessionActive = false;
    burstActive = false;
    interrupts();

    processUnifiedSessionRoll();

    noInterrupts();
    completedBurSTS = 0;
    currentPulseCount = 0;
    memset((void *)currentDurations, 0, sizeof(currentDurations));
    memset((void *)currentLevels, 0, sizeof(currentLevels));
    lastEdgeMicros = micros();
    interrupts();
}

static bool isMotionSensor(uint32_t sensorID)
{
    return sensorID == id here || sensorID == id here;
}

static bool isDoorContact(uint32_t sensorID)
{
    return sensorID == id here || sensorID == id here;
}

static bool isGarageSensor(uint32_t sensorID)
{
    return sensorID == id here || sensorID == id here;
}

// Decode the stable level and the event latches observed in the complete
// field captures. Returns false for sensors that do not have an OPEN/CLOSED
// state. "physicalEvent" is false for a periodic supervisor packet.
static bool decodeContactFrame(uint32_t sensorID,
                               const BitStreamBuffer &bits,
                               uint16_t bitCount,
                               bool &state,
                               bool &physicalEvent,
                               bool &positiveLatch,
                               bool &negativeLatch)
{
    positiveLatch = false;
    negativeLatch = false;

    if (isDoorContact(sensorID))
    {
        // Rear and Front contacts use the observed F5 fields:
        //   data[49] = positive/open latch
        //   data[50] = debounced level (1 OPEN, 0 CLOSED)
        //   data[51] = negative/close latch
        if (bitCount < 52)
            return false;

        positiveLatch = bits.data[49] != 0;
        state = bits.data[50] != 0;
        negativeLatch = bits.data[51] != 0;
        physicalEvent = positiveLatch || negativeLatch;
        return true;
    }

    if (isGarageSensor(sensorID))
    {
        // Both garage transmitters use data[41] as the stable level:
        //   52 D1... = OPEN
        //   5A 11... = CLOSED
        // A negative transition asserts both observed latch fields. Some
        // later supervisor cycles can retain one of them, so STEP 2 still
        // makes the stable level—not the counter or prefix—the final guard.
        if (bitCount < 44)
            return false;

        positiveLatch = bits.data[40] != 0;
        state = bits.data[41] != 0;
        negativeLatch = bits.data[36] != 0 && bits.data[43] != 0;
        physicalEvent = positiveLatch || negativeLatch;
        return true;
    }

    return false;
}

static bool decodeMotionEvent(uint8_t statusByte)
{
    bool motionBit = (statusByte & 0x04) != 0;
    return motionBit || statusByte == 0x0A || statusByte == 0x14;
}

static uint8_t decodeEventCounter(uint8_t statusByte)
{
    // The three on-air counter bits occupy status[2:0] in reverse bit order.
    // Example sequence: 1,5,3,7,0,4,2,6 decodes to 4,5,6,7,0,1,2,3.
    uint8_t encoded = statusByte & 0x07;
    return ((encoded & 0x01) << 2) |
           (encoded & 0x02) |
           ((encoded & 0x04) >> 2);
}

void processUnifiedSessionRoll()
{
    noInterrupts();
    uint16_t totalBurSTS = completedBurSTS;
    interrupts();

    if (totalBurSTS == 0)
        return;

    struct ScratchpadState
    {
        uint32_t id;
        const char *name;

        // Supervisor information is kept separately from physical data.
        bool sawSupervisor;
        uint8_t supervisorStatus;
        uint8_t supervisorCounter;
        bool supervisorStateValid;
        bool supervisorState;
        uint8_t supervisorConfirmCount;

        // Physical event candidate.
        bool sawPhysicalEvent;
        bool physicalStateValid;
        bool physicalState;
        uint8_t physicalCounter;
        uint8_t physicalConfirmCount;

        // Motion has no open/closed meaning.
        bool sawMotionEvent;
        uint8_t motionCounter;
        uint8_t motionConfirmCount;

        bool seen;
    };

    ScratchpadState sessionCache[SENSOR_COUNT];
    memset(sessionCache, 0, sizeof(sessionCache));

    // ============================================================
    // STEP 1: DECODE EVERY RF BURST
    // ============================================================
    for (uint16_t bIdx = 0; bIdx < totalBurSTS; bIdx++)
    {
        const RFBurst &b = sessionBurSTS[bIdx];

        BitStreamBuffer local_bit_stream;
        memset(local_bit_stream.data, 0, sizeof(local_bit_stream.data));
        uint16_t bitCount = 0;
        bool syncFound = false;

        for (uint16_t i = 0; i < b.count; i++)
        {
            if (b.duration[i] > 10000)
                continue;

            uint32_t units = (b.duration[i] + (BASE_UNIT_US / 2)) / BASE_UNIT_US;
            if (units < 1)
                units = 1;

            if (b.level[i] == 1 && units >= 7)
            {
                syncFound = true;
                bitCount = 0;
                continue;
            }

            if (syncFound && b.level[i] == 0)
            {
                // Preserve the original decoder timing behavior.
                // A low pulse is a bit slot; 1 unit = 0, 2 units = 1.
                // For compatibility with the working decoder, advance the
                // bit position for every low pulse and leave unrecognized
                // widths as zero.
                if (bitCount < 128)
                {
                    if (units == 1)
                        local_bit_stream.data[bitCount] = 0;
                    else if (units == 2)
                        local_bit_stream.data[bitCount] = 1;
                    else
                        local_bit_stream.data[bitCount] = 0;
                    bitCount++;
                }
                if (bitCount >= 128)
                    break;
            }
        }

        // Forty bits are enough to identify a known transmitter and read its
        // status byte. decodeContactFrame() applies the stricter per-device
        // minimum before it touches each stable-level/latch field.
        if (bitCount < 40)
            continue;

        ByteFrameBuffer local_byte_frame;
        memset(local_byte_frame.data, 0, sizeof(local_byte_frame.data));

        for (uint16_t i = 0; i < 40; i++)
        {
            if (local_bit_stream.data[i] == 1)
                local_byte_frame.data[i / 8] |= (1 << (7 - (i % 8)));
        }

        uint32_t sensorID = ((uint32_t)local_byte_frame.data[1] << 16) |
                            ((uint32_t)local_byte_frame.data[2] << 8) |
                            local_byte_frame.data[3];

        uint8_t statusByte = local_byte_frame.data[4];
        // The trigger counter is encoded in status bits 2:0, bit-reversed.
        // Periodic supervisors advance it, and garage OPEN/CLOSE packets can
        // also legitimately share one counter.
        uint8_t currentCounter = decodeEventCounter(statusByte);

        int listIdx = -1;
        for (size_t i = 0; i < SENSOR_COUNT; i++)
        {
            if (sensorID == (SENSOR_LIST + i)->id)
            {
                listIdx = static_cast<int>(i);
                break;
            }
        }

        if (listIdx == -1)
            continue;

#if RF_DEBUG
        Serial.printf("[FULL_BITS count=%u] ", bitCount);

        for (uint16_t i = 0; i < bitCount; i++)
        {
            Serial.print(local_bit_stream.data[i]);

            if ((i & 0x07) == 0x07 && i + 1 < bitCount)
                Serial.print(' ');
        }

        Serial.println();

        uint16_t byteCount = (bitCount + 7) / 8;
        Serial.printf("[FULL_HEX bytes=%u] ", byteCount);

        for (uint16_t byteIndex = 0; byteIndex < byteCount; byteIndex++)
        {
            uint8_t value = 0;

            for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++)
            {
                uint16_t sourceIndex = byteIndex * 8 + bitIndex;

                if (sourceIndex < bitCount &&
                    local_bit_stream.data[sourceIndex])
                {
                    value |= 1 << (7 - bitIndex);
                }
            }

            Serial.printf("%02X", value);

            if (byteIndex + 1 < byteCount)
                Serial.print(' ');
        }

        Serial.println();
#endif

        ScratchpadState *cell = &sessionCache[listIdx];
        cell->id = sensorID;
        cell->name = (SENSOR_LIST + listIdx)->name;
        cell->seen = true;

        bool motionSensor = isMotionSensor(sensorID);
        bool decodedState = false;
        bool stateFieldValid = false;
        bool positiveLatch = false;
        bool negativeLatch = false;
        bool physicalEvent = false;
        bool supervisor = true;

        if (motionSensor)
        {
            // Counter advancement alone cannot mean motion: untouched Office
            // supervisor packets were observed advancing the counter. Keep
            // the proven motion indication and treat the clear phase as
            // supervisor/rest traffic.
            supervisor = !decodeMotionEvent(statusByte);
        }
        else
        {
            stateFieldValid = decodeContactFrame(sensorID,
                                                 local_bit_stream,
                                                 bitCount,
                                                 decodedState,
                                                 physicalEvent,
                                                 positiveLatch,
                                                 negativeLatch);
            if (stateFieldValid)
                supervisor = !physicalEvent;

            // Once a state is established, another frame reporting that
            // identical stable level is health/repeat traffic even if an old
            // latch remains set. This covers the observed untouched Side
            // Garage 5E 11... supervisor without weakening real transitions.
            int trackerIndex = findCacheIndex(sensorID);
            if (stateFieldValid && trackerIndex >= 0 &&
                liveCache[trackerIndex].initialized &&
                decodedState == liveCache[trackerIndex].lastKnownState)
            {
                physicalEvent = false;
                supervisor = true;
            }
        }

#if RF_DEBUG
        Serial.printf("[RF_RAW] %s ID=0x%06X BYTES=%02X %02X %02X %02X %02X COUNTER=%u STATUS=0x%02X CLASS=%s\n",
                      cell->name, sensorID,
                      local_byte_frame.data[0], local_byte_frame.data[1],
                      local_byte_frame.data[2], local_byte_frame.data[3],
                      local_byte_frame.data[4], currentCounter, statusByte,
                      supervisor ? "SUPERVISOR" : "PHYSICAL");

        if (stateFieldValid)
        {
            Serial.printf("[CONTACT_FIELDS] %s LEVEL=%u POS_LATCH=%u NEG_LATCH=%u\n",
                          cell->name,
                          decodedState ? 1 : 0,
                          positiveLatch ? 1 : 0,
                          negativeLatch ? 1 : 0);
        }
#endif

        if (supervisor)
        {
            // Preserve the supervisor's stable debounced level separately
            // for safe startup initialization. It must never alter or
            // invalidate a physical event collected in the same session.
            if (stateFieldValid)
            {
                if (!cell->supervisorStateValid)
                {
                    cell->supervisorStateValid = true;
                    cell->supervisorState = decodedState;
                    cell->supervisorConfirmCount = 1;
                }
                else if (cell->supervisorState == decodedState)
                {
                    if (cell->supervisorConfirmCount < 255)
                        cell->supervisorConfirmCount++;
                }
                else
                {
                    // Contradictory supervisor copies cannot establish a
                    // startup baseline.
                    cell->supervisorStateValid = false;
                    cell->supervisorConfirmCount = 0;
                }
            }

            cell->sawSupervisor = true;
            cell->supervisorStatus = statusByte;
            cell->supervisorCounter = currentCounter;
            continue;
        }

        // ---------------- PHYSICAL FRAME ----------------
        if (isMotionSensor(sensorID))
        {
            // Motion is simply an event. There is no OPEN/CLOSED state.
            if (!decodeMotionEvent(statusByte))
                continue;

            if (!cell->sawMotionEvent)
            {
                cell->sawMotionEvent = true;
                cell->motionCounter = currentCounter;
                cell->motionConfirmCount = 1;
            }
            else if (cell->motionCounter == currentCounter)
            {
                // Same counter = same physical transmission/event repeat.
                if (cell->motionConfirmCount < 255)
                    cell->motionConfirmCount++;
            }
            else
            {
                // A different counter is a new physical event.
                // Do not mix counters inside one confirmation group.
                cell->motionCounter = currentCounter;
                cell->motionConfirmCount = 1;
            }

            continue;
        }

        if (!stateFieldValid)
            continue;

        if (!cell->sawPhysicalEvent)
        {
            cell->sawPhysicalEvent = true;
            cell->physicalStateValid = true;
            cell->physicalState = decodedState;
            cell->physicalCounter = currentCounter;
            cell->physicalConfirmCount = 1;
        }
        else if (cell->physicalCounter == currentCounter &&
                 cell->physicalState == decodedState)
        {
            // Same counter + same state = another copy of the SAME event.
            if (cell->physicalConfirmCount < 255)
                cell->physicalConfirmCount++;
        }
        else
        {
            // A new counter is a new event. Garage OPEN and CLOSE can also
            // share one counter, so a changed debounced level replaces the
            // candidate instead of being discarded as a contradiction.
            cell->physicalCounter = currentCounter;
            cell->physicalState = decodedState;
            cell->physicalStateValid = true;
            cell->physicalConfirmCount = 1;
        }
    }

    // ============================================================
    // STEP 2: RESOLVE CONFIRMED PHYSICAL EVENTS
    // ============================================================
    unsigned long nowMs = millis();

    for (size_t i = 0; i < SENSOR_COUNT; i++)
    {
        ScratchpadState *cell = &sessionCache[i];
        if (!cell->seen)
            continue;

        uint32_t sID = cell->id;
        const char *sName = cell->name;
        bool motion = isMotionSensor(sID);

        int cacheIdx = findCacheIndex(sID);

        // Create tracker if needed.
        if (cacheIdx == -1 && trackedSensorCount < TRACKER_MAX)
        {
            cacheIdx = trackedSensorCount;
            LiveTracker *t = &liveCache[cacheIdx];

            memset(t, 0, sizeof(LiveTracker));
            t->id = sID;
            t->lastPhysicalCounter = 0xFF;
            t->physicalCounterValid = false;
            t->lastMotionCounter = 0xFF;
            t->motionCounterValid = false;
            t->candidateConfirmCount = 0;
            t->candidateStartTime = 0;

            if (sID == id here || sID == id here)
                t->confirmTimeout = 3000;
            else
                t->confirmTimeout = 2100;

            trackedSensorCount++;
        }

        if (cacheIdx < 0)
            continue;

        LiveTracker *t = &liveCache[cacheIdx];

        // --------------------------------------------------------
        // Supervisors are health traffic. Their debounced level may safely
        // establish the first RAM-only baseline after boot, but a supervisor
        // can never overwrite an already initialized panel state.
        // --------------------------------------------------------
        if (cell->sawSupervisor)
        {
            // If this session also contains a physical event, the
            // supervisor is intentionally ignored for state purposes.
            if (!cell->sawPhysicalEvent && !cell->sawMotionEvent)
            {
                // Keep the most recent supervisor counter as the motion
                // baseline. Counter advancement alone is not motion.
                if (motion)
                {
                    t->lastMotionCounter = cell->supervisorCounter;
                    t->motionCounterValid = true;
                }

                // With no NVS, an untouched sensor may first be seen through
                // its periodic supervisor. Initialize from the stable level,
                // but never generate SENSOR_EVENT for that baseline.
                if (!motion && !t->initialized &&
                    cell->supervisorStateValid &&
                    cell->supervisorConfirmCount >= 2)
                {
                    t->lastKnownState = cell->supervisorState;
                    t->initialized = true;
                    t->lastEventTime = nowMs;

                    enqueueLog("SENSOR_INIT: %s (ID: 0x%06X) baseline state: %s",
                               sName, sID,
                               t->lastKnownState ? "OPEN" : "CLOSED");
                }

                logSupervisor(sID, cell->supervisorStatus);
            }
        }

        // --------------------------------------------------------
        // MOTION PIPELINE
        // --------------------------------------------------------
        if (motion && cell->sawMotionEvent)
        {
            // Motion is an event only; it has no persistent OPEN/CLOSED state.
            if (cell->motionConfirmCount >= 1)
            {
                bool counterChanged = !t->motionCounterValid ||
                                      cell->motionCounter != t->lastMotionCounter;

                // The frame has already passed the motion indication test.
                // The counter now deduplicates repeats of that activation.
                if (counterChanged)
                {
                    t->lastMotionCounter = cell->motionCounter;
                    t->motionCounterValid = true;
                    t->lastEventTime = nowMs;

                    enqueueLog("MOTION_EVENT: %s", sName);
                    neopixelWrite(INTERNAL_LED_PIN, 150, 150, 0);
                    motionLedTimer = nowMs;
                    motionLedActive = true;
                }
            }
            continue;
        }

        // --------------------------------------------------------
        // CONTACT / GARAGE PIPELINE
        // --------------------------------------------------------
        if (!motion && cell->sawPhysicalEvent &&
            cell->physicalStateValid && cell->physicalConfirmCount >= 1)
        {
            uint8_t eventCounter = cell->physicalCounter;
            bool eventState = cell->physicalState;

#if RF_DEBUG
            Serial.printf("[CONTACT_DECODE] %s COUNTER=%u STATE=%s PREVIOUS=%s INITIALIZED=%d\n",
                          sName, eventCounter,
                          eventState ? "OPEN" : "CLOSED",
                          t->lastKnownState ? "OPEN" : "CLOSED",
                          t->initialized ? 1 : 0);
#endif

            bool newEvent = !t->physicalCounterValid ||
                            eventCounter != t->lastPhysicalCounter;

            if (!newEvent && eventState == t->lastKnownState)
            {
                // Only the complete (counter,state) pair identifies a repeat.
                continue;
            }

            // Do NOT require two separate RF sessions here. The sensor's
            // repeated copies can be separated far enough that they land
            // in different sessions, which previously caused a real CLOSE
            // to remain stuck at the original OPEN baseline.

            if (!t->initialized)
            {
                t->lastKnownState = eventState;
                t->initialized = true;
                t->lastPhysicalCounter = eventCounter;
                t->physicalCounterValid = true;
                t->lastEventTime = nowMs;

                enqueueLog("SENSOR_INIT: %s (ID: 0x%06X) baseline state: %s",
                           sName, sID, eventState ? "OPEN" : "CLOSED");
                continue;
            }

            if (eventState == t->lastKnownState)
            {
                // The counter still advances even when the logical state is
                // unchanged (for example, another physical sensor action).
                t->lastPhysicalCounter = eventCounter;
                t->physicalCounterValid = true;
                t->lastEventTime = nowMs;
                t->candidateConfirmCount = 0;
                continue;
            }

            // An opposite state is a real event even when it shares the cycle
            // counter with OPEN. Do not time-squelch contacts.
            t->lastKnownState = eventState;
            t->lastPhysicalCounter = eventCounter;
            t->physicalCounterValid = true;
            t->candidateConfirmCount = 0;
            t->lastEventTime = nowMs;

            if (!eventState)
                neopixelWrite(INTERNAL_LED_PIN, 0, 150, 0);
            else
                neopixelWrite(INTERNAL_LED_PIN, 150, 0, 0);

            doorLedTimer = nowMs;
            doorLedActive = true;

            enqueueLog("SENSOR_EVENT: %s turned: %s",
                       sName, eventState ? "OPEN" : "CLOSED");
        }
    }

    noInterrupts();
    completedBurSTS = 0;
    interrupts();
}

// Utility: find cache index for a given sensor ID
int findCacheIndex(uint32_t sensorID)
{
    for (int c = 0; c < trackedSensorCount; c++)
    {
        if (liveCache[c].id == sensorID)
        {
            return c;
        }
    }
    return -1;
}

// ============================================================
// BLOCK 4: LOG ROUTERS, WI-FI NETWORKING, SYSTEM LOOP
// ============================================================
void enqueueLog(const char *format, ...)
{
    uint8_t nextHead = (logQueueHead + 1) % LOG_QUEUE_SIZE;
    if (nextHead == logQueueTail)
        return;

    va_list args;
    va_start(args, format);

    portENTER_CRITICAL(&logQueueMux); // <-- add
    vsnprintf(logQueue[nextHead].message_storage.text,
              sizeof(logQueue[nextHead].message_storage.text),
              format, args);
    logQueueHead = nextHead;
    portEXIT_CRITICAL(&logQueueMux); // <-- add

    va_end(args);
}

void processLogQueue()
{
    if (logQueueTail == logQueueHead)
        return;

    portENTER_CRITICAL(&logQueueMux); // <-- add
    uint8_t nextTail = (logQueueTail + 1) % LOG_QUEUE_SIZE;
    char *msg = logQueue[nextTail].message_storage.text;
    logQueueTail = nextTail;
    portEXIT_CRITICAL(&logQueueMux); // <-- add

    Serial.println(msg);

    if (WiFi.status() == WL_CONNECTED)
    {
        time_t rawNow = time(nullptr);
        time_t localNow = rawNow - 25200;
        struct tm *timeinfo = gmtime(&localNow);

        char local_string_time[64];
        uint8_t local_string_payload[512];

        memset(local_string_time, 0, sizeof(local_string_time));
        memset(local_string_payload, 0, sizeof(local_string_payload));

        strftime(local_string_time, sizeof(local_string_time) - 1, "%Y-%m-%dT%H:%M:%S", timeinfo);
        int packetLen = snprintf((char *)local_string_payload, sizeof(local_string_payload) - 1,
                                 "<14>1 %s-07:00 AtomS3 SimonXT - - - %s", local_string_time, msg);
        if (packetLen > 0 && packetLen < (int)sizeof(local_string_payload))
        {
            local_string_payload[packetLen] = '\0';
            UdpClient.beginPacket(syslogServer, syslogPort);
            UdpClient.write(local_string_payload, packetLen);
            UdpClient.endPacket();
        }
    }
}

void logSupervisor(uint32_t sensorID, uint8_t statusByte)
{
#if !LOG_SUPERVISORS
    (void)sensorID;
    (void)statusByte;
    return;
#else
    const char *sName = "Unknown Sensor";
    int cacheIdx = -1;

    // Find sensor name
    for (size_t i = 0; i < SENSOR_COUNT; i++)
    {
        if ((SENSOR_LIST + i)->id == sensorID)
        {
            sName = (SENSOR_LIST + i)->name;
            break;
        }
    }

    // Find sensor in liveCache
    for (int c = 0; c < trackedSensorCount; c++)
    {
        if (liveCache[c].id == sensorID)
        {
            cacheIdx = c;
            break;
        }
    }

    unsigned long nowMs = millis();

    // Only log if we found the sensor in the struct and throttle allows
    if (cacheIdx >= 0)
    {
        if (liveCache[cacheIdx].lastSupervisorLogTime == 0 ||
            nowMs - liveCache[cacheIdx].lastSupervisorLogTime >= SUPERVISOR_LOG_INTERVAL_MS)
        {
            liveCache[cacheIdx].lastSupervisorLogTime = nowMs;

            // Status bits 2:0 are the three-bit trigger counter, so they must
            // not be reported as battery/tamper flags.
            enqueueLog("SUPERVISOR_HEALTH: %s (ID: 0x%06X) link status: OK | Counter: 0x%X | Status: 0x%02X",
                       sName, sensorID,
                       decodeEventCounter(statusByte), statusByte);
        }
    }
#endif
}

void checkLedTimeout()
{
    if (doorLedActive && (millis() - doorLedTimer >= 300))
    {
        neopixelWrite(INTERNAL_LED_PIN, baseR, baseG, baseB);
        doorLedActive = false;
    }
    if (motionLedActive && (millis() - motionLedTimer >= 300))
    {
        neopixelWrite(INTERNAL_LED_PIN, baseR, baseG, baseB);
        motionLedActive = false;
    }
}

void connectToWiFi()
{
    Serial.print("Connecting to Wi-Fi via pure DHCP...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 25)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? "\n[NETWORK] Connected!" : "\n[NETWORK] Timeout.");
}

void initCC1101()
{
    Serial.println("Initializing CC1101...");
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.setGDO0(CC1101_GDO0);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setMHZ(319.50);
    ELECHOUSE_cc1101.setModulation(2);
    ELECHOUSE_cc1101.setCCMode(0);
    ELECHOUSE_cc1101.setPktFormat(3);
    ELECHOUSE_cc1101.setRxBW(203.00);
    ELECHOUSE_cc1101.setDRate(16.38);
    ELECHOUSE_cc1101.setSyncMode(0);
    ELECHOUSE_cc1101.setManchester(0);
    ELECHOUSE_cc1101.setCrc(0);
    ELECHOUSE_cc1101.SetRx();
    Serial.println("CC1101 Operational.");
}

void setup()
{
    Serial.begin(115200);
    pinMode(CC1101_CS, OUTPUT);
    digitalWrite(CC1101_CS, HIGH);
    pinMode(CC1101_GDO0, INPUT);
    pinMode(INTERNAL_LED_PIN, OUTPUT);
    neopixelWrite(INTERNAL_LED_PIN, baseR, baseG, baseB);
    delay(2000);
    connectToWiFi();
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("[NETWORK] Stabilizing connection for 10 seconds...");
        delay(10000);
    }
    configTime(0, 0, "192.168.1.1");
    Serial.print("Syncing internal clock with firewall NTP");
    int timeRetry = 0;
    while (time(nullptr) < 100000 && timeRetry < 15)
    {
        delay(500);
        Serial.print(".");
        timeRetry++;
    }
    Serial.println("\nClock sync complete!");
    initCC1101();
    // Permit metadata
    const char *alarmPermitInfo =
        "City of _____ Alarm Permit #___ (Valid ____ to ____)";
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("[TEST] Launching official localized protocol boot packet...");
        enqueueLog("SYSTEM CHECK-IN: Atom S3 Lite Simon XT event monitor online.");
        enqueueLog("PERMIT CONFIRMATION: %s", alarmPermitInfo);
        neopixelWrite(INTERNAL_LED_PIN, 0, 150, 0);
        delay(2000);
        neopixelWrite(INTERNAL_LED_PIN, baseR, baseG, baseB);
    }
    attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), gdo0ISR, CHANGE);
    Serial.println("\n==============================================\n Simon XT Locked Syslog Blended Listener Ready\n==============================================");
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        neopixelWrite(INTERNAL_LED_PIN, 130, 0, 130);
        if (millis() - lastWiFiCheck >= 10000)
        {
            lastWiFiCheck = millis();
            Serial.println("[WARNING] Wi-Fi link down! Attempting re-authentication...");
            WiFi.begin(ssid, password);
        }
    }
    else
        checkLedTimeout();

    if (interruptSquelchActive)
    {
        if (squelchRecoveryTimer == 0)
        {
            detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));
            squelchRecoveryTimer = millis() + 6000;
            Serial.println("\n[DoS WARNING] CC1101 data flooding detected! Squelching receiver pin for 6s...");
        }
        else if (millis() > squelchRecoveryTimer)
        {
            noInterrupts();
            completedBurSTS = 0;
            currentPulseCount = 0;
            interruptSquelchActive = false;
            squelchRecoveryTimer = 0;
            interrupts();
            attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), gdo0ISR, CHANGE);
            Serial.println("[DoS RECOVERY] Receiver un-squelched. Listening to airwaves...");
        }
    }

    uint32_t now = micros();
    if (!interruptSquelchActive)
    {
        if (burstActive && ((uint32_t)(now - lastEdgeMicros) > BURST_GAP_US))
            saveCurrentBurst();
        if (sessionActive && !burstActive && ((uint32_t)(now - lastEdgeMicros) > SESSION_END_GAP_US))
            endSession();
    }

    processLogQueue();
    delayMicroseconds(50);
}
