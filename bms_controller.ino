#define BLYNK_TEMPLATE_ID   "TMPL3BzhfQ-5_"
#define BLYNK_TEMPLATE_NAME "BMS Telemetry"
#define BLYNK_AUTH_TOKEN    "Write Actual Blynk Token" // Fill in Actual token

#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "BmsEngine.h"

#define NUM_CELLS 4

#define red_led    15
#define yellow_led 16
#define green_led  17

#define buzzer 25
#define relay  2

#define RELAY_ACTIVE_HIGH false

const uint8_t cellPins[NUM_CELLS] = { 32, 33, 34, 35 };

const unsigned long SAMPLE_INTERVAL_MS   = 200;
const unsigned long WARN_DEBOUNCE_MS     = 1000;
const unsigned long FAULT_DEBOUNCE_MS    = 800;
const unsigned long RECOVERY_DWELL_MS    = 8000;
const unsigned long BUZZER_PULSE_MS      = 300;
const unsigned long FAULT_REBEEP_MS      = 2000;
const unsigned long RECOVERY_BLINK_MS    = 250;

const unsigned long PAGE_ROTATE_MS       = 3000;
const unsigned long LCD_REFRESH_MS       = 100;

BmsEngine<NUM_CELLS> bms;

enum RelayState { ST_NORMAL, ST_WARNING, ST_FAULT, ST_RECOVERY };

enum SystemState : uint8_t { SYS_NORMAL = 0, SYS_DEGRADED = 1, SYS_FAILSAFE = 2, SYS_SHUTDOWN = 3 };

RelayState state = ST_NORMAL;
unsigned long stateEnteredAt = 0;

unsigned long lastSampleAt   = 0;
unsigned long warnSince      = 0;
unsigned long faultSince     = 0;
unsigned long clearSince     = 0;

unsigned long buzzerOffAt    = 0;
unsigned long lastFaultBeepAt = 0;
unsigned long lastBlinkAt    = 0;
bool blinkPhase = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);

enum LcdPage { PAGE_BATTERY_STATUS, PAGE_SYSTEM_STATE, PAGE_TELEMETRY };
LcdPage currentPage = PAGE_BATTERY_STATUS;

unsigned long lastPageRotateAt = 0;
unsigned long lastLcdRefreshAt  = 0;

char currentLcdBuffer[2][17] = {"                ", "                "};
char nextLcdBuffer[2][17]    = {"                ", "                "};

void setRelayClosed(bool closed);
void setLeds(bool r, bool y, bool g);
void pulseBuzzer();
void serviceBuzzer();
const char* stateName(RelayState s);
void enterState(RelayState s, const char* reason);
void updateLcdEngine(unsigned long now);
void renderDifferentialLcd();
void printTelemetry();

void setRelayClosed(bool closed)
{
    bool level = RELAY_ACTIVE_HIGH ? closed : !closed;
    digitalWrite(relay, level ? HIGH : LOW);
}

void setLeds(bool r, bool y, bool g)
{
    digitalWrite(red_led,    r ? HIGH : LOW);
    digitalWrite(yellow_led, y ? HIGH : LOW);
    digitalWrite(green_led,  g ? HIGH : LOW);
}

void pulseBuzzer()
{
    digitalWrite(buzzer, HIGH);
    buzzerOffAt = millis() + BUZZER_PULSE_MS;
}

void serviceBuzzer()
{
    if (buzzerOffAt != 0 && millis() >= buzzerOffAt)
    {
        digitalWrite(buzzer, LOW);
        buzzerOffAt = 0;
    }
}

const char* stateName(RelayState s)
{
    switch (s)
    {
        case ST_NORMAL:   return "NORMAL";
        case ST_WARNING:  return "WARNING";
        case ST_FAULT:    return "FAULT";
        case ST_RECOVERY: return "RECOVERY";
    }
    return "UNKNOWN";
}

#ifndef BMS_WIFI_SSID
#define BMS_WIFI_SSID "Wokwi-GUEST" //Replace with Actual SSID if doing on actual Hardware leave Wokwi-GUEST for simulation on Wokwi
#endif
#ifndef BMS_WIFI_PASSWORD
#define BMS_WIFI_PASSWORD "" // Give actual Password for wifi leave blank for simulation on wokwi
#endif
const char* WIFI_SSID = BMS_WIFI_SSID;
const char* WIFI_PASSWORD = BMS_WIFI_PASSWORD;

const uint8_t EVENT_QUEUE_CAPACITY = 20;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
const unsigned long BLYNK_RETRY_INTERVAL_MS = 5000;
const unsigned long BLYNK_CONNECT_TIMEOUT_MS = 2000;
const unsigned long REPLAY_INTERVAL_MS = 500;
const unsigned long TX_FIELD_INTERVAL_MS = 150;
const float CELL_EVENT_DELTA_V = 0.03f;
const int RSSI_EVENT_DELTA_DBM = 8;
const unsigned long ANALYTICS_SNAPSHOT_MS = 10000;

struct TelemetryEvent {
    float cellV[NUM_CELLS];
    uint8_t weakestCell;
    uint8_t strongestCell;
    uint8_t relayClosed;
    uint8_t faultState;
    uint8_t wifiConnected;
    uint8_t blynkConnected;
    int16_t rssiDbm;
    uint8_t faultMask;
    float packSoc;
    float imbalanceMv;
    float adaptiveThresholdMv;
    float riskScore;
    uint32_t uptimeSeconds;
    uint32_t totalFaults;
    uint8_t systemStateCode;
    char recommendation[64];
    char systemStateText[12];
    uint32_t sequence;
};

TelemetryEvent eventQueue[EVENT_QUEUE_CAPACITY];
uint8_t queueHead = 0;
uint8_t queueTail = 0;
uint8_t queueCount = 0;
TelemetryEvent activeTxEvent = {};
bool activeTxActive = false;
bool activeTxReplayed = false;
uint8_t activeTxField = 0;
unsigned long lastTxFieldAt = 0;
uint32_t nextEventSequence = 0;
uint32_t droppedEvents = 0;
unsigned long lastWifiAttemptAt = 0;
unsigned long lastBlynkAttemptAt = 0;
unsigned long lastReplayAt = 0;
unsigned long lastConnectionReportAt = 0;
int lastPublishedQueueDepth = -1;

bool networkMonitorReady = false;
bool previousWifiConnected = false;
bool previousBlynkConnected = false;
bool networkEventPending = false;
bool demoCloudOutage = false;
bool telemetryBaselineReady = false;
float previousEventCellV[NUM_CELLS] = {0};
uint8_t previousEventWeakest = 0;
uint8_t previousEventStrongest = 0;
uint8_t previousEventFaultMask = 0;
uint8_t previousEventFaultState = 255;
uint8_t previousEventRelayClosed = 255;
int previousEventRssi = -127;
bool previousEventWifi = false;
bool previousEventBlynk = false;

const uint8_t FAULT_HISTORY_CAPACITY = 32;
uint32_t faultTimestamps[FAULT_HISTORY_CAPACITY] = {0};
uint8_t faultTimestampHead = 0;
uint8_t faultTimestampCount = 0;
uint32_t totalFaultCount = 0;
unsigned long lastAnalyticsSnapshotAt = 0;

SystemState getSystemState() {
    switch (state) {
        case ST_NORMAL:   return SYS_NORMAL;
        case ST_WARNING:  return SYS_DEGRADED;
        case ST_FAULT:    return SYS_FAILSAFE;
        case ST_RECOVERY: return SYS_FAILSAFE;
    }
    return SYS_SHUTDOWN;
}

const char* systemStateName(SystemState s) {
    switch (s) {
        case SYS_NORMAL:   return "NORMAL";
        case SYS_DEGRADED: return "DEGRADED";
        case SYS_FAILSAFE: return "FAILSAFE";
        case SYS_SHUTDOWN: return "SHUTDOWN";
    }
    return "UNKNOWN";
}

void recordFaultOccurrence() {
    const uint32_t now = millis();
    faultTimestamps[faultTimestampHead] = now;
    faultTimestampHead = (uint8_t)((faultTimestampHead + 1) % FAULT_HISTORY_CAPACITY);
    if (faultTimestampCount < FAULT_HISTORY_CAPACITY) ++faultTimestampCount;
    ++totalFaultCount;
}

uint32_t faultsInLast24Hours() {
    const uint32_t now = millis();
    const uint32_t windowMs = 24UL * 60UL * 60UL * 1000UL;
    uint32_t count = 0;
    for (uint8_t i = 0; i < faultTimestampCount; ++i) {
        if ((uint32_t)(now - faultTimestamps[i]) <= windowMs) ++count;
    }
    return count;
}

float clampRisk(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 100.0f) return 100.0f;
    return value;
}

float calculateRiskScore() {
    const float threshold = fmaxf(bms.getThresholdMv(), 1.0f);
    const float imbalanceRisk = clampRisk(
        (bms.getImbalanceMv() / threshold) * 100.0f);
    const float trend = bms.getImbalanceTrendMvPerSec(SAMPLE_INTERVAL_MS);
    const float trendRisk = clampRisk(fmaxf(0.0f, trend) * 10.0f);
    const float faultRisk = clampRisk((float)faultsInLast24Hours() * 20.0f);
    const float socRisk = clampRisk((30.0f - bms.getPackSoC()) * (100.0f / 30.0f));
    return clampRisk(0.40f * imbalanceRisk +
                     0.20f * trendRisk +
                     0.25f * faultRisk +
                     0.15f * socRisk);
}

const char* getMaintenanceRecommendation() {
    if (bms.anySensorFault())
        return "Inspect cell sensor/wiring";
    if (state == ST_FAULT)
        return "FAILSAFE: inspect battery";
    if (state == ST_RECOVERY)
        return "Recovery: verify stability";
    if (state == ST_WARNING)
        return "Check cell imbalance";
    if (WiFi.status() != WL_CONNECTED)
        return "WiFi disconnected";
    if (WiFi.RSSI() < -85)
        return "Weak WiFi: improve signal";
    if (bms.getImbalanceTrendMvPerSec(SAMPLE_INTERVAL_MS) > 1.0f)
        return "Imbalance rising: inspect pack";
    if (calculateRiskScore() >= 60.0f)
        return "High risk: schedule inspection";
    return "Battery operating normally";
}

bool telemetryCloudAvailable() {
    return Blynk.connected() && !demoCloudOutage;
}

bool telemetryWifiAvailable() {
    return WiFi.status() == WL_CONNECTED;
}

void serviceDemoSerialCommands() {
    while (Serial.available() > 0) {
        char command = (char)Serial.read();
        if (command == 'o' || command == 'O') {
            if (!demoCloudOutage) {
                demoCloudOutage = true;
                networkEventPending = true;
                Serial.println("[DEMO] Cloud uplink forced OFFLINE. Change potentiometers to queue events.");
            }
        } else if (command == 'r' || command == 'R') {
            if (demoCloudOutage) {
                demoCloudOutage = false;
                networkEventPending = true;
                Serial.println("[DEMO] Cloud uplink RESTORED. FIFO replay will begin.");
            }
        }
    }
}

bool relayIsClosedNow() {
    return !(state == ST_FAULT || state == ST_RECOVERY);
}

uint8_t currentFaultMask() {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NUM_CELLS; ++i) {
        if (bms.isCellFaulted(i)) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

TelemetryEvent makeTelemetryEvent() {
    TelemetryEvent e = {};
    for (uint8_t i = 0; i < NUM_CELLS; ++i) e.cellV[i] = bms.getCellVoltage(i);
    e.weakestCell = bms.getWeakestCellIndex();
    e.strongestCell = bms.getStrongestCellIndex();
    e.relayClosed = relayIsClosedNow() ? 1 : 0;
    e.faultState = (uint8_t)state;
    e.wifiConnected = telemetryWifiAvailable() ? 1 : 0;
    e.blynkConnected = telemetryCloudAvailable() ? 1 : 0;
    e.rssiDbm = e.wifiConnected ? (int16_t)WiFi.RSSI() : -100;
    e.faultMask = currentFaultMask();
    e.packSoc = constrain(bms.getPackSoC(), 0.0f, 100.0f);
    e.imbalanceMv = bms.getImbalanceMv();
    e.adaptiveThresholdMv = bms.getThresholdMv();
    e.riskScore = calculateRiskScore();
    e.uptimeSeconds = millis() / 1000UL;
    e.totalFaults = totalFaultCount;
    const SystemState sysState = getSystemState();
    e.systemStateCode = (uint8_t)sysState;
    snprintf(e.recommendation, sizeof(e.recommendation), "%s",
             getMaintenanceRecommendation());
    snprintf(e.systemStateText, sizeof(e.systemStateText), "%s",
             systemStateName(sysState));
    if (nextEventSequence >= 2147483646UL) nextEventSequence = 0;
    e.sequence = ++nextEventSequence;
    return e;
}

uint8_t pendingEventCount() {
    return (uint8_t)(queueCount + ((activeTxActive && activeTxReplayed) ? 1 : 0));
}

void publishQueueDepthIfChanged() {
    if (!telemetryCloudAvailable()) return;
    const uint8_t pending = pendingEventCount();
    if (lastPublishedQueueDepth != (int)pending) {
        Blynk.virtualWrite(V12, pending);
        Blynk.virtualWrite(V16, (long)droppedEvents);
        lastPublishedQueueDepth = pending;
    }
}

bool enqueueEvent(const TelemetryEvent &e) {
    if (queueCount >= EVENT_QUEUE_CAPACITY) {
        ++droppedEvents;
        Serial.printf("[QUEUE FULL] Dropped newest event; total dropped=%lu\n", (unsigned long)droppedEvents);
        return false;
    }
    eventQueue[queueTail] = e;
    queueTail = (uint8_t)((queueTail + 1) % EVENT_QUEUE_CAPACITY);
    ++queueCount;
    Serial.printf("[QUEUED] seq=%lu depth=%u\n", (unsigned long)e.sequence, queueCount);
    publishQueueDepthIfChanged();
    return true;
}

bool dequeueEvent(TelemetryEvent &e) {
    if (queueCount == 0) return false;
    e = eventQueue[queueHead];
    queueHead = (uint8_t)((queueHead + 1) % EVENT_QUEUE_CAPACITY);
    --queueCount;
    return true;
}

//Sending Data to Blynk Dashboard
void beginTelemetryTransmission(const TelemetryEvent &e, bool replayed) {
    activeTxEvent = e;
    activeTxActive = true;
    activeTxReplayed = replayed;
    activeTxField = 0;
    lastTxFieldAt = millis() - TX_FIELD_INTERVAL_MS;
    Serial.printf("[%s START] seq=%lu queue=%u\n",
                  replayed ? "REPLAY" : "LIVE", (unsigned long)e.sequence,
                  pendingEventCount());
}

void serviceTelemetryTransmission(unsigned long now) {
    if (!activeTxActive || !telemetryCloudAvailable()) return;
    if (now - lastTxFieldAt < TX_FIELD_INTERVAL_MS) return;
    lastTxFieldAt = now;

    const TelemetryEvent &e = activeTxEvent;
    switch (activeTxField) {
        case 0:  Blynk.virtualWrite(V0, e.cellV[0]); break;
        case 1:  Blynk.virtualWrite(V1, e.cellV[1]); break;
        case 2:  Blynk.virtualWrite(V2, e.cellV[2]); break;
        case 3:  Blynk.virtualWrite(V3, e.cellV[3]); break;
        case 4:  Blynk.virtualWrite(V4, e.weakestCell + 1); break;
        case 5:  Blynk.virtualWrite(V5, e.cellV[e.weakestCell]); break;
        case 6:  Blynk.virtualWrite(V6, e.strongestCell + 1); break;
        case 7:  Blynk.virtualWrite(V7, e.cellV[e.strongestCell]); break;
        case 8:  Blynk.virtualWrite(V8, e.relayClosed); break;
        case 9:  Blynk.virtualWrite(V9, e.faultState); break;
        case 10: Blynk.virtualWrite(V10, e.rssiDbm); break;
        case 11: Blynk.virtualWrite(V11, e.wifiConnected); break;
        case 12: Blynk.virtualWrite(V12, pendingEventCount()); lastPublishedQueueDepth = pendingEventCount(); break;
        case 13: Blynk.virtualWrite(V13, activeTxReplayed ? 1 : 0); break;
        case 14: Blynk.virtualWrite(V15, e.blynkConnected); break;
        case 15: Blynk.virtualWrite(V16, (long)droppedEvents); break;
        case 16: break;
        case 17:
            Blynk.virtualWrite(V17, e.packSoc);
            Serial.printf("[TX] V17 SoC=%.1f%%\n", e.packSoc);
            break;
        case 18:
            Blynk.virtualWrite(V18, e.imbalanceMv);
            Serial.printf("[TX] V18 imbalance=%.1f mV\n", e.imbalanceMv);
            break;
        case 19:
            Blynk.virtualWrite(V19, e.adaptiveThresholdMv);
            Serial.printf("[TX] V19 threshold=%.1f mV\n", e.adaptiveThresholdMv);
            break;
        case 20:
            Blynk.virtualWrite(V20, e.riskScore);
            Serial.printf("[TX] V20 risk=%.1f\n", e.riskScore);
            break;
        case 21:
            Blynk.virtualWrite(V21, (long)e.uptimeSeconds);
            Serial.printf("[TX] V21 uptime=%lu s\n", (unsigned long)e.uptimeSeconds);
            break;
        case 22:
            Blynk.virtualWrite(V22, (long)e.totalFaults);
            Serial.printf("[TX] V22 totalFaults=%lu\n", (unsigned long)e.totalFaults);
            break;
        case 23:
            Blynk.virtualWrite(V23, e.systemStateCode);
            Serial.printf("[TX] V23 stateCode=%u\n", e.systemStateCode);
            break;
        case 24:
            Blynk.virtualWrite(V24, e.recommendation);
            Serial.printf("[TX] V24 recommendation=%s\n", e.recommendation);
            break;
        case 25:
            Blynk.virtualWrite(V25, e.systemStateText);
            Blynk.virtualWrite(V14, (long)e.sequence);
            Serial.printf("[TX] V25 state=%s | snapshot seq=%lu COMPLETE\n",
                          e.systemStateText, (unsigned long)e.sequence);
            break;
    }

    ++activeTxField;
    if (activeTxField >= 26) {
        const bool wasReplay = activeTxReplayed;
        activeTxActive = false;
        Serial.printf("[%s SENT] seq=%lu queue=%u WiFi=%u Blynk-at-event=%u RSSI=%d dBm\n",
                      wasReplay ? "REPLAY" : "LIVE", (unsigned long)e.sequence,
                      queueCount, e.wifiConnected, e.blynkConnected, e.rssiDbm);
        publishQueueDepthIfChanged();
    }
}

void createTelemetryEvent() {
    TelemetryEvent e = makeTelemetryEvent();
    if (queueCount > 0 || activeTxActive || !telemetryCloudAvailable()) {
        enqueueEvent(e);
    } else {
        beginTelemetryTransmission(e, false);
    }
}

void serviceWifiStateMachine(unsigned long now) {
    if (WiFi.status() != WL_CONNECTED && now - lastWifiAttemptAt >= WIFI_RETRY_INTERVAL_MS) {
        lastWifiAttemptAt = now;
        Serial.println("[WiFi] Reconnection attempt (non-blocking)");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    if (WiFi.status() == WL_CONNECTED && !Blynk.connected() &&
        now - lastBlynkAttemptAt >= BLYNK_RETRY_INTERVAL_MS) {
        lastBlynkAttemptAt = now;
        Serial.println("[Blynk] Connection attempt (timeout 2000 ms)");
        const bool connected = Blynk.connect(BLYNK_CONNECT_TIMEOUT_MS);
        Serial.printf("[Blynk] Connect result: %s\n", connected ? "CONNECTED" : "FAILED");
    }
}

void reportNetworkStatus(unsigned long now) {
    if (now - lastConnectionReportAt < 5000) return;
    lastConnectionReportAt = now;

    Serial.printf("[LINK] WiFi=%s RSSI=%d dBm | Blynk=%s | demoOutage=%s | queue=%u | txActive=%s\n",
                  WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED",
                  WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -100,
                  Blynk.connected() ? "CONNECTED" : "DISCONNECTED",
                  demoCloudOutage ? "YES" : "NO",
                  queueCount,
                  activeTxActive ? "YES" : "NO");
}

void monitorNetworkTransitions() {
    const bool wifiNow = telemetryWifiAvailable();
    const bool blynkNow = telemetryCloudAvailable();
    if (!networkMonitorReady) {
        previousWifiConnected = wifiNow;
        previousBlynkConnected = blynkNow;
        networkMonitorReady = true;
        return;
    }
    if (wifiNow != previousWifiConnected || blynkNow != previousBlynkConnected) {
        Serial.printf("[NETWORK] WiFi %s, Blynk %s\n",
                      wifiNow ? "CONNECTED" : "DISCONNECTED",
                      blynkNow ? "CONNECTED" : "DISCONNECTED");
        networkEventPending = true;
        previousWifiConnected = wifiNow;
        previousBlynkConnected = blynkNow;
    }
}

void serviceOfflineQueue(unsigned long now) {
    if (!telemetryCloudAvailable() || activeTxActive || queueCount == 0) return;
    if (now - lastReplayAt < REPLAY_INTERVAL_MS) return;

    lastReplayAt = now;
    TelemetryEvent e;
    if (dequeueEvent(e)) {
        beginTelemetryTransmission(e, true);
        publishQueueDepthIfChanged();
    }
}

void checkForTelemetryChanges() {
    if (lastSampleAt == 0) return;

    const bool wifiNow = telemetryWifiAvailable();
    const bool blynkNow = telemetryCloudAvailable();
    const int rssiNow = wifiNow ? WiFi.RSSI() : -100;
    const uint8_t faultMask = currentFaultMask();
    bool changed = !telemetryBaselineReady || networkEventPending ||
                   (millis() - lastAnalyticsSnapshotAt >= ANALYTICS_SNAPSHOT_MS);

    if (telemetryBaselineReady) {
        if (state != (RelayState)previousEventFaultState ||
            relayIsClosedNow() != (bool)previousEventRelayClosed ||
            faultMask != previousEventFaultMask ||
            bms.getWeakestCellIndex() != previousEventWeakest ||
            bms.getStrongestCellIndex() != previousEventStrongest ||
            wifiNow != previousEventWifi || blynkNow != previousEventBlynk) {
            changed = true;
        }
        if (wifiNow && previousEventRssi != -127 && abs(rssiNow - previousEventRssi) >= RSSI_EVENT_DELTA_DBM) {
            changed = true;
        }
        for (uint8_t i = 0; i < NUM_CELLS; ++i) {
            if (fabsf(bms.getCellVoltage(i) - previousEventCellV[i]) >= CELL_EVENT_DELTA_V) {
                changed = true;
            }
        }
    }

    if (changed) {
        createTelemetryEvent();
        lastAnalyticsSnapshotAt = millis();
    }

    for (uint8_t i = 0; i < NUM_CELLS; ++i) previousEventCellV[i] = bms.getCellVoltage(i);
    previousEventWeakest = bms.getWeakestCellIndex();
    previousEventStrongest = bms.getStrongestCellIndex();
    previousEventFaultMask = faultMask;
    previousEventFaultState = (uint8_t)state;
    previousEventRelayClosed = relayIsClosedNow() ? 1 : 0;
    previousEventRssi = rssiNow;
    previousEventWifi = wifiNow;
    previousEventBlynk = blynkNow;
    telemetryBaselineReady = true;
    networkEventPending = false;
}

void enterState(RelayState s, const char* reason)
{
    const RelayState previous = state;
    const char* previousName = stateName(previous);
    const char* nextName = stateName(s);

    if (s == ST_FAULT && previous != ST_FAULT) recordFaultOccurrence();

    state = s;
    stateEnteredAt = millis();

    if (Blynk.connected() && previous != s) {
        char eventMessage[128];
        snprintf(eventMessage, sizeof(eventMessage),
                 "t=%lu ms | %s->%s | faultMask=0x%02X | %s",
                 (unsigned long)stateEnteredAt, previousName, nextName,
                 currentFaultMask(), reason);
        Blynk.logEvent("bms_transition", eventMessage);
        if (s == ST_FAULT)
            Blynk.logEvent("bms_fault", eventMessage);
        else if (s == ST_WARNING)
            Blynk.logEvent("bms_warning", eventMessage);
        else if (s == ST_RECOVERY)
            Blynk.logEvent("bms_recovery", eventMessage);
    }

    warnSince  = 0;
    faultSince = 0;
    clearSince = 0;

    Serial.println();
    Serial.print("[");
    Serial.print(stateEnteredAt);
    Serial.print(" ms] >>> STATE TRANSITION: ");
    Serial.print(stateName(s));
    Serial.print(" | Reason: ");
    Serial.print(reason);
    Serial.print(" | Executive: ");
    Serial.print(systemStateName(getSystemState()));
    Serial.println(" <<<");

    switch (s)
    {
        case ST_NORMAL:
            setRelayClosed(true);
            setLeds(false, false, true);
            break;

        case ST_WARNING:
            setRelayClosed(true);
            setLeds(false, true, false);
            break;

        case ST_FAULT:
            setRelayClosed(false);
            setLeds(true, false, false);
            pulseBuzzer();
            lastFaultBeepAt = millis();
            break;

        case ST_RECOVERY:
            setRelayClosed(false);
            break;
    }
}

void printTelemetry()
{
    Serial.print("State: ");
    Serial.print(stateName(state));
    Serial.print(" | Cell V: [");
    for (uint8_t i = 0; i < NUM_CELLS; i++)
    {
        Serial.print(bms.getCellVoltage(i), 2);
        if (i < NUM_CELLS - 1) Serial.print("V, ");
    }
    Serial.print("V] | SoC: ");
    Serial.print(bms.getPackSoC(), 1);
    Serial.print("% | Imb: ");
    Serial.print(bms.getImbalanceMv(), 0);
    Serial.print("mV (Th: ");
    Serial.print(bms.getThresholdMv(), 0);
    Serial.print("mV) | Trend: ");
    float trend = bms.getImbalanceTrendMvPerSec(SAMPLE_INTERVAL_MS);
    Serial.print(trend >= 0 ? "+" : "");
    Serial.print(trend, 1);
    Serial.print("mV/s | Relay: ");
    Serial.println((state == ST_FAULT || state == ST_RECOVERY) ? "OPEN" : "CLOSED");

    if (bms.wasTransientSuppressed())
    {
        Serial.println("  (correlated multi-channel spike suppressed as load transient)");
    }
    for (uint8_t i = 0; i < NUM_CELLS; i++)
    {
        if (bms.isCellFaulted(i))
        {
            Serial.print("  WARNING: Sensor fault on Cell ");
            Serial.print(i + 1);
            Serial.print(" -> OutOfRange: ");
            Serial.print(bms.isCellOutOfRange(i));
            Serial.print(", Frozen: ");
            Serial.print(bms.isCellFrozen(i));
            Serial.print(", PersistentJumpCount: ");
            Serial.println(bms.getJumpConfirmCount(i));
        }
    }
}

void renderDifferentialLcd()
{
    for (uint8_t r = 0; r < 2; r++)
    {
        for (uint8_t c = 0; c < 16; c++)
        {
            if (nextLcdBuffer[r][c] != currentLcdBuffer[r][c])
            {
                lcd.setCursor(c, r);
                lcd.print(nextLcdBuffer[r][c]);
                currentLcdBuffer[r][c] = nextLcdBuffer[r][c];
            }
        }
    }
}

void updateLcdEngine(unsigned long now)
{
    if (state != ST_FAULT)
    {
        if (now - lastPageRotateAt >= PAGE_ROTATE_MS)
        {
            lastPageRotateAt = now;
            currentPage = static_cast<LcdPage>((currentPage + 1) % 3);
        }
    }

    char line0[17];
    char line1[17];

    if (state == ST_FAULT)
    {
        snprintf(line0, sizeof(line0), "FAULT: OVERRIDE ");
        snprintf(line1, sizeof(line1), "IMB:%.0fmV TH:%.0f", bms.getImbalanceMv(), bms.getThresholdMv());
    }
    else
    {
        switch (currentPage)
        {
            case PAGE_BATTERY_STATUS:
            {
                float trend = bms.getImbalanceTrendMvPerSec(SAMPLE_INTERVAL_MS);
                char trendCh = (trend > 1.0f) ? '^' : (trend < -1.0f) ? 'v' : '-';
                snprintf(line0, sizeof(line0), "SoC: %-3.1f%%       ", bms.getPackSoC());
                snprintf(line1, sizeof(line1), "Imb:%-4.0fmV %c    ", bms.getImbalanceMv(), trendCh);
                break;
            }

            case PAGE_SYSTEM_STATE:
                snprintf(line0, sizeof(line0), "State:%-10s", stateName(state));
                snprintf(line1, sizeof(line1), "Relay:%-10s", (state == ST_FAULT || state == ST_RECOVERY) ? "OPEN" : "CLOSED");
                break;

            case PAGE_TELEMETRY:
                snprintf(line0, sizeof(line0), "S:C%d %1.2fV      ", bms.getStrongestCellIndex() + 1, bms.getCellVoltage(bms.getStrongestCellIndex()));
                snprintf(line1, sizeof(line1), "W:C%d %1.2fV      ", bms.getWeakestCellIndex() + 1, bms.getCellVoltage(bms.getWeakestCellIndex()));
                break;
        }
    }

    for (int i = strlen(line0); i < 16; i++) line0[i] = ' ';
    for (int i = strlen(line1); i < 16; i++) line1[i] = ' ';
    line0[16] = '\0';
    line1[16] = '\0';

    strncpy(nextLcdBuffer[0], line0, 17);
    strncpy(nextLcdBuffer[1], line1, 17);

    if (now - lastLcdRefreshAt >= LCD_REFRESH_MS)
    {
        lastLcdRefreshAt = now;
        renderDifferentialLcd();
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println("\n=============================================");
    Serial.println("  ESP32 BMS Protection Controller Starting   ");
    Serial.println("=============================================");
    if (strcmp(BLYNK_AUTH_TOKEN, "REPLACE_WITH_NEW_BLYNK_AUTH_TOKEN") == 0) {
        Serial.println("[CONFIG ERROR] Replace BLYNK_AUTH_TOKEN with the device token from Blynk.");
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    lastWifiAttemptAt = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Blynk.config(BLYNK_AUTH_TOKEN);

    pinMode(red_led, OUTPUT);
    pinMode(yellow_led, OUTPUT);
    pinMode(green_led, OUTPUT);
    pinMode(buzzer, OUTPUT);
    pinMode(relay, OUTPUT);

    BmsEngine<NUM_CELLS>::Config cfg;
    for (uint8_t i = 0; i < NUM_CELLS; i++) cfg.pins[i] = cellPins[i];
    cfg.adcRefVoltage       = 3.3f;
    cfg.adcMaxCounts        = 4095;
    cfg.cellVoltageMin      = 0.20f;
    cfg.cellVoltageMax      = 3.20f;
    cfg.frozenEpsilonCounts = 1;
    cfg.frozenSampleLimit   = 150;
    cfg.maxJumpCounts       = 800;
    cfg.jumpConfirmSamples  = 3;
    bms.begin(cfg);

    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    lcd.clear();

    stateEnteredAt = millis();
    setRelayClosed(true);
    setLeds(false, false, true);
    Serial.println("BMS protection relay online — Initial State: NORMAL\n");
}

void loop()
{
    unsigned long now = millis();

    serviceDemoSerialCommands();
    serviceWifiStateMachine(now);
    Blynk.run();
    reportNetworkStatus(now);
    monitorNetworkTransitions();
    serviceOfflineQueue(now);
    serviceTelemetryTransmission(now);

    serviceBuzzer();

    if (state == ST_RECOVERY && now - lastBlinkAt >= RECOVERY_BLINK_MS)
    {
        lastBlinkAt = now;
        blinkPhase = !blinkPhase;
        setLeds(false, blinkPhase, !blinkPhase);
    }

    if (state == ST_FAULT && now - lastFaultBeepAt >= FAULT_REBEEP_MS)
    {
        lastFaultBeepAt = now;
        pulseBuzzer();
    }

    updateLcdEngine(now);

    if (now - lastSampleAt < SAMPLE_INTERVAL_MS) return;
    lastSampleAt = now;

    bms.update();

    bool  anySensorFault  = bms.anySensorFault();
    float currentImbalance = bms.getImbalanceMv();
    float currentThreshold = bms.getThresholdMv();

    float warnThreshold  = currentThreshold * 0.8f;
    float resetThreshold = currentThreshold * 0.6f;

    printTelemetry();

    switch (state)
    {
        case ST_NORMAL:
        case ST_WARNING:
        {
            if (anySensorFault)
            {
                enterState(ST_FAULT, "sensor anomaly detected");
                break;
            }

            if (currentImbalance > currentThreshold)
            {
                if (faultSince == 0) faultSince = now;
                if (now - faultSince >= FAULT_DEBOUNCE_MS)
                {
                    enterState(ST_FAULT, "imbalance exceeded threshold");
                    break;
                }
            }
            else
            {
                faultSince = 0;
            }

            if (state == ST_NORMAL)
            {
                if (currentImbalance > warnThreshold)
                {
                    if (warnSince == 0) warnSince = now;
                    if (now - warnSince >= WARN_DEBOUNCE_MS)
                        enterState(ST_WARNING, "imbalance approaching limit");
                }
                else
                {
                    warnSince = 0;
                }
            }
            else
            {
                if (currentImbalance <= resetThreshold)
                {
                    if (clearSince == 0) clearSince = now;
                    if (now - clearSince >= WARN_DEBOUNCE_MS)
                        enterState(ST_NORMAL, "imbalance back within normal range");
                }
                else
                {
                    clearSince = 0;
                }
            }
            break;
        }

        case ST_FAULT:
        {
            if (!anySensorFault && currentImbalance <= resetThreshold)
            {
                if (clearSince == 0) clearSince = now;
                if (now - clearSince >= FAULT_DEBOUNCE_MS)
                    enterState(ST_RECOVERY, "fault condition cleared, starting recovery dwell");
            }
            else
            {
                clearSince = 0;
            }
            break;
        }

        case ST_RECOVERY:
        {
            if (anySensorFault || currentImbalance > resetThreshold)
            {
                enterState(ST_FAULT, "re-faulted during recovery dwell");
            }
            else if (now - stateEnteredAt >= RECOVERY_DWELL_MS)
            {
                enterState(ST_NORMAL, "recovery dwell complete, system verified stable");
            }
            break;
        }
    }

    checkForTelemetryChanges();
}
