#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2020.03.03.004"

#define MIC_PIN         A0
#define PIR_PIN          5

#define BUDDY_PORT  7001
WiFiUDP buddy;

#define GROUP_PORT  7002
WiFiUDP group;
IPAddress groupIp(224, 0, 42, 69);

#define MAX_LAMPS   4
uint32_t lampIps[MAX_LAMPS];
uint32_t lampTimes[MAX_LAMPS];

typedef struct {
    uint16_t sampleavg;
    uint16_t samplepeak;
    uint16_t oldsample;
} MicSample;

#define FRONT_CTX       0x01
#define BACK_CTX        0x10
#define ALL_CTX         FRONT_CTX | BACK_CTX

#define SAMPLE_REQ      100
#define SAMPLE_ADV      101

#define MAX_CMD_DATA    64
typedef struct {
    uint32_t src;
    uint16_t ctx;
    uint16_t op;
    uint8_t  data[MAX_CMD_DATA];
} Command;

#define AD_FREQUENCY     3000
#define HELLO_TIMEOUT   10000

// For moving averages
uint32_t mat = 0;
uint32_t nmat = 0;

// Sample average, max and peak detection
uint16_t sampleavg = 0;
uint16_t oldsample = 0;
uint16_t samplepeak = 0;

uint32_t lastAdvertisement = 0;

void setup() {
    gizmo.beginSetup(MIC_BUDDY, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        handleMulticast();
        handleMic();
        handleAdvertisement();
    }
}

void finishWiFiConnect() {
    buddy.begin(BUDDY_PORT);
    group.beginMulticast(WiFi.localIP(), groupIp, GROUP_PORT);
    advertise();
    Serial.printf("%s is ready\n", MIC_BUDDY);
}

void handleAdvertisement() {
    if (lastAdvertisement < millis()) {
        lastAdvertisement = millis() + AD_FREQUENCY;
        advertise();
    }
}

void advertise() {
    Command ad = { .src = (uint) WiFi.localIP(), .ctx = ALL_CTX, .op = SAMPLE_ADV, .data = { [0] = 0}};
    group.beginPacketMulticast(groupIp, GROUP_PORT, WiFi.localIP());
    group.write((char *) &ad, sizeof(ad));
    group.endPacket();
}

void addLamp(uint32_t ip) {
    int ai = -1;
    for (int i = 0; i < MAX_LAMPS; i++) {
        if (ip == lampIps[i]) {
            lampTimes[i] = millis() + HELLO_TIMEOUT;
            return;
        }
        if (ai < 0 && !lampIps[i]) {
            ai = i;
        }
    }
    if (ai >= 0) {
        lampIps[ai] = ip;
        lampTimes[ai] = millis() + HELLO_TIMEOUT;
        advertise();
        Serial.printf("Lamp %s added\n", IPAddress(ip).toString().c_str());
    }
}

void removeLamp(uint32_t  ip) {
    for (int i = 0; i < MAX_LAMPS; i++) {
        if (ip == lampIps[i]) {
            lampIps[i] = 0;
            lampTimes[i] = 0;
            Serial.printf("Lamp %s removed\n", IPAddress(ip).toString().c_str());
            return;
        }
    }
}


void handleMulticast() {
    Command command;
    while (group.parsePacket()) {
        int len = group.read((char *) &command, sizeof(command));
        if (len < 0) {
            Serial.printf("Unable to read command!!!!\n");
        }

        switch (command.op) {
            case SAMPLE_REQ:
                if (command.data[0]) {
                    addLamp(command.src);
                } else {
                    removeLamp(command.src);
                }
                break;
            default:
                break;
        }
    }
}

uint16_t minv = 4;
uint16_t maxv = 128;

void handleMic() {
    uint16_t v = analogRead(MIC_PIN);

    mat = mat + v - (mat >> 8);
    uint16_t av = abs(v - (mat >> 8));

    if (av < minv) {
        av = 0;
    } else if (av > maxv) {
        av = maxv;
    }

    av = map(av, 0, maxv, 0, 255);
    nmat = nmat + av - (nmat >> 4);
    sampleavg = nmat >> 4;

    // We're on the down swing, so we just peaked.
    samplepeak = av > (sampleavg + 16) && (av < oldsample);
    oldsample = av;

    MicSample sample;
    sample.sampleavg = sampleavg;
    sample.samplepeak = samplepeak;
    sample.oldsample = oldsample;

    uint32_t now = millis();
    boolean sampling = false;
    for (int i = 0; i < MAX_LAMPS; i++) {
        if (lampIps[i] && lampTimes[i] >= now) {
            buddy.beginPacket(IPAddress(lampIps[i]), BUDDY_PORT);
            buddy.write((char *) &sample, sizeof(sample));
            buddy.endPacket();
            sampling = true;
        } else if (lampTimes[i] && lampTimes[i] < now) {
            Serial.printf("Lamp %s timed out\n", IPAddress(lampIps[i]).toString().c_str());
            lampIps[i] = 0;
            lampTimes[i] = 0;
        }
    }

    if (sampling) {
        Serial.printf("255, %d, %d, %d\n", av, sampleavg, samplepeak * 200);
    }

    delay(5);
}
