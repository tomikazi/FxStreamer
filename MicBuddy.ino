#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2020.03.02.008"

#define MIC_PIN         A0
#define PIR_PIN          5

#define GROUP_PORT  7001
#define BUDDY_PORT  7002

WiFiUDP udp;
WiFiUDP buddy;

IPAddress buddyIp(224,0,42,69);

#define MAX_LAMPS   4
uint32_t lampIps[MAX_LAMPS];
uint32_t lampTimes[MAX_LAMPS];

typedef struct {
    uint16_t sampleavg;
    uint16_t samplepeak;
    uint16_t oldsample;
} MicSample;

#define SAMPLING_REMINDER   10000
#define SAMPLE  100

typedef struct {
    uint16_t op;
    uint16_t param;
    uint32_t src;
} Command;

// For moving averages
uint32_t mat = 0;
uint32_t nmat = 0;

// Sample average, max and peak detection
uint16_t sampleavg = 0;
uint16_t oldsample = 0;
uint16_t samplepeak = 0;

void setup() {
    gizmo.beginSetup(MIC_BUDDY, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        handleMulticast();
        handleMic();
    }
}

void finishWiFiConnect() {
    udp.begin(GROUP_PORT);
    buddy.beginMulticast(WiFi.localIP(), buddyIp, BUDDY_PORT);
    Serial.printf("%s is ready\n", MIC_BUDDY);
}

void addLamp(uint32_t ip) {
    int ai = -1;
    for (int i = 0; i < MAX_LAMPS; i++) {
        if (ip == lampIps[i]) {
            lampTimes[i] = millis() + SAMPLING_REMINDER;
            return;
        }
        if (ai < 0 && !lampIps[i]) {
            ai = i;
        }
    }
    if (ai >= 0) {
        lampIps[ai] = ip;
        lampTimes[ai] = millis() + SAMPLING_REMINDER;
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
    while (buddy.parsePacket()) {
        int len = buddy.read((char *) &command, sizeof(command));
        if (len < 0) {
            Serial.printf("Unable to read command!!!!\n");
        }

        switch (command.op) {
            case SAMPLE:
                if (command.param) {
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
            udp.beginPacket(IPAddress(lampIps[i]), GROUP_PORT);
            udp.write((char *) &sample, sizeof(sample));
            udp.endPacket();
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
