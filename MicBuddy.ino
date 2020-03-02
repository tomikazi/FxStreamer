#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2020.03.02.005"

#define MIC_PIN         A0
#define PIR_PIN          5

#define GROUP_PORT  7001
#define BUDDY_PORT  7002

WiFiUDP udp;
WiFiUDP buddy;

IPAddress buddyIp(224,0,42,69);

IPAddress lamp1(192,168,0,15);
IPAddress lamp2(192,168,0,25);

typedef struct {
    uint16_t sampleavg;
    uint16_t samplepeak;
    uint16_t oldsample;
} MicSample;

#define SAMPLING_REMINDER   30000
#define SAMPLE  100

typedef struct {
    uint16_t op;
    uint16_t param;
} Command;

// For moving averages
uint32_t mat = 0;
uint32_t nmat = 0;

// Sample average, max and peak detection
uint16_t sampleavg = 0;
uint16_t oldsample = 0;
uint16_t samplepeak = 0;

boolean sampling = false;
uint32_t lastReminder = 0;

void setup() {
    gizmo.beginSetup(MIC_BUDDY, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        handleCommand();
        handleMic();
    }
}

void finishWiFiConnect() {
    udp.begin(GROUP_PORT);
    buddy.beginMulticast(WiFi.localIP(), buddyIp, BUDDY_PORT);
    Serial.printf("%s is ready\n", MIC_BUDDY);
}

void handleCommand() {
    Command command;
    while (buddy.parsePacket()) {
        int len = buddy.read((char *) &command, sizeof(command));
        if (len < 0) {
            Serial.printf("Unable to read command!!!!\n");
        }

        switch (command.op) {
            case SAMPLE:
                sampling = command.param != 0;
                lastReminder = millis() + SAMPLING_REMINDER;
                Serial.printf("Sampling is %s\n", sampling ? "on" : "off");
                break;
            default:
                break;
        }
    }

    if (lastReminder && lastReminder < millis()) {
        // Process aged out sampling requests
        sampling = false;
        lastReminder = 0;
        Serial.printf("Sampling timed out\n");
    }
}

void handleMic() {
    uint16_t v = analogRead(MIC_PIN);

    mat = mat + v - (mat >> 8);
    uint16_t av = abs(v - (mat >> 8));

    if (av < 4) {
        av = 0;
    } else if (av > 128) {
        av = 128;
    }

    av = map(av, 0, 128, 0, 255);
    nmat = nmat + av - (nmat >> 4);
    sampleavg = nmat >> 4;

    // We're on the down swing, so we just peaked.
    samplepeak = av > (sampleavg + 16) && (av < oldsample);
    oldsample = av;

    if (sampling) {
        MicSample sample;
        sample.sampleavg = sampleavg;
        sample.samplepeak = samplepeak;
        sample.oldsample = oldsample;

        udp.beginPacket(lamp1, GROUP_PORT);
        udp.write((char *) &sample, sizeof(sample));
        udp.endPacket();

        udp.beginPacket(lamp2, GROUP_PORT);
        udp.write((char *) &sample, sizeof(sample));
        udp.endPacket();

        Serial.printf("255, %d, %d, %d\n", av, sampleavg, samplepeak * 200);
    }

    delay(5);
}
