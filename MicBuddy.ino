#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2020.03.01.001"

#define MIC_PIN         A0
#define PIR_PIN          5

WiFiUDP UDP;

// TODO: Make configurable
IPAddress dest(192,168,1,243);

typedef struct {
    uint16_t sampleavg;
    uint16_t samplepeak;
    uint16_t oldsample;
} MicSample;

// For moving averages
uint32_t mat = 0;
uint32_t nmat = 0;

// Sample average, max and peak detection
uint16_t sampleavg = 0;
uint16_t oldsample = 0;
uint16_t samplepeak = 0;

MicSample sample;

void setup() {
    gizmo.beginSetup(MIC_BUDDY, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        handleMic();
    }
}

void finishWiFiConnect() {
    Serial.printf("%s is ready\n", MIC_BUDDY);
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

    Serial.printf("255, %d, %d, %d\n", av, sampleavg, samplepeak * 200);
    oldsample = av;

    sample.sampleavg = sampleavg;
    sample.samplepeak = samplepeak;
    sample.oldsample = oldsample;

    UDP.beginPacket(dest, 7001);
    UDP.write((char *) &sample, sizeof(sample));
    UDP.endPacket();

    delay(5);
}
