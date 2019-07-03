#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2019.07.01.001"

#define MIC_PIN         A0
#define PIR_PIN          5

WiFiUDP UDP;

// TODO: Make configurable
IPAddress dest(192,168,1,148);

#define MAX_SAMPLES     8
typedef struct {
    uint16_t count;
    uint16_t data[MAX_SAMPLES];
} Sample;

// For moving average for the N most recent samples used to normalize the input signal.
#define N   256
uint32_t mat = 600 * N;
uint16_t cutoff = 10;

Sample sample;

void setup() {
    gizmo.beginSetup(MIC_BUDDY, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.endSetup();
    sample.count = 0;
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

    sample.data[sample.count] = av > cutoff ? av : 0;
    sample.count++;

    if (sample.count == MAX_SAMPLES) {
        UDP.beginPacket(dest, 7001);
        UDP.write((char *) &sample, sizeof(sample));
        UDP.endPacket();

        for (int i = 0; i < sample.count; i++) {
            Serial.printf("100, %d\n", sample.data[i]);
        }

        sample.count = 0;
    }

    delay(10);
}