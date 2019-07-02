#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2019.07.01.001"

#define MIC_PIN         A0
#define PIR_PIN          5

WiFiUDP UDP;
IPAddress dest(192,168,1,148);

#define MAX_SAMPLES     8
typedef struct {
    uint16_t count;
    uint16_t data[MAX_SAMPLES];
} Sample;

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
    sample.data[sample.count] = analogRead(MIC_PIN);
    sample.count++;

    if (sample.count == MAX_SAMPLES) {
        UDP.beginPacket(dest, 7001);
        UDP.write((char *) &sample, sizeof(sample));
        UDP.endPacket();

        for (int i = 0; i < sample.count; i++) {
            Serial.printf("200, 300, %d\n", sample.data[i]);
        }

        sample.count = 0;
    }

    delay(10);
}