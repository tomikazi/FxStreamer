#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#include <WebSocketsServer.h>

#define MIC_BUDDY      "MicBuddy"
#define SW_UPDATE_URL   "http://iot.vachuska.com/MicBuddy.ino.bin"
#define SW_VERSION      "2020.03.05.001"

#define STATE      "/cfg/state"

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
boolean lampSampling[MAX_LAMPS];
uint8_t lampCount = 0;

typedef struct {
    uint16_t sampleavg;
    uint16_t samplepeak;
    uint16_t oldsample;
} MicSample;

#define FRONT_CTX       0x01
#define BACK_CTX        0x10
#define ALL_CTX         FRONT_CTX | BACK_CTX

#define HELLO           001
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
#define HELLO_TIMEOUT   15000

// For moving averages
uint32_t mat = 0;
uint32_t nmat = 0;

// Sample average, max and peak detection
uint16_t baseavg = 0;
uint16_t sampleavg = 0;
uint16_t oldsample = 0;
uint16_t samplepeak = 0;

uint16_t minv, maxv;
uint8_t n1, n2, peakdelta;

uint32_t nextAdvertisement = 0;
uint32_t nextSample = 0;
boolean sampling = false;

static WebSocketsServer wsServer(81);

void setup() {
    gizmo.beginSetup(MIC_BUDDY, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.setCallback(defaultMqttCallback);

    setupWebSocket();

    setupSampler();
    loadState();
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        wsServer.loop();
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

void setupWebSocket() {
    wsServer.begin();
    wsServer.onEvent(webSocketEvent);
    Serial.println("WebSocket server setup.");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected.\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected.\n", num);
            break;
        case WStype_TEXT:
            char cmd[128];
            cmd[0] = '\0';
            strncat(cmd, (char *) payload, length);
            handleWsCommand(cmd);
            break;

        default:
            Serial.printf("Unhandled message type\n");
            break;
    }
}

void handleWsCommand(char *cmd) {
    char *t = strtok(cmd, "&");
    char *m = strtok(NULL, "&");

    if (strcmp(t, "get")) {
        processCallback(t, m);
    }
    broadcastState();
}

void broadcastState() {
    char state[512];
    state[0] = '\0';
    snprintf(state, 511,
             "{\"minv\": %u,\"maxv\": %u,\"n1\": %u,\"n2\": %u,\"peakdelta\": %u,\"lamps\": %u,\"sampling\": %s, \"version\":\"" SW_VERSION "\"}",
             minv, maxv, n1, n2, peakdelta, lampCount, sampling ? "true" : "false");
    wsServer.broadcastTXT(state);
}

void processCallback(char *topic, char *value) {
    if (strstr(topic, "/minv")) {
        minv = parse(value, 0, 16, 4);
    } else if (strstr(topic, "/maxv")) {
        maxv = parse(value, 8, 255, 128);
    } else if (strstr(topic, "/n1")) {
        n1 = parse(value, 2, 8, 4);
    } else if (strstr(topic, "/n2")) {
        n2 = parse(value, 0, 4, 2);
    } else if (strstr(topic, "/peakdelta")) {
        peakdelta = parse(value, 8, 96, 16);
    } else {
        setupSampler();
    }
    saveState();
}

void handleAdvertisement() {
    if (nextAdvertisement < millis()) {
        nextAdvertisement = millis() + AD_FREQUENCY;
        advertise();
    }
}

void advertise() {
    Command ad = { .src = (uint) WiFi.localIP(), .ctx = ALL_CTX, .op = SAMPLE_ADV, .data = { [0] = 0}};
    group.beginPacketMulticast(groupIp, GROUP_PORT, WiFi.localIP());
    group.write((char *) &ad, sizeof(ad));
    group.endPacket();
}

void addSampling(uint32_t ip, boolean refreshSampling) {
    int ai = -1;
    for (int i = 0; i < MAX_LAMPS; i++) {
        if (ip == lampIps[i]) {
            lampTimes[i] = millis() + HELLO_TIMEOUT;
            lampSampling[i] = refreshSampling ? true : lampSampling[i];
            return;
        }
        if (ai < 0 && !lampIps[i]) {
            ai = i;
        }
    }
    if (ai >= 0) {
        lampIps[ai] = ip;
        lampTimes[ai] = millis() + HELLO_TIMEOUT;
        lampSampling[ai] = refreshSampling ? true : lampSampling[ai];
        advertise();
        Serial.printf("Started sampling for %s\n", IPAddress(ip).toString().c_str());
    }
}

void removeSampling(uint32_t  ip) {
    for (int i = 0; i < MAX_LAMPS; i++) {
        if (ip == lampIps[i]) {
            lampSampling[i] = false;
            Serial.printf("Stopped sampling for %s\n", IPAddress(ip).toString().c_str());
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
            case HELLO:
                addSampling(command.src, false);
                break;
            case SAMPLE_REQ:
                if (command.data[0]) {
                    addSampling(command.src, true);
                } else {
                    removeSampling(command.src);
                }
                break;
            default:
                break;
        }
    }
}

void setupSampler() {
    minv = 4;
    maxv = 128;
    n1 = 4;
    n2 = 2;
    peakdelta = 16;
}

uint16_t parse(char * v, uint16_t min, uint16_t max, uint16 d) {
    uint16_t n = atoi(v);
    return min <= n && n <= max ? n : d;
}

void loadState() {
    File f = SPIFFS.open(STATE, "r");
    if (f) {
        char field[32];
        int l = f.readBytesUntil('|', field, 7);
        field[l] = '\0';
        minv = parse(field, 0, 16, 4);

        l = f.readBytesUntil('|', field, 7);
        field[l] = '\0';
        maxv = parse(field, 8, 255, 128);

        l = f.readBytesUntil('|', field, 7);
        field[l] = '\0';
        n1 = parse(field, 2, 8, 4);

        l = f.readBytesUntil('|', field, 7);
        field[l] = '\0';
        n2 = parse(field, 0, 4, 2);

        l = f.readBytesUntil('|', field, 7);
        field[l] = '\0';
        peakdelta = parse(field, 8, 96, 16);
        f.close();
    }
}

void saveState() {
    File f = SPIFFS.open(STATE, "w");
    if (f) {
        f.printf("%u|%u|%u|%u|%u\n", minv, maxv, n1, n2, peakdelta);
        f.close();
    }
}


void handleMic() {
    if (nextSample < millis()) {
        nextSample = millis() + 5;

        uint16_t v = analogRead(MIC_PIN);

        mat = mat + v - (mat >> n1);
        baseavg = mat >> n1;
        uint16_t av = abs(v - baseavg);

        if (av < minv) {
            av = 0;
        } else if (av > maxv) {
            av = maxv;
        }

        av = map(av, 0, maxv, 0, 255);

        // Allow n2 to be 0, which means sampleavg == av;
        if (n2) {
            nmat = nmat + av - (nmat >> n2);
            sampleavg = nmat >> n2;
        } else {
            nmat = av;
            sampleavg = av;
        }

        // We're on the down swing, so we just peaked.
        samplepeak = av > (sampleavg + peakdelta) && (av < oldsample);
        oldsample = av;

        MicSample sample;
        sample.sampleavg = sampleavg;
        sample.samplepeak = samplepeak;
        sample.oldsample = oldsample;

        uint32_t now = millis();
        uint8_t count = 0;
        boolean active = false;
        for (int i = 0; i < MAX_LAMPS; i++) {
            if (lampIps[i] && lampTimes[i] >= now && lampSampling[i]) {
                buddy.beginPacket(IPAddress(lampIps[i]), BUDDY_PORT);
                buddy.write((char *) &sample, sizeof(sample));
                buddy.endPacket();
                active = true;
                count++;
            } else if (lampTimes[i] && lampTimes[i] < now) {
                Serial.printf("Lamp %s timed out\n", IPAddress(lampIps[i]).toString().c_str());
                lampIps[i] = 0;
                lampTimes[i] = 0;
            } else {
                count++;
            }
        }
        lampCount = count;
        sampling = active;

        if (sampling) {
            Serial.printf("255, %d, %d, %d, %d, %d\n", av, sampleavg, baseavg, samplepeak * 200, v);
        }
    }
}
