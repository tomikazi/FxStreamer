#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FS.h>

#include <WebSocketsServer.h>

#define FX_STREAMER     ""
#define SW_UPDATE_URL   "http://iot.vachuska.com/FxStreamer.ino.bin"
#define SW_VERSION      "2020.03.08.001"

#define STATE      "/cfg/state"

#define MIC_PIN         A0
#define PIR_PIN          5

#define BUDDY_PORT  7001
WiFiUDP buddy;

#define PEER_PORT  7003
WiFiUDP peer;

#define GROUP_PORT  7002
WiFiUDP group;
IPAddress groupIp(224, 0, 42, 69);

#define MAX_PEERS   4
typedef struct {
    uint32_t ip;
    uint32_t lastHeard;
    boolean  sampling;
    char name[64];
} Peer;

Peer peers[MAX_PEERS];
uint8_t peerCount = 0;

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
#define HELLO_TIMEOUT   20000

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
    gizmo.beginSetup(FX_STREAMER, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);
    gizmo.setCallback(defaultMqttCallback);
//    gizmo.debugEnabled = true;

    setupWebSocket();

    setupSampler();
    loadState();
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        wsServer.loop();
        handleClients();
        handleMic();
        handleAdvertisement();
    }
}

void finishWiFiConnect() {
    buddy.begin(BUDDY_PORT);
    peer.begin(PEER_PORT);
    group.beginMulticast(WiFi.localIP(), groupIp, GROUP_PORT);
    advertise();
    Serial.printf("%s is ready\n", FX_STREAMER);
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

#define STATUS \
    "{\"minv\": %u,\"maxv\": %u,\"n1\": %u,\"n2\": %u,\"peakdelta\": %u," \
    "\"lamps\": %u,\"sampling\": %s,\"name\": \"%s\",\"version\":\"" SW_VERSION "\"}"


void broadcastState() {
    char state[512];
    state[0] = '\0';
    snprintf(state, 511, STATUS, minv, maxv, n1, n2, peakdelta,
            peerCount, sampling ? "true" : "false", gizmo.getHostname());
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
        gizmo.debug("peerCount=%d sampling=%d", peerCount, sampling);
    }
}

void advertise() {
    Command ad = { .src = (uint) WiFi.localIP(), .ctx = ALL_CTX, .op = SAMPLE_ADV, .data = { [0] = 0}};
    group.beginPacketMulticast(groupIp, GROUP_PORT, WiFi.localIP());
    group.write((char *) &ad, sizeof(ad));
    group.endPacket();

    for (int i = 0; i < MAX_PEERS; i++) {
        if (peers[i].ip && peers[i].lastHeard) {
            peer.beginPacket(peers[i].ip, PEER_PORT);
            peer.write((char *) &ad, sizeof(ad));
            peer.endPacket();
        }
    }
}

void addSampling(uint32_t ip, boolean refreshSampling) {
    int ai = -1;
    for (int i = 0; i < MAX_PEERS; i++) {
        if (ip == peers[i].ip) {
            peers[i].lastHeard = millis() + HELLO_TIMEOUT;
            peers[i].sampling = refreshSampling ? true : peers[i].sampling;
            return;
        }
        if (ai < 0 && !peers[i].ip) {
            ai = i;
        }
    }
    if (ai >= 0) {
        peers[ai].ip = ip;
        peers[ai].lastHeard = millis() + HELLO_TIMEOUT;
        peers[ai].sampling = refreshSampling ? true : peers[ai].sampling;
        advertise();
        gizmo.debug("Started sampling for %s", IPAddress(ip).toString().c_str());
    }
}

void removeSampling(uint32_t  ip) {
    for (int i = 0; i < MAX_PEERS; i++) {
        if (ip == peers[i].ip) {
            peers[i].sampling = false;
            gizmo.debug("Stopped sampling for %s", IPAddress(ip).toString().c_str());
            return;
        }
    }
}


void handleClients() {
    Command command;
    while (group.parsePacket()) {
        int len = group.read((char *) &command, sizeof(command));
        if (len < 0) {
            gizmo.debug("Unable to read command!!!!");
        } else {
            handleClient(command);
        }
    }

    while (peer.parsePacket()) {
        int len = peer.read((char *) &command, sizeof(command));
        if (len < 0) {
            gizmo.debug("Unable to read command!!!!");
        } else {
            handleClient(command);
        }
    }
}

void handleClient(Command command) {
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
        for (int i = 0; i < MAX_PEERS; i++) {
            if (peers[i].ip && peers[i].lastHeard >= now && peers[i].sampling) {
                buddy.beginPacket(IPAddress(peers[i].ip), BUDDY_PORT);
                buddy.write((char *) &sample, sizeof(sample));
                buddy.endPacket();
                active = true;
            } else if (peers[i].lastHeard && peers[i].lastHeard < now) {
                gizmo.debug("Lamp %s timed out", IPAddress(peers[i].ip).toString().c_str());
                peers[i].ip = 0;
                peers[i].lastHeard = 0;
                peers[i].lastHeard = 0;
                peers[i].sampling = 0;
            }

            if (peers[i].ip) {
                count++;
            }
        }
        peerCount = count;
        sampling = active;

        if (sampling) {
            Serial.printf("255, %d, %d, %d, %d, %d, 470\n", av, sampleavg, baseavg, samplepeak * 200, v);
        }
    }
}
