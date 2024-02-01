#include <ESPGizmoDefault.h>
#include <WiFiUDP.h>
#include <FastLED.h>
#include <FS.h>

#include <WebSocketsServer.h>
#include "LampSync.h"

#define FX_STREAMER     "FxStreamer"
#define SW_UPDATE_URL   "http://iot.vachuska.com/FxStreamer.ino.bin"
#define SW_VERSION      "2024.01.31.015"

#define STATE      "/cfg/state"

#define MIC_PIN         A0
#define BTN_PIN         D2
#define IND_PIN         D1

CRGB indicator[1];

uint8_t peerCount = 0;
uint8_t streamCount = 0;

#define SAMPLE_FREQUENCY   10
#define AD_FREQUENCY     3000

// For moving averages
uint32_t mat = 0;
uint32_t nmat = 0;
uint32_t smat = 0;

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
boolean wasSilenceDetected = false;
boolean silenceDetected = true;
boolean silenceMandated = false;

static WebSocketsServer wsServer(81);

void setup() {
    gizmo.useMulticast = true;
    gizmo.beginSetup(FX_STREAMER, SW_VERSION, "gizmo123");
    gizmo.setUpdateURL(SW_UPDATE_URL);

    gizmo.httpServer()->on("/channel", handleChannel);
    gizmo.setupWebRoot();
    setupWebSocket();

    gizmo.setCallback(defaultMqttCallback);

    setupSampler();
    setupButtonAndIndicator();
    loadState();
    gizmo.endSetup();
}

void loop() {
    if (gizmo.isNetworkAvailable(finishWiFiConnect)) {
        handleClients();
        handleMic();
        handleButton();

        EVERY_N_MILLIS(10)
        {
            handleIndicator();
        }

        handleAdvertisement();
    }
    wsServer.loop();
}

void finishWiFiConnect() {
    setupSync();
    advertise();
    Serial.printf("%s is ready\n", FX_STREAMER);
}

void setupButtonAndIndicator() {
    pinMode(BTN_PIN, INPUT_PULLUP);
    FastLED.addLeds<WS2812B, IND_PIN, GRB>(indicator, 1);  // GRB ordering is typical
    indicator[0] = CRGB::Yellow;
    FastLED.setBrightness(32);
    FastLED.show();
}


#define SHORT_PRESS_TIME  30
#define HOLD_DOWN_TIME  1500
#define INDICATION_TIME 2000

static uint8_t lastState = HIGH;
static uint32_t pressedTime = 0;
static uint32_t modeSwitchTime = 0;

void handleButton() {
    // read the state of the switch/button:
    uint32_t currentState = digitalRead(BTN_PIN);
    if (lastState == HIGH && currentState == LOW) {       // button is pressed
        pressedTime = millis();
    } else if (pressedTime && lastState == LOW && currentState == HIGH) { // button is released
        long pressDuration = millis() - pressedTime;
        if (pressDuration > SHORT_PRESS_TIME) {
            silenceMandated = !silenceMandated;
            modeSwitchTime = millis() + INDICATION_TIME;
            broadcastState();
        }
        pressedTime = 0;

    } else if (pressedTime && currentState == LOW && (millis() - pressedTime) > HOLD_DOWN_TIME) {
        Serial.println("Power on/off hold-down press detected");
        sendPowerOff();
        pressedTime = 0;
    }

    // save the the last state
    lastState = currentState;
}

void handleIndicator() {
    if (pressedTime) {
        indicator[0] = CRGB::White;
    } else if (modeSwitchTime > millis()) {
        indicator[0] = silenceMandated ? CRGB::Red : CRGB::Blue;
    } else if (silenceDetected) {
        indicator[0] = CRGB::Black;
    } else if (silenceMandated) {
        indicator[0] = CRGB::Red;
    } else {
        indicator[0] = sampling ? CRGB::Green : CRGB::Blue;
    }
    FastLED.show();
}

void sendPowerOff() {
    Command cmd = {.src = (uint32_t) WiFi.localIP(), .ctx = ALL_CTX, .op = CHOP(POWER_ON_OFF), .data = {[0] = uint8_t(
            millis() % 256)}};
    for (int i = 0; i < 16; i++) {
        broadcast(cmd);
    }
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
    "{\"minv\": %u,\"maxv\": %u,\"n1\": %u,\"n2\": %u,\"peakdelta\": %u,\"silence\": %s," \
    "\"lamps\": %u,\"streams\": %u,\"sampling\": %s,\"name\": \"%s\",\"version\":\"" SW_VERSION "\"}"


void broadcastState() {
    char state[512];
    state[0] = '\0';
    snprintf(state, 511, STATUS, minv, maxv, n1, n2, peakdelta, silenceDetected || silenceMandated ? "true" : "false",
             peerCount, streamCount, sampling ? "true" : "false", gizmo.getHostname());
    wsServer.broadcastTXT(state);
}

void processCallback(char *topic, char *value) {
    if (strstr(topic, "/minv")) {
        minv = parse(value, 0, 16, 4);
    } else if (strstr(topic, "/maxv")) {
        maxv = parse(value, 4, 255, 128);
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
    Command ad = {.src = (uint) WiFi.localIP(), .ctx = ALL_CTX, .op = CHOP(
            SAMPLE_ADV), .data = {[0] = silenceDetected || silenceMandated}};
    group.beginPacketMulticast(groupIp, GROUP_PORT, WiFi.localIP());
    group.write((char *) &ad, sizeof(ad));
    group.endPacket();
}

void addSampling(uint32_t ip, boolean refreshSampling) {
    int ai = -1;
    for (int i = 0; i < MAX_PEERS; i++) {
        if (ip == peers[i].ip) {
            peers[i].lastHeard = millis();
            peers[i].sampling = refreshSampling ? true : peers[i].sampling;
            return;
        } else if (!peers[i].ip) {
            ai = i;
        }
    }

    if (ai >= 0) {
        peers[ai].ip = ip;
        peers[ai].lastHeard = millis();
        peers[ai].sampling = refreshSampling;
        advertise();
        broadcastState();
    }
}

void removeSampling(uint32_t ip) {
    for (int i = 0; i < MAX_PEERS; i++) {
        if (ip == peers[i].ip && peers[i].sampling) {
            peers[i].sampling = false;
            broadcastState();
            return;
        }
    }
}


void handleClients() {
    EVERY_N_SECONDS(1)
    {
        prunePeers();
    }

    Command command;
    while (group.parsePacket()) {
        int len = group.read((char *) &command, sizeof(command));
        if (len < 0) {
            gizmo.debug("Unable to read command!!!!");
        } else if (command.src != WiFi.localIP()) {
            handleClient(command);
        }
    }
}

void handleClient(Command command) {
    uint8_t ch = CH(command.op);
    uint16_t op = OP(command.op);

    if (ch == channel) {
        switch (op) {
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
    maxv = 32;
    n1 = 4;
    n2 = 2;
    peakdelta = 16;
}

uint16_t parse(char *v, uint16_t min, uint16_t max, uint16 d) {
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
        maxv = parse(field, 4, 255, 128);

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

void prunePeers() {
    uint32_t now = millis();
    uint8_t newPeerCount = 0;
    uint8_t newStreamCount = 0;

    for (int i = 0; i < MAX_PEERS; i++) {
        if (peers[i].ip && peers[i].lastHeard && peers[i].lastHeard + PEER_TIMEOUT < now) {
            gizmo.debug("Lamp %s timed out", IPAddress(peers[i].ip).toString().c_str());
            peers[i].ip = 0;
            peers[i].lastHeard = 0;
            peers[i].sampling = 0;
        }

        if (peers[i].ip) {
            newPeerCount++;
            if (!silenceMandated && peers[i].sampling) {
                newStreamCount++;
            }
        }
    }

    uint8_t oldPeerCount = peerCount;
    uint8_t oldStreamCount = streamCount;
    boolean wasSampling = sampling;

    peerCount = newPeerCount;
    streamCount = newStreamCount;
    sampling = newStreamCount > 0;

    if (sampling != wasSampling || silenceDetected != wasSilenceDetected ||
        streamCount != oldStreamCount || peerCount != oldPeerCount) {
        broadcastState();
    }
}

void handleMic() {
    if (nextSample < millis()) {
        nextSample = millis() + SAMPLE_FREQUENCY;

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

        wasSilenceDetected = silenceDetected;

        smat = smat + av - (smat >> 11);
        silenceDetected = (smat >> 11) < (3 * minv);

        // We're on the down swing, so we just peaked.
        samplepeak = av > (sampleavg + peakdelta) && (av < oldsample);
        oldsample = av;

        // If we're supposed to be sampling, encode and send the sample... also log it.
        if (sampling) {
            EVERY_N_MILLIS(50)
            {
                MicSample sample;
                sample.sampleavg = sampleavg;
                sample.samplepeak = samplepeak;
                sample.oldsample = oldsample;

                Command cmd = {.src = (uint32_t) WiFi.localIP(), .ctx = ALL_CTX, .op = CHOP(SAMPLE), .data = {[0] = 0}};
                encodeSample(&sample, &cmd);
                broadcast(cmd);
            }

            Serial.printf("255, %d, %d, %d, %d, %d, 470\n", av, sampleavg, baseavg, samplepeak * 200, v);
        }
    }
}
