//
// Created by Thomas Vachuska on 8/14/22.
//

#ifndef ARDUINO_LAMPSYNC_H
#define ARDUINO_LAMPSYNC_H

typedef struct {
    uint16_t sampleavg;
    uint16_t samplepeak;
    uint16_t oldsample;
} MicSample;

#define PEER_TIMEOUT   5000

#define MAX_PEERS   17
typedef struct {
    uint32_t ip;
    uint32_t lastHeard;
    boolean  sampling;
    char name[64];
} Peer;

Peer peers[MAX_PEERS];
uint8_t master = 0;

boolean syncWithMaster = true;
boolean buddyAvailable = false;
boolean buddySilent = true;
uint32_t buddyTimestamp = 0;
uint32_t buddyIp = 0;

#define GROUP_MASK      0x01000
#define FRONT_CTX       (0x0001 | GROUP_MASK)
#define BACK_CTX        (0x0002 | GROUP_MASK)
#define ALL_CTX         (FRONT_CTX | BACK_CTX)

#define HELLO           001

#define SAMPLE_REQ      100
#define SAMPLE_ADV      101
#define SAMPLE          102

#define SYNC_REQ        200
#define PATTERN         201
#define COLORS          202

#define POWER_ON_OFF    666

#define MAX_CMD_DATA    64
typedef struct {
    uint32_t src;
    uint16_t ctx;
    uint16_t op;
    uint8_t data[MAX_CMD_DATA];
} Command;

uint8_t channel = 0;
#define CHANNEL_MASK    0xf000
#define OPERATION_MASK  0x0fff
#define CH(co)      (co & CHANNEL_MASK) >> 12
#define OP(co)      co & OPERATION_MASK
#define CHOP(o)     (uint16_t) (o | (channel<<12))

#define GROUP_PORT  7002
WiFiUDP group;
IPAddress groupIp(224, 0, 0, 69);

#define CHANNEL    "/cfg/channel"

void saveChannel() {
    File f = SPIFFS.open(CHANNEL, "w");
    if (f) {
        f.printf("%d\n", channel);
        f.close();
    }
}

void loadChannel() {
    File f = SPIFFS.open(CHANNEL, "r");
    if (f) {
        char num[8];
        int l = f.readBytesUntil('\n', num, 8);
        num[l] = '\0';
        channel = atoi(num);
    }

    SPIFFS.remove("/cfg/ports");
}

void setupSync() {
    loadChannel();
    group.beginMulticast(WiFi.localIP(), groupIp, GROUP_PORT);
}

void broadcast(Command command) {
    group.beginPacketMulticast(groupIp, GROUP_PORT, WiFi.localIP());
    group.write((char *) &command, sizeof(command));
    group.endPacket();
}

void handleChannel() {
    ESP8266WebServer *server = gizmo.httpServer();
    char ch[8];
    if (server->hasArg("ch")) {
        strncpy(ch, server->arg("ch").c_str(), 7);
        channel = atoi(ch);
    }

    saveChannel();

    char resp[64];
    snprintf(resp, 63, "channel=%d\n", channel);
    server->send(200, "text/plain", resp);
    gizmo.scheduleRestart();
}

void addPeer(uint32_t ip, char *name) {
    int ai = 0;
    for (int i = 1; i < MAX_PEERS; i++) {
        if (ip == peers[i].ip) {
            peers[i].lastHeard = millis();
            strcpy(peers[i].name, name);
            return;
        } else if (!ai && !peers[i].ip) {
            ai = i;
        }
    }
    if (ai) {
        peers[ai].ip = ip;
        peers[ai].lastHeard = millis();
        strcpy(peers[ai].name, name);
        gizmo.debug("Peer %s discovered", IPAddress(ip).toString().c_str());
    }
}

boolean hasPotentialMaster() {
    for (int i = 1; i < MAX_PEERS; i++) {
        if (peers[i].ip && peers[0].ip > peers[i].ip) {
            return true;
        }
    }
    return false;
}

void determineMaster() {
    master = 0;
    for (int i = 1; syncWithMaster && i < MAX_PEERS; i++) {
        if (peers[i].ip && peers[master].ip > peers[i].ip) {
            master = i;
        }
    }
}

boolean isMaster(IPAddress ip) {
    return peers[master].ip == (uint32_t) ip;
}

void decodeSample(Command *cmd, MicSample *sample) {
    sample->sampleavg = ((uint16_t) cmd->data[1] << 8) | cmd->data[0];
    sample->samplepeak = ((uint16_t) cmd->data[3] << 8) | cmd->data[2];
    sample->oldsample = ((uint16_t) cmd->data[5] << 8) | cmd->data[4];
}

void encodeSample(MicSample *sample, Command *cmd) {
    cmd->data[0] = uint8_t(sample->sampleavg);
    cmd->data[1] = uint8_t(sample->sampleavg >> 8);
    cmd->data[2] = uint8_t(sample->samplepeak);
    cmd->data[3] = uint8_t(sample->samplepeak >> 8);
    cmd->data[4] = uint8_t(sample->oldsample);
    cmd->data[5] = uint8_t(sample->oldsample >> 8);
}

#endif //ARDUINO_LAMPSYNC_H
