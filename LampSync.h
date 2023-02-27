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

#define PEER_TIMEOUT   20000

#define MAX_PEERS   5
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
boolean buddySilent = false;
uint32_t buddyTimestamp = 0;
uint32_t buddyIp = 0;

boolean hadBuddyAndPeer = false;

#define GROUP_MASK      0x01000
#define FRONT_CTX       (0x0001 | GROUP_MASK)
#define BACK_CTX        (0x0002 | GROUP_MASK)
#define ALL_CTX         (FRONT_CTX | BACK_CTX)

#define HELLO           001

#define SAMPLE_REQ      100
#define SAMPLE_ADV      101

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

#define BUDDY_PORT  7001
WiFiUDP buddy;

#define PEER_PORT  7003
WiFiUDP peer;

#define GROUP_PORT  7002
WiFiUDP group;
IPAddress groupIp(239, 0, 42, 69);

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
    buddy.begin(BUDDY_PORT);
    peer.begin(PEER_PORT);
    group.beginMulticast(WiFi.localIP(), groupIp, GROUP_PORT);
}

void unicast(uint32_t ip, uint16_t port, Command command) {
    peer.beginPacket(ip, port);
    peer.write((char *) &command, sizeof(command));
    peer.endPacket();
}

void broadcast(Command command) {
    group.beginPacketMulticast(groupIp, GROUP_PORT, WiFi.localIP());
    group.write((char *) &command, sizeof(command));
    group.endPacket();

    if (buddyAvailable && buddyIp) {
        unicast(buddyIp, PEER_PORT, command);
    }

    for (int i = 1; i < MAX_PEERS; i++) {
        if (peers[i].ip && peers[i].lastHeard) {
            unicast(peers[i].ip, PEER_PORT, command);
        }
    }
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
            peers[i].lastHeard = millis() + PEER_TIMEOUT;
            return;
        } else if (!ai && !peers[i].ip) {
            ai = i;
        }
    }
    if (ai) {
        peers[ai].ip = ip;
        peers[ai].lastHeard = millis() + PEER_TIMEOUT;
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


#endif //ARDUINO_LAMPSYNC_H
