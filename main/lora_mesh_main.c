/* 

*/
#ifdef SIMULATION
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "../simulator/routers/base.h"
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_spi_flash.h"
#include "lora.h"
#include "softap.h"
#include "serial.h"
#include "config.h"
#endif

#define TAG "LORA32"
#define HEADER_LENGTH 16
#define SHA1_LENGTH 40
#define ADDR_LENGTH 6 
#define MAX_ROUTES_PER_PACKET 24
#define MAX_PACKET_BUFFER 10
//#define DEBUG 1

//char macaddr[ADDR_LENGTH*2];
#ifndef SIMULATION
uint8_t mac[ADDR_LENGTH];
#endif
uint8_t protocol = 0xAA;
uint8_t proto_address = 0xAA;
//char* nodeID;

// global sequence number
uint8_t messageCount = 0;

// mode switches
int retransmitEnabled = 0;
int hashingEnabled = 0;

// timeout intervals
int _helloInterval = 10;
int _routeInterval = 10;
int _messageInterval = 5;
int _discoveryTimeout = 30;
int _learningTimeout = 200;
int _maxRandomDelay = 20;
#ifndef SIMULATION
floar timeDistortion = 1;
#endif
long startTime;

int simulationTime(int realTime) {
    return realTime * timeDistortion;
}
int helloInterval() {
    return simulationTime(_helloInterval);
}
int routeInterval() {
    return simulationTime(_routeInterval);
}
int messageInterval() {
    return simulationTime(_messageInterval);
}
int discoveryTimeout() {
    return simulationTime(_discoveryTimeout);
}
int learningTimeout() {
    return simulationTime(_learningTimeout);
}
int maxRandomDelay() {
    return simulationTime(_maxRandomDelay);
}

// metric variables
float packetSuccessWeight = .8;
float RSSIWeight = .3;
float SNRWeight = .3;
float randomMetricWeight = .2;

// packet structures
struct Metadata {
    int rssi;
    float snr;
    uint8_t randomness;
};

struct Packet {
    uint8_t ttl;
    uint8_t totalLength;
    uint8_t source[ADDR_LENGTH];
    uint8_t destination[ADDR_LENGTH];
    uint8_t sequence;
    uint8_t type;
    uint8_t data[240];
};

struct Packet buffer[MAX_PACKET_BUFFER];
int bufferEntry = 0;

struct NeighborTableEntry{
    uint8_t address[ADDR_LENGTH];
    uint8_t lastReceived;
    uint8_t packet_success;
    uint8_t metric;
    uint8_t protocol;
    uint8_t proto_address;
};
struct NeighborTableEntry neighborTable[255];
int neighborEntry = 0;

struct RoutingTableEntry{
    uint8_t destination[ADDR_LENGTH];
    uint8_t nextHop[ADDR_LENGTH];
    uint8_t distance;
    uint8_t lastReceived;
    uint8_t metric;
    uint8_t protocol;
    uint8_t proto_address;
};
struct RoutingTableEntry routeTable[255];
int routeEntry = 0;




struct Packet buildPacket( uint8_t ttl, uint8_t src[6], uint8_t dest[6], uint8_t sequence, uint8_t type, uint8_t data[240], uint8_t dataLength){

    uint8_t packetLength = HEADER_LENGTH + dataLength;
    uint8_t* buffer = (uint8_t*)  malloc(dataLength);
    buffer = (uint8_t*) data;
    struct Packet packet;
    
    packet.ttl = ttl;
    packet.totalLength = packetLength;
    packet.sequence = sequence;
    packet.type = type;
    memcpy(&packet.data, buffer, dataLength);
    memcpy(&packet.source, src, ADDR_LENGTH);
    memcpy(&packet.destination, dest, ADDR_LENGTH);
    
    //printPacketBuild(packet);
  
    return packet;
}

#ifndef SIMULATION
int send_packet(uint8_t* data, int len){

    Serial.printf("Sending:\r\n ");
    lora32_send(&lora,data,len);
    return 0;
}
#endif



int sendPacket(struct Packet packet) {


    uint8_t* sending = (uint8_t*) malloc(sizeof(packet));
    memcpy(sending, &packet, sizeof(packet));
   
    send_packet(sending, packet.totalLength);
    messageCount++;

    //Serial.printf("SendPacket: ");
    for(int i = 0 ; i < packet.totalLength ; i++){
       // Serial.printf("%02x", sending[i]);
    }
    //Serial.printf("\r\n");

    return messageCount;
  
}


void pushToBuffer(struct Packet packet){

    if(bufferEntry > 7){
        bufferEntry = 0;
    }

    memset(&buffer[bufferEntry], 0, sizeof(buffer[bufferEntry]));
    memcpy(&buffer[bufferEntry], &packet, sizeof(buffer[bufferEntry]));
    bufferEntry++;
}

struct Packet popFromBuffer(){

    bufferEntry--;
    struct Packet pop;
    memcpy(&pop, &buffer[bufferEntry], sizeof(pop));
    return pop;
}

void checkBuffer(){

    if (bufferEntry > 0){
        struct Packet packet = popFromBuffer();
        sendPacket(packet);
    }
    //else buffer is empty;
}

int searchDestination(uint8_t protocol, uint8_t proto_address){

    int entry = -1;
    for( int i = 0 ; i < routeEntry ; i++){
        if(memcmp(&routeTable[i].protocol, &protocol, sizeof(protocol)) == 0){
            if(memcmp(&routeTable[i].proto_address, &proto_address, sizeof(proto_address)) == 0){
                entry = i;
                return entry;
            }
        }
    }
    return entry;
}


#ifndef SIMULATION
void  parserSerialToMeshSend(struct Serial_Packet dataFromSerial)
{
    struct Packet dataToMesh;
    uint8_t destination[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    uint8_t ttl = 255;

    int entry = searchDestination(dataFromSerial.protocol, dataFromSerial.proto_address);
    if(entry != -1){
        memcpy(destination, &routeTable[entry].destination, sizeof(destination));
        ttl = 255;
    }

    if(dataFromSerial.dataLength < 240){
        dataToMesh = buildPacket(ttl, mac, destination, messageCount, dataFromSerial.protocol, dataFromSerial.data, dataFromSerial.dataLength); 
        sendPacket(dataToMesh);
    }else{
        uint8_t dataTemp[240];
        memcpy(dataTemp,dataFromSerial.data, 240);
        dataToMesh = buildPacket(ttl, mac, destination, messageCount, dataFromSerial.protocol, dataTemp, 240); 
        sendPacket(dataToMesh);

        memcpy(dataTemp,dataFromSerial.data+240, dataFromSerial.dataLength-240);
        dataToMesh = buildPacket(ttl, mac, destination, messageCount, dataFromSerial.protocol, dataTemp, dataFromSerial.dataLength-240); 
        sendPacket(dataToMesh);

    }

    
}


void checkSendBuffer(){
    struct Serial_Packet dataTemp;
    
    //Serial.printf("checksendBuffer\r\n");

    dataTemp = serial_read();

    if (dataTemp.dataLength){
        parserSerialToMeshSend(dataTemp);
       // Serial.printf("Send buffer enviou \r\n");
    }else{
        //Serial.printf("Send buffer is empty\r\n");
    } 
}

#endif

void printPacketBuild(struct Packet packet){

    Serial.printf("\n");
    Serial.printf("Packet build: \n");
    Serial.printf("ttl: %d\n", packet.ttl);
    Serial.printf("length: %d\n", packet.totalLength);
    Serial.printf("source: ");
    for(int i = 0 ; i < ADDR_LENGTH ; i++){
        Serial.printf("%02x", packet.source[i]);
    }
    Serial.printf("\n");
    Serial.printf("destination: ");
    for(int i = 0 ; i < ADDR_LENGTH ; i++){
        Serial.printf("%02x", packet.destination[i]);
    }
    Serial.printf("\n");
    Serial.printf("sequence: %02x\n", packet.sequence);
    Serial.printf("type: %c\n", packet.type);
    Serial.printf("data: ");
    for(int i = 0 ; i < packet.totalLength-HEADER_LENGTH ; i++){
        Serial.printf("%02x",packet.data[i]);
    }
    Serial.printf("\n");
}



void printPacketInfo(struct Packet packet, struct Metadata metadata){

    Serial.printf("\n");
    Serial.printf("Packet Received: \n");
    Serial.printf("RSSI: %d\n", metadata.rssi);
    Serial.printf("SNR: %f\n", metadata.snr);
    Serial.printf("ttl: %d\n", packet.ttl);
    Serial.printf("length: %d\n", packet.totalLength);
    Serial.printf("source: ");
    for(int i = 0 ; i < ADDR_LENGTH ; i++){
        Serial.printf("%02x", packet.source[i]);
    }
    Serial.printf("\n");
    Serial.printf("destination: ");
    for(int i = 0 ; i < ADDR_LENGTH ; i++){
        Serial.printf("%02x", packet.destination[i]);
    }
    Serial.printf("\n");
    Serial.printf("sequence: %02x\n", packet.sequence);
    Serial.printf("type: %c\n", packet.type);
    Serial.printf("data: ");
    for(int i = 0 ; i < packet.totalLength-HEADER_LENGTH ; i++){
        Serial.printf("%02x",packet.data[i]);
    }
    Serial.printf("\n");
}

void printNeighborTable(){

    Serial.printf("\n");
    Serial.printf("Neighbor Table:\n");
    for( int i = 0 ; i < neighborEntry ; i++){
        for(int j = 0 ; j < ADDR_LENGTH ; j++){
            Serial.printf("%02x", neighborTable[i].address[j]);
        }
        Serial.printf(" last rec. %3d ", neighborTable[i].lastReceived);
        Serial.printf(" metric %3d ", neighborTable[i].metric);
        Serial.printf(" proto. %02X ", neighborTable[i].protocol);
        Serial.printf(" proto addr %02X ", neighborTable[i].proto_address);
        Serial.printf("\n");
    }
    Serial.printf("\n");
}

void printRoutingTable(){

    Serial.printf("\n");
    Serial.printf("Routing Table: total routes %d\n", routeEntry);
    for( int i = 0 ; i < routeEntry ; i++){
        Serial.printf("%d hops from ", routeTable[i].distance);
        for(int j = 0 ; j < ADDR_LENGTH ; j++){
            Serial.printf("%02x", routeTable[i].destination[j]);
        }
        Serial.printf(" via ");
        for(int j = 0 ; j < ADDR_LENGTH ; j++){
            Serial.printf("%02x", routeTable[i].nextHop[j]);
        }
        Serial.printf(" last rec. %3d ", routeTable[i].lastReceived);
        Serial.printf(" metric %3d ", routeTable[i].metric);
        Serial.printf(" protocol %02X ", routeTable[i].protocol);
        Serial.printf(" prot. address %02X ", routeTable[i].proto_address);
        Serial.printf("\n");
    }
    Serial.printf("\n");
}


void printAddress(uint8_t address[ADDR_LENGTH]){
    for( int i = 0 ; i < ADDR_LENGTH; i++){
        Serial.printf("%02x", address[i]);
    }
}

uint8_t calculatePacketLoss(int entry, uint8_t sequence){

    uint8_t packet_loss = 0XFF;
    uint8_t sequence_diff = sequence - neighborTable[entry].lastReceived;
    if(sequence_diff == 0){
        // this is first packet received from neighbor
        // assume perfect packet success
        neighborTable[entry].packet_success = 0xFF; 
        packet_loss = 0x00; 
    }else if(sequence_diff == 1){
        // do not decrease packet success rate
        packet_loss = 0x00; 
    }else if(sequence_diff > 1 && sequence_diff < 16){
        // decrease packet success rate by difference
        packet_loss = 0x10 * sequence_diff; 
    }else if(sequence_diff > 16){
        // no packet received recently
        // assume complete pakcet loss
        packet_loss = 0xFF; 
    }
    return packet_loss;
}

uint8_t calculateMetric(int entry, uint8_t sequence, struct Metadata metadata){

    float weightedPacketSuccess =  ((float) neighborTable[entry].packet_success)*packetSuccessWeight;
    float weightedRandomness =  ((float) metadata.randomness)*randomMetricWeight;
    float weightedRSSI =  ((float) metadata.rssi)*RSSIWeight;
    float weightedSNR =  ((float) metadata.snr)*SNRWeight;
    uint8_t metric = weightedPacketSuccess+weightedRandomness;
    //Serial.printf("weighted packet success: %3f \r\n", weightedPacketSuccess);
    //Serial.printf("weighted randomness: %3f \r\n", weightedRandomness);
    //Serial.printf("weighted RSSI: %3f \r\n", weightedRSSI);
    //Serial.printf("weighted SNR: %3f \r\n", weightedSNR);
    //Serial.printf("metric calculated: %3d \r\n", metric);
    return metric;
}

int checkNeighborTable(struct NeighborTableEntry neighbor){

    int entry = routeEntry;
    for( int i = 0 ; i < neighborEntry ; i++){
        //had to use memcmp instead of strcmp?
        if(memcmp(neighbor.address, neighborTable[i].address, sizeof(neighbor.address)) == 0){
            entry = i; 
            return entry; /////
        }
    }
    return entry;
}

int checkRoutingTable(struct RoutingTableEntry route){

    int entry = routeEntry; // assume this is a new route
    for( int i = 0 ; i < routeEntry ; i++){
        if(memcmp(route.destination, mac, sizeof(route.destination)) == 0){
            //this is me don't add to routing table 
            //debug_printf("this route is my local address\n");
            entry = -1;
            return entry;
        }else 
        if(memcmp(route.destination, routeTable[i].destination, sizeof(route.destination)) == 0){
            if(memcmp(route.nextHop, routeTable[i].nextHop, sizeof(route.nextHop)) == 0){
                // already have this exact route, update metric
                entry = i; 
                return entry;
            }else{
                // already have this destination, but via a different neighbor
                if(route.distance < routeTable[i].distance){
                    // replace route if distance is better 
                    entry = i;
                }else 
                if(route.distance == routeTable[i].distance){
                    if(route.metric > routeTable[i].metric){
                    // replace route if distance is equal and metric is better 
                        entry = i;
                    }else{
                        entry = -1;
                    }
                }else{
                    // ignore route if distance and metric are worse
                    entry = -1;
                }
                return entry;
            }
        } 
    }
    return entry;
}

int updateNeighborTable(struct NeighborTableEntry neighbor, int entry){

    memset(&neighborTable[entry], 0, sizeof(neighborTable[entry]));
    memcpy(&neighborTable[entry], &neighbor, sizeof(neighborTable[entry]));
    if(entry == neighborEntry){
        neighborEntry++;
        //Serial.printf("new neighbor found: \r\n");
    }else{
       // Serial.printf("neighbor updated! \r\n");
    }
    return entry;
}

int updateRouteTable(struct RoutingTableEntry route, int entry){

    memset(&routeTable[entry], 0, sizeof(routeTable[entry]));
    memcpy(&routeTable[entry], &route, sizeof(routeTable[entry]));
    if(entry == routeEntry){
        routeEntry++;
       // Serial.printf("new route found! \r\n ");
    }else{
        //Serial.printf("route updated! \r\n");
    }
    if(DEBUG){
        printAddress(routeTable[entry].destination);
    }
    //Serial.printf("\n");
    return entry;
}

int selectRoute(struct Packet packet){

    int entry = -1;
    for( int i = 0 ; i < routeEntry ; i++){
        if(memcmp(packet.destination, routeTable[i].destination, sizeof(packet.destination)) == 0){
            entry = i;
        }
    }
    return entry;
}

void retransmitRoutedPacket(struct Packet packet, struct RoutingTableEntry route){

    // decrement ttl
    packet.ttl--;
    Serial.printf("node : retransmitting\r\n");
    uint8_t data[240];
    int dataLength = 0;
    for( int i = 0 ; i < ADDR_LENGTH ; i++){
        data[dataLength] = route.nextHop[i];
        dataLength++;
    }
    struct Packet newMessage = buildPacket(packet.ttl, packet.source, packet.destination, packet.sequence, packet.type, data, packet.totalLength-HEADER_LENGTH); 

    // queue packet to be transmitted
    pushToBuffer(newMessage);//TODO Ver com fazer
}


int parseHelloPacket(struct Packet packet, struct Metadata metadata){

    struct NeighborTableEntry neighbor;
    memcpy(neighbor.address, packet.source, sizeof(neighbor.address));
    int n_entry = checkNeighborTable(neighbor);
    neighbor.lastReceived = packet.sequence;
    uint8_t packet_loss = calculatePacketLoss(n_entry, packet.sequence);
    neighbor.packet_success = neighborTable[n_entry].packet_success - packet_loss;
    uint8_t metric = calculateMetric(n_entry, packet.sequence, metadata); 
    neighbor.metric = metric;
    neighbor.protocol = packet.data[0];
    neighbor.proto_address = packet.data[1];
    updateNeighborTable(neighbor, n_entry);  

    struct RoutingTableEntry route;
    memcpy(route.destination, packet.source, ADDR_LENGTH);
    memcpy(route.nextHop, packet.source, ADDR_LENGTH);
    route.distance = 1;
    route.metric = neighbor.metric;
    route.protocol = neighbor.protocol;
    route.proto_address = neighbor.proto_address;
    int r_entry = checkRoutingTable(route);
    if(r_entry == -1){
        //Serial.printf("do nothing, already have better route to ");
        if(DEBUG){
            printAddress(route.destination);
        }
        //Serial.printf("\n");
    }else{
        updateRouteTable(route, r_entry);
    }
    return n_entry;
}

int parseRoutingPacket(struct Packet packet, struct Metadata metadata){
    int numberOfRoutes = (packet.totalLength - HEADER_LENGTH) / (ADDR_LENGTH+4);
    //Serial.printf("routes in packet: %d\r\n", numberOfRoutes);

    //int n_entry = parseHelloPacket(packet, metadata);

    struct NeighborTableEntry neighbor;
    memcpy(neighbor.address, packet.source, sizeof(neighbor.address));
    int n_entry = checkNeighborTable(neighbor);

    for( int i = 0 ; i < numberOfRoutes ; i++ ){
        struct RoutingTableEntry route; 
        memcpy(route.destination, packet.data + (ADDR_LENGTH+4)*i, ADDR_LENGTH);
        memcpy(route.nextHop, packet.source, ADDR_LENGTH);
        route.distance = packet.data[(ADDR_LENGTH+4)*i + ADDR_LENGTH]; 
        route.distance++; // add a hop to distance
        float metric = (float) packet.data[(ADDR_LENGTH+4)*i + ADDR_LENGTH+1];
        route.protocol = packet.data[(ADDR_LENGTH+4)*i + ADDR_LENGTH+2];
        route.proto_address = packet.data[(ADDR_LENGTH+4)*i + ADDR_LENGTH+3];
        //printf( "protocolo %X  endereco %X\r\n",route.protocol, route.proto_address);
        int entry = checkRoutingTable(route);
        if(entry == -1){
            //Serial.printf("do nothing, already have route to ");
            if(DEBUG){
                printAddress(route.destination);
            }
            //Serial.printf("\n");
        }else{
            // average neighbor metric with rest of route metric
            float hopRatio = 1/((float)route.distance);
            metric = ((float) neighborTable[n_entry].metric)*(hopRatio) + ((float)route.metric)*(1-hopRatio);
            route.metric = (uint8_t) metric;
            //if(routeEntry <= 30){
            updateRouteTable(route, entry);
            //}
        }
    }
    return numberOfRoutes;
}


int packet_received(char* data, size_t len) {

    struct Packet packet;
    struct Metadata metadata;
    
    data[len] = '\0';
    // convert ASCII data to pure bytes
    uint8_t* byteData = ( uint8_t* ) data;

#ifdef SIMULATION
    int packet_rssi = rand() % (256 - 128) + 128;
    float packet_snr = rand() % (256 - 128) + 128;
    // articial packet loss
    uint8_t packet_randomness = rand() % (256 - 128) + 128;
#else
    int packet_rssi = lora32_packet_rssi(&lora); 
    float packet_snr = lora32_packet_snr(&lora); 
    uint8_t packet_randomness = lora32_packet_ramdomness(&lora);
#endif
    

    metadata.rssi = packet_rssi;
    metadata.snr = packet_snr;
    metadata.randomness = packet_randomness;

    packet.ttl = byteData[0];
    packet.totalLength = byteData[1];
    memcpy(&packet.source, &byteData[2], ADDR_LENGTH);
    memcpy(&packet.destination, &byteData[8], ADDR_LENGTH);
    packet.sequence = byteData[14];
    packet.type = byteData[15];
   // memcpy(&packet.data, &byteData[16], sizeof(packetLength));
    memcpy(packet.data, byteData + HEADER_LENGTH, packet.totalLength-HEADER_LENGTH);

    //printPacketInfo(packet, metadata);
    
    switch(packet.type){
        case 'h' :
            // hello packet;
            parseHelloPacket(packet, metadata);
            //printNeighborTable();
            break;
        case 'r':
            // routing packet;
            parseRoutingPacket(packet, metadata);
            //printRoutingTable();
            break;
        case 'c' :
            // chat packet
            //parseChatPacket(packet);
            //Serial.printf("this is a chat message\n");
            break;
        case 'm' :
            //Serial.printf("this is a map message\n");
            break;
        default :
            //printPacketInfo(packet, metadata);
            Serial.printf("message type not found\n");
    }

} 



long lastRouteTime = 0;
void transmitRoutes(){

    if (time(NULL) - lastRouteTime > routeInterval()) {
        uint8_t data[240];
        int dataLength = 0;
        //Serial.printf("number of routes before transmit: %d\n", routeEntry);
        if (routeEntry == 0){
            lastRouteTime = time(NULL);
            return;
        }
        int routesPerPacket = routeEntry;
        if (routeEntry >= MAX_ROUTES_PER_PACKET-1){
            routesPerPacket = MAX_ROUTES_PER_PACKET-1;
        }
        // random select without replacement of routes
        for( int i = 0 ; i < routesPerPacket ; i++){
            for( int j = 0 ; j < ADDR_LENGTH ; j++){
                data[dataLength] = routeTable[i].destination[j];
                dataLength++;
            }
            data[dataLength] = routeTable[i].distance; //distance
            dataLength++;
            data[dataLength] = routeTable[i].metric;
            dataLength++;
            data[dataLength] = routeTable[i].protocol;
            dataLength++;
            data[dataLength] = routeTable[i].proto_address;
            dataLength++;
        }
        //Serial.printf("Sending route data: ");
        for(int i = 0 ; i < dataLength ; i++){
            //Serial.printf("%02x ", data[i]);
        }
        //Serial.printf("\n");
        uint8_t destination[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        struct Packet routeMessage = buildPacket(1, mac, destination, messageCount, 'r', data, dataLength); 
        uint8_t* sending = (uint8_t*) malloc(sizeof(routeMessage));
        memcpy(sending, &routeMessage, sizeof(routeMessage));
        send_packet(sending, routeMessage.totalLength);
        messageCount++;
        lastRouteTime = time(NULL);
        //Serial.printf("Sending routes: ");
        for(int i = 0 ; i < routeMessage.totalLength ; i++){
            //Serial.printf("%02x", sending[i]);
        }
        //Serial.printf("\n");
    }
}

long lastHelloTime = 0;
void transmitHello(){

    if (time(NULL) - lastHelloTime > helloInterval()) {
        uint8_t data[240];
        data[0] = protocol;
        data[1] = proto_address;
        data[2] = 0x11;
        data[3] = 0x11;
        int dataLength = 4;
        //TODO: add randomness to message to avoid hashisng issues
        uint8_t destination[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        struct Packet helloMessage = buildPacket(1, mac, destination, messageCount, 'h', data, dataLength); 
        uint8_t* sending = (uint8_t*)  malloc(sizeof(helloMessage));
        memcpy(sending, &helloMessage, sizeof(helloMessage));
        send_packet(sending, helloMessage.totalLength);
        messageCount++;
        lastHelloTime = time(NULL);
        //Serial.printf("Sending beacon: ");
        for(int i = 0 ; i < helloMessage.totalLength ; i++){
            //Serial.printf("%02x", sending[i]);
        }
        //Serial.printf("\r\n");
    }
}

int choose = 0;
long lastMessageTime = 0;
void transmitToRandomRoute(){

    if (time(NULL) - lastMessageTime > messageInterval()) {

        if (routeEntry == 0){
            Serial.printf("node trying to send but has no routes\r\n");
            lastMessageTime = time(NULL);
            return;
        }
        uint8_t destination[ADDR_LENGTH];
        memcpy(destination, &routeTable[choose].destination, sizeof(destination));

        Serial.printf("node  sending a message using route %d to ", choose);
        for( int j = 0 ; j < ADDR_LENGTH ; j++){
            Serial.printf("%02x", routeTable[choose].destination[j]);
        }
        Serial.printf(" via ");
        for( int j = 0 ; j < ADDR_LENGTH ; j++){
            Serial.printf("%02x", routeTable[choose].nextHop[j]);
        }
        Serial.printf("\n");

        uint8_t data[240];
        int dataLength = 0;
        for( int i = 0 ; i < ADDR_LENGTH ; i++){
            data[dataLength] = routeTable[choose].nextHop[i];
            dataLength++;
        }
        struct Packet randomMessage = buildPacket(32, mac, destination, messageCount, 'c', data, dataLength); 
        uint8_t* sending = (uint8_t*)  malloc(sizeof(randomMessage));
        memcpy(sending, &randomMessage, sizeof(randomMessage));
        send_packet(sending, randomMessage.totalLength);
        messageCount++;
        choose++;
        if(choose == routeEntry){
            choose = 0;
        }
        lastMessageTime = time(NULL);
    }
}




#ifdef SIMULATION

long startTime;
int chance;
int setup() {
    Serial.printf("node %s initialized\n", nodeID);
    debug_printf("debuggin enabled\n");

    srand(time(NULL) + getpid());
    // random wait at boot
    int wait = rand()%maxRandomDelay();
    debug_printf("waiting %d s\n", wait);
    nsleep(wait, 0);

    startTime = time(NULL);
    lastHelloTime = time(NULL);
    lastRouteTime = time(NULL);
    _learningTimeout += wait;
    _discoveryTimeout += wait;

    chance=rand()%15;
    if(chance == 1){
        Serial.printf("node %s shall send a random message\n", nodeID);
    }

    return 0;
}


int state = 0;
int loop() {

    if(!begin_packet()){
        //debug_printf("transmit in progress please wait");
    }else{
        if(state == 0){
            Serial.printf("learning... %d\r", time(NULL) - startTime);
            if(DEBUG){
                printNeighborTable();
                printRoutingTable();
            }
            transmitHello();
            if(time(NULL) - startTime > discoveryTimeout()) {
                state++;
            }
        }else if(state == 1){
            Serial.printf("learning... %d\r", time(NULL) - startTime);
            if(DEBUG){
                printNeighborTable();
                printRoutingTable();
            }
            transmitRoutes();
            if(time(NULL) - startTime > learningTimeout()) {
                state++;
                printNeighborTable();
                printRoutingTable();
            }
        }else if(state == 2){
            checkBuffer();
            if(chance == 1){
                transmitToRandomRoute();
            }
        }
    }
    nsleep(0, 1000000*simulationTime(1));
}

#else // Real Application

int state = 0;

void loop(void *p) {
  ESP_LOGI(TAG, "starting main loop");

  while(true) {
    //lora32_send(&lora, data_available, 11);
        if(state == 0){
            Serial.printf("learning... %ld\r\n", time(NULL) - startTime);
            if(DEBUG){
                printNeighborTable();
                printRoutingTable();
            }
            transmitHello();
            if(time(NULL) - startTime > discoveryTimeout()) {
                state++;
            }
        }else if(state == 1){
            Serial.printf("learning... %ld\r\n", time(NULL) - startTime);
            if(DEBUG){
                printNeighborTable();
                printRoutingTable();
            }
            transmitRoutes();
            if(time(NULL) - startTime > learningTimeout()) {
                state++;
                printNeighborTable();
                printRoutingTable();
            }
        }else if(state == 2){
            checkSendBuffer(); 
        }


    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void app_main()
{
    esp_err_t ret;

    printf("Hello world!\r\n");

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_read_mac(mac, 0);

    wifi_init_softap();

    lora = lora32_create();
    lora.frequency = frequencies[F915];
    lora.receive = &packet_received;
   
    lora32_init(&lora);

    serial_init();

    printf("Node %X:%X:%X:%X:%X:%X initialized\r\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5] );

    startTime = time(NULL);
    lastHelloTime = time(NULL);
    lastRouteTime = time(NULL);
   
    xTaskCreate(loop, "loop",  4096, NULL, 6, NULL);
    
}

#endif //SIMULATION