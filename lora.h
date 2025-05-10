
#ifndef LORA_MODULE
#define LORA_MODULE

#define PACKET_MAX_LENGTH 60
#define PACKET_DATA_MAX_LENGTH 40

void LoRaInit();
bool LoRaService();
void LoRaSendPacket(const char* packet, bool debugOn);

#endif
