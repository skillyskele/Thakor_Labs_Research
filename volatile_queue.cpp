#include "circular_queue.h"
#include "common_config.h"
#include <cstring> // for memcpy
#include <cstdlib> // for new/delete

#define PACKET_SIZE ((SAMPLES_PER_BUFFER + 1) * sizeof(uint32_t))

class Volatile_Queue : public Circular_Queue {


// Private static instance
private:
  int front;
  int back;
  int capacity;
  int num_elements;
  uint32_t* packet_history;
public:
  Volatile_Queue(int cap)
      : front(0), back(0), capacity(cap), num_elements(0)
  {
      packet_history = new uint32_t[capacity * (SAMPLES_PER_BUFFER + 1)];
  }

  ~Volatile_Queue() override {
      delete[] packet_history;
  }
bool enqueue(const BluetoothPacket& packet) override {
    if (full()) {
        return false; // queue full
    }

    uint32_t* dest = &packet_history[back * (SAMPLES_PER_BUFFER + 1)];
    memcpy(dest, packet.samples, SAMPLES_PER_BUFFER * sizeof(PACKET_TYPE));
    dest[SAMPLES_PER_BUFFER] = packet.packet_id;

    back = (back + 1) % capacity;
    num_elements++;
    return true;
}

BluetoothPacket dequeue() override {
    if (num_elements == 0) {
        return BluetoothPacket();
    }

    // Get pointer to start of packet
    uint32_t* src = &packet_history[front * (SAMPLES_PER_BUFFER + 1)];

    BluetoothPacket packet;
    memcpy(packet.samples, src, SAMPLES_PER_BUFFER * sizeof(PACKET_TYPE));
    packet.packet_id = src[SAMPLES_PER_BUFFER];

    front = (front + 1) % capacity;
    num_elements--;

    return packet;
}

bool circular_buffer_empty(void) {
    return buffer_instance.num_elements == 0;
}

bool empty() const override { return num_elements == 0; }
bool full() const override { return num_elements == capacity; }

}



