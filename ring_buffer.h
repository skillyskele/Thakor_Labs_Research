#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stddef.h>
#include <stdint.h>

typedef struct {
    size_t s_elem; // configurable. 60 x uint32 or 60 x uint16 size packets
    size_t n_elem; // configurable. 16 packet queue?
    void *buffer; // points to static sampleQueue[QUEUE_SIZE] in code
} rb_attr_t;

typedef unsigned int rbd_t;

struct ring_buffer
{
    size_t s_elem;
    size_t n_elem;
    uint8_t *buf; // does this force my buf to be uint8 or  can i still configure
    volatile size_t head;
    volatile size_t tail;
};

# define RING_BUFFER_MAX 2 // one for BLE packets and one for compressed BLE packets
static struct ring_buffer _rb[RING_BUFFER_MAX]; // viewable from app.c as well as ring_buffer.c

int ring_buffer_init(rbd_t *rbd, rb_attr_t *attr);
static int _ring_buffer_full(struct ring_buffer *rb);
static int _ring_buffer_empty(struct ring_buffer *rb);
int ring_buffer_put(rbd_t rbd, const void *data);
int ring_buffer_get(rbd_t rbd, void *data);

#endif // RING_BUFFER_H_
