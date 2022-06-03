#ifndef __RING_BUFFER_INC_
#define __RING_BUFFER_INC_

#include "stdint.h"
#include "stdlib.h"

typedef struct ring_buffer_  {
    uint8_t  *buffer; // Pointer to the fixed-size buffer
    size_t   head;    // Head of the ring buffer
    size_t   tail;    // Tail of the ring buffer 
    size_t   max;     // Total size of the buffer / Capacity /
    uint8_t  full;    // Flag to record when the buffer gets full 
} ring_buffer_t;


uint8_t ring_buffer_init(ring_buffer_t *, uint8_t *, size_t);
uint8_t ring_buffer_full(ring_buffer_t *);
uint8_t ring_buffer_empty(ring_buffer_t *);
size_t ring_buffer_size(ring_buffer_t *);
uint8_t ring_buffer_get(ring_buffer_t *, uint8_t *);
void ring_buffer_put(ring_buffer_t *, uint8_t);
uint8_t *ring_buffer_tail_position(ring_buffer_t *);


#endif /* __RING_BUFFER_INC_ */