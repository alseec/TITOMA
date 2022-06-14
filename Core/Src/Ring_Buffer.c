#include "Ring_Buffer.h"

static void ring_buffer_reset(ring_buffer_t *ring_buffer)
{
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    ring_buffer->full = 0;
}

uint8_t ring_buffer_full(ring_buffer_t *ring_buffer)
{
    return (ring_buffer->full);
}

uint8_t ring_buffer_empty(ring_buffer_t *ring_buffer)
{
    return ((ring_buffer->full == 0) && (ring_buffer->head == ring_buffer->tail));
}

size_t ring_buffer_size(ring_buffer_t *ring_buffer)
{
	size_t size;

	if(ring_buffer_full(ring_buffer)) {
		return(ring_buffer->max);
    } else {
        if(ring_buffer->head >= ring_buffer->tail) {
            size = (ring_buffer->head - ring_buffer->tail);
        } else {
            size = (ring_buffer->max - ring_buffer->tail + ring_buffer->head);
        }
    }
	return size;
}

uint8_t ring_buffer_get(ring_buffer_t *ring_buffer, uint8_t *p_data)
{
	if (!ring_buffer_empty(ring_buffer)) {
		*p_data = ring_buffer->buffer[ring_buffer->tail];
		ring_buffer->tail = (ring_buffer->tail + 1) % ring_buffer->max;
		ring_buffer->full = 0;
	}

	return 0;
}

void ring_buffer_put(ring_buffer_t *ring_buffer, uint8_t data)
{
	ring_buffer->buffer[ring_buffer->head] = data;
	ring_buffer->head = (ring_buffer->head + 1) % ring_buffer->max;
	
	if (ring_buffer_full(ring_buffer)) {
		ring_buffer->tail = (ring_buffer->tail + 1) % ring_buffer->max;
	}
	
	if (ring_buffer->head == ring_buffer->tail) {
		ring_buffer->full = 1;
	}
}

uint8_t ring_buffer_init(ring_buffer_t *ring_buffer, uint8_t *p_buffer, size_t len)
{
	if(p_buffer == NULL || (len == 0)) {
		return 0;
	}
	ring_buffer->buffer = p_buffer;
	ring_buffer->max = len;
	ring_buffer_reset(ring_buffer);
	return 1;
}

uint8_t *ring_buffer_tail_position(ring_buffer_t *ring_buffer)
{
	return &ring_buffer->buffer[ring_buffer->tail];
}
