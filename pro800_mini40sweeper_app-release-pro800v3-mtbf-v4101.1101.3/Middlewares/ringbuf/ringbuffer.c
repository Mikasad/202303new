#include "ringbuffer.h"

/**
 * @file
 * Implementation of ring buffer functions.
 */

int32_t ring_buffer_create(ring_buffer_t *buffer, uint32_t size)
{
    buffer->buffer = pvPortMalloc(size);
    if(buffer->buffer == NULL)
        return -1;

    buffer->buf_mutex = xSemaphoreCreateMutex();
    buffer->tail_index = 0;
    buffer->head_index = 0;
    buffer->size = size;

    return size;
}

void ring_buffer_destroy(ring_buffer_t *buffer)
{
    if(buffer->buffer != NULL)
        vPortFree(buffer->buffer);
}

void ring_buffer_queue(ring_buffer_t *buffer, uint8_t data)
{
    /* Is buffer full? */
    if (ring_buffer_is_full(buffer))
    {
        /* Is going to overwrite the oldest byte */
        /* Increase tail index */
        buffer->tail_index = ((buffer->tail_index + 1) & (buffer->size - 1) );
    }

    /* Place data in buffer */
    buffer->buffer[buffer->head_index] = data;
    buffer->head_index = ((buffer->head_index + 1) & (buffer->size - 1) );
}

void ring_buffer_queue_arr(ring_buffer_t *buffer, const uint8_t *data, ring_buffer_size_t size)
{
    /* Add bytes; one by one */
    ring_buffer_size_t i;
    /* 上锁 */
    xSemaphoreTake(buffer->buf_mutex, portMAX_DELAY);

    for (i = 0; i < size; i++)
    {
        ring_buffer_queue(buffer, data[i]);
    }
    /* 开锁 */
    xSemaphoreGive(buffer->buf_mutex);
}

uint8_t ring_buffer_dequeue(ring_buffer_t *buffer, uint8_t *data)
{
    if (ring_buffer_is_empty(buffer))
    {
        /* No items */
        return 0;
    }

    *data = buffer->buffer[buffer->tail_index];
    buffer->tail_index = ((buffer->tail_index + 1) & (buffer->size - 1));

    return 1;
}

ring_buffer_size_t ring_buffer_dequeue_arr(ring_buffer_t *buffer, uint8_t *data, ring_buffer_size_t len)
{
    if (ring_buffer_is_empty(buffer))
    {
        /* No items */
        return 0;
    }

    uint8_t *data_ptr = data;
    ring_buffer_size_t cnt = 0;

    /* 上锁 */
    xSemaphoreTake(buffer->buf_mutex, portMAX_DELAY);

    while ((cnt < len) && ring_buffer_dequeue(buffer, data_ptr))
    {
        cnt++;
        data_ptr++;
    }
    /* 开锁 */
    xSemaphoreGive(buffer->buf_mutex);

    return cnt;
}

uint8_t ring_buffer_peek(ring_buffer_t *buffer, uint8_t *data, ring_buffer_size_t index)
{
    if (index >= ring_buffer_num_items(buffer))
    {
        /* No items at index */
        return 0;
    }

    /* Add index to pointer */
    ring_buffer_size_t data_index = ((buffer->tail_index + index) & (buffer->size - 1));
    *data = buffer->buffer[data_index];
    return 1;
}

extern inline uint8_t ring_buffer_is_empty(ring_buffer_t *buffer);
extern inline uint8_t ring_buffer_is_full(ring_buffer_t *buffer);
extern inline ring_buffer_size_t ring_buffer_num_items(ring_buffer_t *buffer);