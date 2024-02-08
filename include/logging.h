#pragma once
#include <Arduino.h>
#include "common.h"



namespace LOGGING {

const uint16_t MAX_LOG_SIZE = 128;
const uint8_t MAX_LOG_BUFFER_DEPTH = 4; // Must be power of 2
const uint8_t MAX_TAG_SIZE = 16;
const uint8_t MAX_LOG_LEVEL = 5;

enum {LOGGING_ERROR=0, LOGGING_WARN, LOGGING_NOTICE, LOGGING_INFO, LOGGING_DEBUG};


typedef struct logItem {
    uint8_t level;
    uint32_t timestamp;
    char tag[MAX_TAG_SIZE];
    char log_message[MAX_LOG_SIZE];
} logItem;

typedef struct ringBuffer {
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile bool overflow_error;
    logItem messages[MAX_LOG_BUFFER_DEPTH];

} ringBuffer;


class Logging {
    public:
    void log(const char *tag, uint8_t level, uint32_t timestamp, const char *format, ...);

    void loop();



    protected:
    ringBuffer _ring_buffer;

    bool _buffer_full() { return (((this->_ring_buffer.head + 1) & (MAX_LOG_BUFFER_DEPTH - 1)) == this->_ring_buffer.tail); }
    bool _buffer_empty() { return (this->_ring_buffer.tail == this->_ring_buffer.head); }
    uint8_t _buffer_next(uint8_t buffer) { return (buffer + 1) & (MAX_LOG_BUFFER_DEPTH - 1); }
    void _xmit_logitem(const char *tag, uint8_t level, uint32_t timestamp, const char *str);
};

} // End Namespace LOGGING


