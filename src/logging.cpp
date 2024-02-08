#include "common.h"

namespace LOGGING {

const char *TAG = "logger";

const char *log_level_strings[MAX_LOG_LEVEL] = {
    "ERROR",
    "WARN",
    "NOTICE",
    "INFO",
    "DEBUG"
};


void Logging::_xmit_logitem(const char *tag, uint8_t level, uint32_t timestamp, const char *str) {
    if(level > MAX_LOG_LEVEL) {
        level = 0; // Protect against bad log level being passed in.
    }

    // Convert time stamp to H:M:S.MS format
    uint32_t hours = timestamp / 3600000;
    timestamp %= 36000000;
    uint8_t minutes = timestamp / 60000;
    timestamp %= 60000;
    uint8_t seconds = timestamp / 1000;
    timestamp %= 1000;

    Serial.printf("[%u:%02u:%02u.%03u] LOG_%s(%s):%s\r\n", hours, minutes, seconds, timestamp, log_level_strings[level], tag, str);
}


void Logging::log(const char *tag, uint8_t level, uint32_t timestamp, const char *format, ...) {
   

    if(this->_buffer_full()){
        this->_ring_buffer.overflow_error = true;
    }
    else {
        va_list alp;
        va_start(alp, format);
        logItem *li = &this->_ring_buffer.messages[_ring_buffer.head];
        vsnprintf(li->log_message, MAX_LOG_SIZE, format, alp);
        va_end(alp);
        li->log_message[MAX_LOG_SIZE - 1] = 0;
        strncpy(li->tag, tag, MAX_TAG_SIZE);
        li->tag[MAX_TAG_SIZE - 1] = 0;
        li->level = level;
        li->timestamp = timestamp;
        // Advance to next position in ring buffer
        _ring_buffer.head = this->_buffer_next(_ring_buffer.head);
    }

}


void Logging::loop() {

    if(this->_ring_buffer.overflow_error) {
        this->_xmit_logitem(TAG, LOGGING_ERROR, millis(), "Log Buffer Overflow");
        this->_ring_buffer.overflow_error = false;

    }
    else { 
        while(!this->_buffer_empty()) {
            logItem *li = &this->_ring_buffer.messages[_ring_buffer.tail];
            this->_xmit_logitem(li->tag, li->level, li->timestamp, li->log_message);
            // Advance to next position in the ring buffer
            _ring_buffer.tail = this->_buffer_next(_ring_buffer.tail);
        }
    }

}

} // End Namespace LOGGING
