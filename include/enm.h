#pragma once
#include "common.h"

namespace ENM {

const uint8_t LINE_COUNT = 2;
const uint32_t TICK_TIME = 25;
const uint8_t EVENT_BUFFER_DEPTH = 4;
const uint8_t EVENT_BUFFER_SIZE = 2;
const uint8_t MAX_I2C_DATA_LENGTH = 3;

/*
* Tunable parameters
*/

const uint16_t WINK_WAIT =  200; // Minimum time to wait before sending wink or after sending wink
const uint16_t WINK_TIME = 200; // Duration of the wink pulse we send
const uint16_t CALL_ABORT_TIME = 100; // Assume call aborted if greater than this time.
const uint16_t WINK_TIMEOUT = 5000; // Time out if no wink seen from far end
const uint16_t MF_ADDRESS_TIME = 5000; // Time to wait for MF digits to be completely sent
const uint16_t SEND_HOLD_TIME = 2000; // Time to leave the send signal asserted after a call is completed.



// Software timers
enum {TIMER_WINK=0, TIMER_WINK_WAIT, TIMER_SENSE, TIMER_GUARD, MAX_TIMERS};
// Events sent to main CPU
enum {EV_NONE=0, EV_REQUEST_IR=1, EV_CALL_DROPPED=2};
// Test modes
enum {TM_NONE=0, TM_STANDALONE=1};
// I2C registers
enum {REG_GET_EVENT=0, REG_GET_BUSY_STATUS=1};
// Main Handler States
enum {MHS_IDLE=0, MHS_PREWINK_WAIT=1, MHS_WAIT_IR=2, MHS_SEND_WINK=3, MHS_SEND_WINK_WAIT=4, MHS_WAIT_ADDRESS_COMPLETE=5, 
MHS_WAIT_MF_ADDR=6, MHS_WAIT_INCOMING_CONNECT=7, MHS_ANSWER_SUPV=8, MHS_IN_INCOMING_CALL=9, MHS_END_INCOMING_CALL=10, MHS_WAIT_INCOMING_HOLD=11};


typedef struct eventBuffer {
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile bool overflow_error;
    uint8_t ring_buffer [EVENT_BUFFER_DEPTH][EVENT_BUFFER_SIZE];
} eventBuffer;


class ENM {
public:
    void setup();
    void service();
    void loop();
    void i2c_request();
    void i2c_receive(int howMany);
protected:
    void _set_orig_led(bool state);
    void _set_term_led(bool state);
    void _set_send(bool state);
    bool _get_sense();
    bool _event_buffer_full() { return (((this->_event_buffer.head + 1) & (EVENT_BUFFER_DEPTH - 1)) == this->_event_buffer.tail); }
    bool _event_buffer_empty() { return (this->_event_buffer.tail == this->_event_buffer.head); }
    uint8_t _event_buffer_next(uint8_t buffer) { return (buffer + 1) & (EVENT_BUFFER_DEPTH - 1); }
    bool _request_service(uint8_t event);


    void _main_handler();

 


  

    volatile bool _i2c_read_data_ready;
    volatile bool _mh_state_advance;
    uint8_t _service_ticks;
    uint8_t _i2c_address;
    volatile uint8_t _mh_state;
    volatile uint8_t _test_mode;
    volatile uint8_t _i2c_register_address;
    volatile uint8_t _i2c_read_data_length;
    volatile uint8_t _i2c_read_data[MAX_I2C_DATA_LENGTH];
    eventBuffer _event_buffer;
    uint16_t _mh_timer[MAX_TIMERS];
    
};

} // End namespace ENM
