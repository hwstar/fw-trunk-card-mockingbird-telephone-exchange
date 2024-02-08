#include "common.h"
#include "enm.h"

#define TAG "enm"

namespace ENM {


void ENM::_set_orig_led(bool state) {
  digitalWrite(LED_ORIG_OFHN, !state);
}

void ENM::_set_term_led(bool state) {
  digitalWrite(LED_TERM_OFHN, !state);
}

void ENM::_set_send(bool state) {
  digitalWrite(SEND, state);
}

bool ENM::_get_sense()  {
  return !digitalRead(SENSE);
}

/*
* I2C Master is requesting something to be sent to it
*/

void ENM::i2c_request() {
  uint8_t tx_str[EVENT_BUFFER_SIZE];
  // Process master TX requests here
  switch(this->_i2c_register_address) {
    case REG_GET_EVENT:
      if(!this->_event_buffer_empty()) {
        memcpy(tx_str, this->_event_buffer.ring_buffer[this->_event_buffer.tail], EVENT_BUFFER_SIZE); // Make local copy of event
        this->_event_buffer.tail = this->_event_buffer_next(this->_event_buffer.tail); // Next tail position
        // If event buffer is empty, drop ATTEN
        if(this->_event_buffer_empty()) {
          digitalWrite(ATTEN, LOW);
        }
        // Send the event
        Wire.write(tx_str[0]);
        Wire.write(tx_str[1]);
      }
      else { // Event queue was empty
        digitalWrite(ATTEN, LOW); // Drop ATTEN to be sure
        Wire.write(EV_NONE);
        Wire.write(0xFF);
      }
      break;

    default:
      Wire.write(0xFF);
      Wire.write(0xFF);
      break;
  }
}

/*
* We received some data from the I2C Master
*/

void ENM::i2c_receive(int howMany) {
  if(howMany > 0) {
    // First byte is the I2C register address 
    this->_i2c_register_address = Wire.read();
    howMany--;
    this->_i2c_read_data_length = 0;
    // If there is data following the I3C address process it here.
    while((howMany) && (this->_i2c_read_data_length < MAX_I2C_DATA_LENGTH)) {
        this->_i2c_read_data[this->_i2c_read_data_length++] = Wire.read();
        howMany--;
    }
    // Signal to the foreground that read data is ready to be processed.
    if(this->_i2c_read_data_length) {
      this->_i2c_read_data_ready = true;
    }
  }
}

/*
* Request service from main processor
*/

bool ENM::_request_service(uint8_t event) {
  bool res = !this->_event_buffer_full();
  LOG_DEBUG(TAG, "Queuing event %d", event);
  //if(this->_test_mode) { // TODO: Uncomment
  //  return true;
  //}

  if(res) {
    uint8_t *ev = this->_event_buffer.ring_buffer[this->_event_buffer.head];
    ev[0] = event;
    this->_event_buffer.head = this->_event_buffer_next(this->_event_buffer.head);
    digitalWrite(ATTEN, HIGH); // Set attention
  }
  else {
    this->_event_buffer.overflow_error = true;
  }

  return res;
}



/*
* Main Handler 
*/

void ENM::_main_handler() {
  for( int i = 0; i < LINE_COUNT; i++) {
    
    switch(this->_mh_state) {

      case MHS_IDLE:
        if(this->_get_sense()) { // Incoming call?
          this->_mh_timer[TIMER_WINK] = WINK_WAIT/TICK_TIME;
          this->_mh_state = MHS_PREWINK_WAIT;

        }
        break;

      case MHS_PREWINK_WAIT:
        if(!this->_get_sense()) { // Was it noise?
          this->_mh_state = MHS_IDLE;
        }
        else if(this->_mh_timer[TIMER_WINK] == 0) { 
          // If in test mode, send a wink immediately
          // else wait for the main processor to connect
          // an IR.
          if(this->_test_mode) {
            this->_mh_state = MHS_SEND_WINK;
          }
          else {
            this->_request_service(EV_REQUEST_IR);
            this->_mh_timer[TIMER_SENSE] = CALL_ABORT_TIME/TICK_TIME;
            this->_mh_state = MHS_WAIT_IR;
          }
        }
        else {
          this->_mh_timer[TIMER_WINK]--;
        }
        break;

      case MHS_WAIT_IR:
        if(!this->_get_sense()) {
          if(this->_mh_timer[TIMER_SENSE] == 0) {
            // Call was dropped
            this->_mh_state = MHS_IDLE;
          }
          else {
            //Call may have been dropped
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else { // Call still active
          this->_mh_timer[TIMER_SENSE] = CALL_ABORT_TIME/TICK_TIME;
          // TODO: Wait for main CPU to send us a state advance to MHS_STATE_WINK
        }
        break;

      case MHS_SEND_WINK: // Send wink to the originating end
        this->_set_send(true);
        this->_mh_timer[TIMER_WINK] = WINK_TIME/TICK_TIME;
        this->_mh_state = MHS_SEND_WINK_WAIT;
        break;

      case MHS_SEND_WINK_WAIT:
        if(this->_mh_timer[TIMER_WINK] == 0) {
          this->_set_send(false); // Release send
          if(this->_test_mode) {
              // Wait for MF Address to be sent
              this->_mh_timer[TIMER_GUARD] = MF_ADDRESS_TIME/TICK_TIME;
              this->_mh_state = MHS_WAIT_MF_ADDR;
          }
          else {
            this->_mh_timer[TIMER_SENSE] = CALL_ABORT_TIME/TICK_TIME;
            this->_mh_state = MHS_WAIT_INCOMING_CONNECT;
          }
        }
        else {
          this->_mh_timer[TIMER_WINK]--;
        }
        break;

      case MHS_WAIT_INCOMING_CONNECT: // Call to destination in process
        if(!this->_get_sense()) {
          if(this->_mh_timer[TIMER_SENSE] == 0) {
            // Call was dropped
            // Send a message to the main CPU indicating this
            this->_request_service(EV_CALL_DROPPED);
            this->_mh_state = MHS_IDLE;
          }
          else {
            //Call may have been dropped
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else {
          // TODO: Wait for main CPU to send answer supervision
        }
        break;

        case MHS_WAIT_MF_ADDR: // Only used in test mode
        // Hold of setting send so that we capture the MF address digits
        if(this->_mh_timer[TIMER_GUARD] == 0) {
          this->_mh_state = MHS_ANSWER_SUPV;
        }
        else {
          this->_mh_timer[TIMER_GUARD]--;
        }
        break;

        case MHS_ANSWER_SUPV:
        // Wait for the wink timer to expire, then set send.
        if(this->_mh_timer[TIMER_WINK] == 0) {
          this->_set_send(true);
          this->_mh_timer[TIMER_SENSE] = CALL_ABORT_TIME/TICK_TIME;
          this->_mh_state = MHS_IN_INCOMING_CALL;
          break;
        }
        else {
          this->_mh_timer[TIMER_WINK]--;
        }

      case MHS_IN_INCOMING_CALL:
        if(!this->_get_sense()) {
          // Monitor other end for end of call
          if(this->_mh_timer[TIMER_SENSE] == 0) {
            // Call was dropped
            // Send a message to the main CPU indicating this
            this->_request_service(EV_CALL_DROPPED);
            this->_mh_state = MHS_END_INCOMING_CALL;
          }
          else {
            // Call may have been dropped
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else {
          // TODO: Monitor for a disconnect request from the main CPU
        }
        break;

      case MHS_END_INCOMING_CALL:
        this->_mh_timer[TIMER_GUARD] = SEND_HOLD_TIME/TICK_TIME;
        this->_mh_state = MHS_WAIT_INCOMING_HOLD;
        break;

      case MHS_WAIT_INCOMING_HOLD:
        if(this->_mh_timer[TIMER_GUARD] == 0) {
          this->_set_send(false); // Tell far side we are available
          this->_mh_state = MHS_IDLE;
        }
        else {
          this->_mh_timer[TIMER_GUARD]--;
        }
        break;

       default:
        this->_mh_state = MHS_IDLE;
        this->_set_send(false);
        break;
    }
    break;
  } 
} 

/*
* Service. Called evey 5 msec.
*/

void ENM::service(void) {
  if(this->_service_ticks++ == 0) {
    this->_main_handler(); // Called every 25 mS.
  }
  if(this->_service_ticks == 5) {
    this->_service_ticks = 0;
  }
}



/*
* Setup
*/


void ENM::setup() {
  pinMode(SEND, OUTPUT);
  digitalWrite(SEND, LOW);

  pinMode(SENSE, INPUT);

  pinMode(LED_ORIG_OFHN, OUTPUT);
  digitalWrite(LED_ORIG_OFHN, HIGH);

  pinMode(LED_TERM_OFHN, OUTPUT);
  digitalWrite(LED_TERM_OFHN, HIGH);

  pinMode(TP401, OUTPUT);
  digitalWrite(TP401, LOW);

  pinMode(TP402, OUTPUT);
  digitalWrite(TP402, LOW);

  pinMode(TP403, OUTPUT);
  digitalWrite(TP403, LOW);

  pinMode(ATTEN, OUTPUT);
  digitalWrite(ATTEN, LOW);

  

  this->_test_mode = 1; // DEBUG

}

/*
* Foreground loop
*/


void ENM::loop() {

  if(this->_event_buffer.overflow_error) {
    LOG_ERROR(TAG, "Event buffer overflow");
    this->_event_buffer.overflow_error = false;
  }

  // Check for and process write register requests from the main processor
  if(this->_i2c_read_data_ready) {
    this->_i2c_read_data_ready = false; // Request processed
  }
}

} // End Namespace ENM