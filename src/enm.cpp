#include "common.h"
#include "enm.h"

#define TAG "enm"

namespace ENM {


void ENM::_set_send(bool state) {
  digitalWrite(SEND, state);
  digitalWrite(LED_ORIG_OFHN, !state);
}

bool ENM::_get_sense()  {
  bool res = !digitalRead(SENSE);
  digitalWrite(LED_TERM_OFHN, !res);
  return res;

}

/*
* I2C Master is requesting something to be sent to it
*/

void ENM::i2c_request() {
  uint8_t tx_str[EVENT_BUFFER_SIZE];
  // Process master TX requests here
  switch(this->_i2c_register_address) {
    case REG_GET_EVENT: // Return an event from the queue.
      if(!this->_event_buffer_empty()) {
        memcpy(tx_str, this->_event_buffer.ring_buffer[this->_event_buffer.tail], EVENT_BUFFER_SIZE); // Make local copy of event
        this->_event_buffer.tail = this->_event_buffer_next(this->_event_buffer.tail); // Next tail position
        // If event buffer is empty, drop ATTEN
        if(this->_event_buffer_empty()) {
          digitalWrite(ATTEN, LOW);
        }
        // Send the event
        Wire.write(tx_str[0]);
      }
      else { // Event queue was empty
        digitalWrite(ATTEN, LOW); // Drop ATTEN to be sure
        Wire.write(0x00); 
 
      }
      break;

    case REG_GET_BUSY_STATUS: // Return busy status
      if(this->_mh_state == MHS_IDLE) {
        Wire.write(0x00);
      }
      else {
        Wire.write(0x01);
      }
      break;


    default:
      Wire.write(0xFF); // Invalid
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
    // If there is data following the I2C address process it here.
    while((howMany) && (this->_i2c_read_data_length < MAX_I2C_DATA_LENGTH)) {
        this->_i2c_read_data[this->_i2c_read_data_length++] = Wire.read();
        howMany--;
    }
  
    this->_i2c_read_data_ready = true;
  }
}

/*
* Request service from main processor
*/

bool ENM::_request_service(uint8_t event) {
  bool res = !this->_event_buffer_full();
  LOG_DEBUG(TAG, "Queuing event %d", event);
  if(this->_test_mode) { 
    return true; // Don't queue the event in test mode
  }

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
        else { // Check for local seizure
          if(this->_mh_state_advance) {
            this->_mh_state_advance = REG_NONE;
            this->_mh_timer[TIMER_GUARD] = WINK_TIMEOUT/TICK_TIME;
            this->_set_send(true);
            this->_mh_state = MHS_WAIT_FAREND_WINK;
          }
        }
        break;

      case MHS_PREWINK_WAIT:
        if(!this->_get_sense()) { // Was it noise?
          LOG_DEBUG(TAG, "Call dropped in PREWINK, going back to IDLE");
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
            this->_mh_state_advance = REG_NONE;
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
            LOG_DEBUG(TAG, "Call dropped in WAIT_IR");
            this->_request_service(EV_CALL_DROPPED);
            this->_mh_state = MHS_END_INCOMING_CALL;
          }
          else {
            //Call may have been dropped
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else { // Call still active
          this->_mh_timer[TIMER_SENSE] = CALL_ABORT_TIME/TICK_TIME;
          // Wait for main CPU to send us a state advance to MHS_STATE_WINK
          if(this->_mh_state_advance) {
            this->_mh_state_advance = REG_NONE;
            this->_mh_state = MHS_SEND_WINK;
          }

        }
        break;

      case MHS_SEND_WINK: // Send wink to the originating end
        LOG_DEBUG(TAG, "Sending wink");
        this->_set_send(true);
        this->_mh_state_advance = REG_NONE;
        this->_mh_timer[TIMER_WINK] = WINK_TIME/TICK_TIME;
        this->_mh_state = MHS_SEND_WINK_WAIT;
        break;

      case MHS_SEND_WINK_WAIT:
        if(this->_mh_timer[TIMER_WINK] == 0) {
          this->_set_send(false); // Release send
          if(this->_test_mode) {
            // Wait for MF Address to be sent
            LOG_DEBUG(TAG,"Test mode state advance");
            this->_mh_timer[TIMER_GUARD] = MF_ADDRESS_TIME/TICK_TIME;
            this->_mh_state = MHS_WAIT_MF_ADDR;
          }
          else {
            LOG_DEBUG(TAG,"Waiting for incoming connect");
            this->_mh_state_advance = REG_NONE;
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
            LOG_DEBUG(TAG, "Call dropped during incoming connect");
            this->_request_service(EV_CALL_DROPPED);
            this->_mh_state = MHS_END_INCOMING_CALL;
          }
          else {
            //Call may have been dropped
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else {
          // Wait for main CPU to send answer supervision state advance
          if(this->_mh_state_advance){
            this->_mh_state_advance = REG_NONE;
            this->_mh_state = MHS_ANSWER_SUPV;
          }
        }
        break;

        case MHS_WAIT_MF_ADDR: // Only used in test mode
        // Hold off setting send so that we capture the MF address digits
        if(this->_mh_timer[TIMER_GUARD] == 0) {
          this->_mh_timer[TIMER_WINK] = 0;
          LOG_DEBUG(TAG,"Address wait time ended");
          this->_mh_state = MHS_ANSWER_SUPV;
        }
        else {
          this->_mh_timer[TIMER_GUARD]--;
        }
        break;

        case MHS_ANSWER_SUPV:
        // Wait for the wink timer to expire, then set send.
        if(this->_mh_timer[TIMER_WINK] == 0) {
          LOG_DEBUG(TAG, "Wink timer expired, setting send");
          this->_set_send(true);
          this->_mh_state_advance = REG_NONE;
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
            LOG_DEBUG(TAG, "Call dropped by the far end");
            this->_request_service(EV_CALL_DROPPED);
            this->_mh_state = MHS_END_INCOMING_CALL;
          }
          else {
            // Call may have been dropped
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else {
            // Monitor for a drop request from the main CPU
            if(this->_mh_state_advance) {
              this->_mh_state_advance = REG_NONE;
              this->_mh_state = MHS_END_INCOMING_CALL;
            }
        }
        break;

      case MHS_END_INCOMING_CALL:
        this->_set_send(false); // Drop send
        LOG_DEBUG(TAG, "Wait for sense lead to de-assert");
        this->_mh_timer[TIMER_GUARD] = SENSE_INACTIVE_TIME/TICK_TIME;
        this->_mh_state = MHS_WAIT_SENSE_INACTIVE;
        this->_mh_state_advance = REG_NONE;
        break;

      case MHS_WAIT_SENSE_INACTIVE:
        if(this->_get_sense()) {
          this->_mh_timer[TIMER_GUARD] = SENSE_INACTIVE_TIME/TICK_TIME;
        }
        else { // Sense is inactive
          if(this->_mh_timer[TIMER_GUARD] == 0) {
            LOG_DEBUG(TAG, "Returning to IDLE state");
            this->_mh_state = MHS_IDLE;
          }
          else {
            this->_mh_timer[TIMER_GUARD]--;
          }
        }
        break;


      case MHS_WAIT_FAREND_WINK: // Wait for wink from far end
        // Look for leading edge of wink
        if(this->_get_sense()) {
          this->_mh_timer[TIMER_MEAS_WINK] = 0; // Measure wink time using this software timer.
          this->_mh_timer[TIMER_GUARD] = WINK_TIMEOUT/TICK_TIME;
          this->_mh_state = MHS_WAIT_FOR_WINK_END;
        }
        else {  // Check for wink timeout
          if(!this->_mh_timer[TIMER_GUARD]) {
            // Send  NO_WINK message to main cpu
            LOG_WARN(TAG, "Wink Timeout, no wink seen");
            this->_set_send(false);
            this->_request_service(EV_NO_WINK);
            this->_mh_timer[TIMER_GUARD] = SENSE_INACTIVE_TIME/TICK_TIME;
            this->_mh_state = MHS_WAIT_OUTGOING_HOLD;

          }
          else {
            this->_mh_timer[TIMER_GUARD]--;
          }
        }
        break;

      case MHS_WAIT_FOR_WINK_END: //  Wait for trailing edge of wink
        if(!this->_get_sense()) {
          // Found trailing edge
          this->_mh_timer[TIMER_WINK_WAIT] = WINK_WAIT/TICK_TIME;
          this->_mh_state = MHS_WAIT_OUTGOING_POSTWINK;
          LOG_DEBUG(TAG, "Wink duration: %d ms", this->_mh_timer[TIMER_WINK_WAIT] * TICK_TIME);
        }
        else {
          this->_mh_timer[TIMER_MEAS_WINK]++;
        }
        break;


      case MHS_WAIT_OUTGOING_POSTWINK: // Wait before sending address info
        if(this->_mh_timer[TIMER_WINK_WAIT] == 0) {
          LOG_DEBUG(TAG, "Ready to send address information");
          this->_mh_state_advance = REG_NONE;
          this->_request_service(EV_SEND_ADDR_INFO);
          this->_mh_state = MHS_WAIT_SEND_ADDRESS_INFO;
        }
        else {
          this->_mh_timer[TIMER_WINK_WAIT]--;
        }
        break;

      case MHS_WAIT_SEND_ADDRESS_INFO:
        // Wait for signal indicating addressing has been completed by main CPU
        if(this->_mh_state_advance == REG_OUTGOING_ADDR_COMPLETE) {
          this->_mh_state_advance = REG_NONE;
          LOG_DEBUG(TAG, "Waiting for answer supervision");
          this->_mh_state = MHS_WAIT_FAREND_SUPV;
        }
        else if (this->_mh_state_advance == REG_DROP_CALL) {
          // Request to drop call
          this->_mh_state_advance = REG_NONE;
          this->_set_send(false);
          LOG_DEBUG(TAG,"Outgoing call terminated");
          this->_mh_timer[TIMER_SENSE] = SENSE_INACTIVE_TIME/TICK_TIME;
          this->_mh_state = MHS_WAIT_OUTGOING_HOLD;

        }
        break;

      case MHS_WAIT_FAREND_SUPV:
        // Wait for far end to assert sense indicating answer supervision
        if(this->_get_sense()){
          LOG_DEBUG(TAG, "Outgoing call answered");
          this->_request_service(EV_FAREND_SUPV);
          this->_mh_state = MHS_IN_OUTGOING_CALL;
        }
        // Test for abort command from main cpu
        else if(this->_mh_state_advance) {
          this->_mh_state_advance = REG_NONE;
          this->_set_send(false);
          LOG_DEBUG(TAG,"Outgoing call terminated");
          this->_mh_timer[TIMER_SENSE] = SENSE_INACTIVE_TIME/TICK_TIME;
          this->_mh_state = MHS_WAIT_OUTGOING_HOLD;
        }

        break;

      case MHS_IN_OUTGOING_CALL:
        if(!this->_get_sense()) {
          if(this->_mh_timer[TIMER_SENSE] == 0) {
            LOG_DEBUG(TAG, "Outgoing call terminated by far end");
            this->_set_send(false);
            this->_request_service(EV_FAREND_DISC);
            this->_mh_timer[TIMER_SENSE] = SENSE_INACTIVE_TIME/TICK_TIME;
            this->_mh_state = MHS_WAIT_OUTGOING_HOLD;
          }
          else {
            this->_mh_timer[TIMER_SENSE]--;
          }
        }
        else {
          this->_mh_timer[TIMER_SENSE] = SENSE_INACTIVE_TIME/TICK_TIME;
        }

        // Test for abort command from main cpu
        if(this->_mh_state_advance) {
          this->_mh_state_advance = REG_NONE;
          this->_set_send(false);
          LOG_DEBUG(TAG,"Outgoing call terminated");
          this->_mh_timer[TIMER_SENSE] = SENSE_INACTIVE_TIME/TICK_TIME;
          this->_mh_state = MHS_WAIT_OUTGOING_HOLD;
        }
        break;

      case MHS_WAIT_OUTGOING_HOLD:
        if(this->_get_sense()) {
          this->_mh_timer[TIMER_SENSE] = SENSE_INACTIVE_TIME/TICK_TIME;
        }
         // Test sense lead false for a prescribed time to guard against glare
        else if (this->_mh_timer[TIMER_SENSE] == 0) {
          this->_mh_state_advance = REG_NONE;
          this->_mh_state = MHS_IDLE;
          LOG_DEBUG(TAG, "Going back to idle state");
        }
        else {
          this->_mh_timer[TIMER_SENSE]--;
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

  

  this->_test_mode = 0; 

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
   
    noInterrupts();
   
   
      switch(this->_i2c_register_address) {

        case REG_SEIZE_TRUNK: // Seize trunk for outgoing call
          if(this->_mh_state == MHS_IDLE) {
            this->_mh_state_advance = REG_SEIZE_TRUNK;
          }
          else {
            this->_request_service(EV_BUSY); // Trunk in use. Send busy EVENT
          }
          break;

        case REG_SEND_WINK: // Send wink on incoming call request
          if(this->_mh_state == MHS_WAIT_IR) {
            this->_mh_state_advance = REG_SEND_WINK;
          }
          break;

        case REG_INCOMING_CONNECTED: // Incoming call connected by main CPU
          if(this->_mh_state == MHS_WAIT_INCOMING_CONNECT) {
            this->_mh_state_advance = REG_INCOMING_CONNECTED;
          }
          break;

        case REG_DROP_CALL: // Drop connected call
          if((this->_mh_state == MHS_IN_INCOMING_CALL) || (this->_mh_state == MHS_WAIT_SEND_ADDRESS_INFO) 
          || (this->_mh_state = MHS_WAIT_INCOMING_CONNECT) || (this->_mh_state = MHS_IN_OUTGOING_CALL)) {
            this->_mh_state_advance = REG_DROP_CALL;
          }
          break;


        case REG_OUTGOING_ADDR_COMPLETE: // Main CPU has finished sending the address to the far end
          if(this->_mh_state == MHS_WAIT_SEND_ADDRESS_INFO) {
            this->_mh_state_advance = REG_OUTGOING_ADDR_COMPLETE;
          }
          break;

        default:
          break;
      }
    this->_i2c_read_data_ready = false; // Request processed

    interrupts();
  }
}

} // End Namespace ENM