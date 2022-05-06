#include "NEET_RF24.h"

/**
*   Constructor for the NEET specific RF24 library with helper functions
* 
*   @param ce_pin  chip enable pin used
*   @param csn_pin chip select pin used
*   @param channel Channel to transmit on, must match between transmitter and receiver pair
*   @param isTransmit true if this is a transmitter
**/
NEET_RF24::NEET_RF24(uint8_t ce_pin, uint8_t csn_pin, uint8_t channel, bool is_transmit){
  _radio = RF24(ce_pin, csn_pin);
  _is_transmit = is_transmit;
  _channel = channel;

  _rx_cur_input = ControlInput();
  _rx_last_packet_time = 0;
  _rx_has_ack_payload = false;
  _rx_telem_length = 0;
  _rx_telem_idx = 0;
  _rx_last_restart_time = 0;
  _rx_telem_payload = Telemetry();

  _tx_telem_idx = 0;
  _tx_has_telem = false;  
  _tx_telem_length = 0;
  _tx_telem_payload = Telemetry();
}

/** 
*   Initializes radio and configures it to operate in its give role (either transmitter or receiver)
*
*   @returns true if radio is successfully initalized
**/
bool NEET_RF24::begin(){
  _rx_last_restart_time = millis();
  bool success = _radio.begin();
  if (!success) {
    return false;
  }

  // Allows dynamic ack payloads for telemetry
  _radio.enableDynamicPayloads();
  _radio.enableAckPayload();
  _radio.setChannel(_channel);
  _radio.setDataRate(DATA_RATE);

  // Sets up correct reading and writing pipes depending on role
  _radio.openWritingPipe(_address[_is_transmit]);
  _radio.openReadingPipe(1, _address[!_is_transmit]);

  _radio.setPALevel(RADIO_POWER_LVL);

  if (!_is_transmit){
    _radio.startListening();
  } else {
    _radio.stopListening();
  }

  return success;
}

/**
*   Checks the radio for incoming messages and updates timeouts
*   Used only for receivers
* 
*   @returns true if the current input is sent within RECEIVER_TIMEOUT_MSEC
**/
bool NEET_RF24::rxUpdate(){
  uint8_t pipe;
  if (_radio.available(&pipe)) {
    
    _radio.read(&_rx_cur_input, sizeof(_rx_cur_input));

    uint8_t bytes = _radio.getDynamicPayloadSize();
    if (bytes != sizeof(_rx_cur_input)){
      // Something is wrong, maybe packet corrupted? 
      return false;
    }

    _rx_last_packet_time = millis();

    if (_rx_has_ack_payload){
      // Assemble payload
      uint8_t end_idx = min(_rx_telem_length, _rx_telem_idx + sizeof(_rx_telem_payload.msg));      
      uint8_t msg_len = end_idx - _rx_telem_idx;
      strncpy(_rx_telem_payload.msg, _rx_telem + _rx_telem_idx, msg_len);
      _rx_telem_payload.length = msg_len;
      _rx_telem_payload.eol = end_idx == _rx_telem_length;

      _rx_telem_idx += msg_len;
      _rx_has_ack_payload = !_rx_telem_payload.eol;

      // Send to radio
      _radio.writeAckPayload(1, &_rx_telem_payload, sizeof(_rx_telem_payload));
    }
    return true;
  }
  bool timeout = millis() - _rx_last_packet_time > RECEIVER_TIMEOUT_MSEC;

  if (timeout && (millis() - _rx_last_restart_time > RECEIVER_RESTART_MSEC)){
    // Serial.println("Failure detected... restarting");
    begin();
    return false;
  }

  return !timeout;
}

/**
*   Gets the most recent values from the controller, need to call rxUpdate() to fetch
*
*   @returns most recent ControlInput value
**/
ControlInput NEET_RF24::rxGetInput(){
  return _rx_cur_input;
}

/**
*   Populates the telemetry string to be sent using ack payloads
*   
*   @param s String telemetry, should be under MAX_TELEM_STRING_LEN
*   @param override bool, if true it will start sending regardless of if rx_has_ack_payload
*   @returns true if s is within length limit and radio is not already sending a telemetry msg
**/
bool NEET_RF24::rxSendTelemetry(String s, bool override){
  if (_rx_has_ack_payload && !override){
    return false;
  }
  if (s.length() > MAX_TELEM_STRING_LEN){
    s = "Telemetry message too long!";
  }
  s += '\n';
  // Populate internal telemetry buffer
  _rx_telem_length = s.length() + 1; // Add 1 to include null terminator
  s.toCharArray(_rx_telem, sizeof(_rx_telem)); // Copy content to _rx_telem

  _rx_telem_idx = 0;
  _rx_has_ack_payload = true; // Set flag to start transmitting via ack payload

  rxUpdate();
  
  return true;  
}


/**
*   Sends a transmission with given ControlInput
*   
*   @param in ControlInput object to send
*   @returns true if the transmission was successful
**/
bool NEET_RF24::txSendControlInput(ControlInput in){
  bool success = _radio.write(&in, sizeof(in));

  if (!success){
    return success;
  }

  uint8_t pipe;
  // If there's a ack payload attached
  if (_radio.available(&pipe)){
    _radio.read(&_tx_telem_payload, sizeof(_tx_telem_payload));

    if (_tx_has_telem || _tx_telem_idx + _tx_telem_payload.length > sizeof(_tx_telem)){
      // Something is wrong, missed a eol or currently already has telem ready
      // Reset by starting over
      _tx_telem_idx = 0;
    }

    // Copy message content to buffer
    strncpy(_tx_telem + _tx_telem_idx, _tx_telem_payload.msg, _tx_telem_payload.length);
    _tx_telem_idx += _tx_telem_payload.length;
    _tx_has_telem = _tx_telem_payload.eol;

    // Populate _tx_telem_length for reading
    if (_tx_has_telem) {
      _tx_telem_length = _tx_telem_idx;
      _tx_telem_idx = 0;
    }
  } else {    // If it's a regular ack packet, handles loss of eol packet
    if (_tx_telem_idx != 0) {
      _tx_has_telem = true;
      _tx_telem_length = _tx_telem_idx;
      _tx_telem_idx = 0;
    }
  }
  return success;
}

/**
*   Puts telemetry into the buffer given if there is a new complete telemetry message
*
*   @param buf pointer to a character buffer with a large size
*   @returns number of characters in the telemetry message, 0 if no message
**/
uint8_t NEET_RF24::txGetTelemetry(char* buf){
  if (_tx_has_telem){
    strncpy(buf, _tx_telem, _tx_telem_length);
    uint8_t ret_len = _tx_telem_length;
    _tx_telem_length = 0; 
    _tx_has_telem = false;

    return ret_len;
  }
  
  return 0;
}

/**
*   Sets the channel if this is a transmitter
*
*   @param channel Channel as a uint8_t
**/
void NEET_RF24::txSetChannel(uint8_t channel){
  if (_is_transmit){
    _channel = channel;
    _radio.setChannel(_channel);
  }
}