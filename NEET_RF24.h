#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define RECEIVER_TIMEOUT_MSEC 100         // msecs without a new packet before receiver enters failsafe
#define RECEIVER_RESTART_MSEC 200
#define CONTROLLER_RATE_HZ 100             // Rate to send control inputs in Hz
#define DATA_RATE RF24_1MBPS              // Data rate of the module, lower means longer time but more reliability
#define RADIO_POWER_LVL RF24_PA_HIGH   // Power level of the receiver on board
#define MAX_TELEM_STRING_LEN 63           // Length of message not including null terminator

struct ControlInput {
  int8_t j1PotX;
  int8_t j1PotY;
  bool j1Button;
  int8_t j2PotX;
  int8_t j2PotY;
  bool j2Button;
  int8_t pot;
  bool tSwitch;
  bool button1;
  bool button2;
  bool button3;
  bool button4;
};

struct Telemetry {
  char msg[30];
  uint8_t length;
  bool eol;
};

class NEET_RF24{

  public:
    NEET_RF24(uint8_t ce_pin, uint8_t csn_pin, uint8_t channel, bool is_transmit = false);
    bool begin();

    bool rxUpdate();
    ControlInput rxGetInput();
    bool rxSendTelemetry(String s, bool override = false);

    void txSetChannel(uint8_t channel);
    bool txSendControlInput(ControlInput in);
    uint8_t txGetTelemetry(char* buf);

  private:
    RF24 _radio;
    bool _is_transmit;
    int _ce_pin;
    int _csn_pin;
    uint8_t _channel;
    uint8_t _address[2][6] = {"1Node", "2Node"};

    ControlInput _rx_cur_input;
    unsigned long _rx_last_packet_time;
    bool _rx_has_ack_payload;
    char _rx_telem[MAX_TELEM_STRING_LEN+1] = "";
    uint8_t _rx_telem_length;
    uint8_t _rx_telem_idx;
    unsigned long _rx_last_restart_time;
    Telemetry _rx_telem_payload;

    char _tx_telem[MAX_TELEM_STRING_LEN+1] = "";
    uint8_t _tx_telem_idx;
    uint8_t _tx_telem_length;
    bool _tx_has_telem;
    Telemetry _tx_telem_payload;
};