// sx126x.h - RadioLib wrapper for RNode firmware
#ifndef SX126X_H
#define SX126X_H

#include <Arduino.h>
#include <SPI.h>

// RNode includes
#include "Boards.h"

// RadioLib includes
#include <RadioLib.h>
#include <modules/SX126x/SX1262.h>

// Constants
#define MAX_PKT_LENGTH 255
#define MODE_STDBY_XOSC_6X 0x01
#define SYNC_WORD_6X 0x1424

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2
#define LORA_DEFAULT_RXEN_PIN  -1
#define LORA_DEFAULT_TXEN_PIN  -1
#define LORA_DEFAULT_BUSY_PIN  -1

// needed for Utilities.h
#define PA_OUTPUT_PA_BOOST_PIN 1
#define PA_OUTPUT_RFO_PIN      0

class sx126x : public Modem {
public:
    sx126x(int ss = 10, int rst = 9, int dio0 = 2, int busy = 3, int rxen = -1);
    ~sx126x();
    
    bool preInit();
    bool begin(long frequency);
    void end();
    
	void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN, int busy = LORA_DEFAULT_BUSY_PIN, int rxen = LORA_DEFAULT_RXEN_PIN);
	
    int beginPacket(int implicitHeader = 0);
    int endPacket(bool wait = true);

    int packetRssi();
    int packetRssi(uint8_t pkt_snr_raw);
    int packetSnrRaw();
    //needed¿
    int getRSSI();
    float getSNR();

    void setFrequency(long frequency);
    uint32_t getFrequency();
    
    void setTxPower(int level, int outputPin);
    uint8_t getTxPower();
    
    void setSpreadingFactor(int sf);
	long getSignalBandwidth();
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int cr);
    void setPreambleLength(long preamble);
    void setSyncWord(uint16_t sw);
    bool dcd();
    void setPacketParams(int crc = 1, int implicitHeader = 0, int payloadLength = 255);
    void enableCrc();
    void disableCrc();
    
	virtual size_t write(uint8_t byte);
	virtual size_t write(const uint8_t *buffer, size_t size);
    
    void onReceive(void(*callback)(int));
    void receive(int size = 0); //REVIEW
    
    void standby();
    void sleep();
    void reset();

    
    int currentRssi();
    bool carrierDetected();
    
    int available();
    int read();
    int peek();
    void flush();

private:
    int _ss;
    int _reset;
    int _dio0;
    int _busy;
    int _rxen;
    void handleDio0Rise();
    
    long _frequency;
    int _txp;
    uint8_t _sf;
    uint8_t _bw;
    uint8_t _cr;
    uint8_t _ldro;
    size_t _packetIndex;
    long _preambleLength;
    uint8_t _implicitHeaderMode;
    uint8_t _payloadLength;
    uint8_t _crcMode;
    int _fifo_tx_addr_ptr;
    int _fifo_rx_addr_ptr;
    
    uint8_t _packet[256];
    bool _preinit_done;
    bool _receiving;
    bool _tcxo;
    bool _radio_online;
    
    void (*_onReceive)(int);
    
    Module* radioModule;
    SX1262* radio;
};

extern sx126x sx126x_modem;

#endif