// sx126x.h - RadioLib wrapper for RNode firmware
#ifndef SX126X_H
#define SX126X_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define MODE_STDBY_RC_6X 0x00
#define MODE_STDBY_XOSC_6X 0x01
#define MODE_FALLBACK_STDBY_RC_6X 0x20

#define IRQ_RX_DONE_MASK_6X 0x40
#define IRQ_PREAMBLE_DET_MASK_6X 0x10

#define SYNC_WORD_6X  0x1424

class sx126x {
public:
    sx126x(int ss = 10, int rst = 9, int dio0 = 2, int busy = 3, int rxen = -1);
    ~sx126x();

    bool begin(long frequency);
    void end();
    
    void setFrequency(long frequency);
    uint32_t getFrequency();
    
    void setTxPower(int level);
    uint8_t getTxPower();
    
    void setSpreadingFactor(int sf);
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int cr);
    void enableLowDataRateOptimization();
    void disableLowDataRateOptimization();
    void setPreambleLength(long preamble);
    void setSyncWord(uint16_t sw);
    void setPacketParams(int crc = 1, int implicitHeader = 0, int payloadLength = 255);
    void enableCrc();
    void disableCrc();
    bool enableTCXO();
    
    void calibrate_image(long frequency);
    void loraMode();
    void calibrate();
    
    void setRxTxFallbackMode(uint8_t mode);
    
    int beginPacket(int implicitHeader = 0);
    int endPacket(bool wait = true);
    size_t write(uint8_t byte);
    size_t write(const uint8_t *buffer, size_t size);
    
    void onReceive(void(*callback)(int));
    void receive();
    
    void standby();
    void sleep();
    void reset();
    
    int getRSSI();
    float getSNR();
    
    static void handleDio0Rise();
    
    int currentRssi();
    void startCarrier();
    bool carrierDetected();
    void stopCarrier();
    
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
    bool preInit();
};

extern sx126x sx126x_modem;

#endif