// sx126x.cpp - RadioLib wrapper implementation
// Task: move all top RadioLib
// Need some ♥
// https://github.com/jgromes/RadioLib/blob/master/src/modules/SX126x/SX1262.cpp
// https://github.com/jgromes/RadioLib/blob/master/src/modules/SX126x/SX126x.cpp
// https://github.com/jgromes/RadioLib/blob/master/src/modules/SX126x/SX126x_config.cpp
//
#include "sx126x.h"

#if MCU_VARIANT == MCU_ESP32
  #if MCU_VARIANT == MCU_ESP32 and !defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "soc/rtc_wdt.h"
  #endif
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

#if BOARD_MODEL == BOARD_TECHO || BOARD_MODEL == BOARD_PROMICRO
  SPIClass spim3 = SPIClass(NRF_SPIM3, pin_miso, pin_sclk, pin_mosi) ;
  #define SPI spim3

#elif defined(NRF52840_XXAA)
  extern SPIClass spiModem;
  #define SPI spiModem
#endif

extern SPIClass SPI;

#define MAX_PKT_LENGTH 255

sx126x sx126x_modem;

sx126x::sx126x(int ss, int rst, int dio0, int busy, int rxen)
  : _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN), _busy(LORA_DEFAULT_BUSY_PIN), _rxen(LORA_DEFAULT_RXEN_PIN),
    _frequency(0),
    _txp(0),
    _sf(0x07),
    _bw(0x04),
    _cr(0x01),
    _ldro(0x00),
    _packetIndex(0),
    _preambleLength(18),
    _implicitHeaderMode(0),
    _payloadLength(255),
    _crcMode(1),
    _fifo_tx_addr_ptr(0),
    _fifo_rx_addr_ptr(0),
    _preinit_done(false),
    _receiving(false),
    _tcxo(false),
    _radio_online(false),
    _onReceive(NULL),
    radioModule(nullptr),
    radio(nullptr)
{ setTimeout(0); }


void sx126x::setPins(int ss, int reset, int dio0, int busy, int rxen) {
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
  _busy = busy;
  _rxen = rxen;
}

bool sx126x::preInit() {
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    
	/* RadioLib handles
    #if BOARD_MODEL == BOARD_T3S3 || BOARD_MODEL == BOARD_HELTEC32_V3 || BOARD_MODEL == BOARD_HELTEC32_V4 || BOARD_MODEL == BOARD_TDECK || BOARD_MODEL == BOARD_XIAO_S3
        SPI.begin(pin_sclk, pin_miso, pin_mosi, pin_cs);
    #elif BOARD_MODEL == BOARD_TECHO
        SPI.setPins(pin_miso, pin_sclk, pin_mosi);
        SPI.begin();
    #else
        SPI.begin();
    #endif
    */
	
    return true;
}

/*
///RadioLib handles this///
readRegister
writeRegister
singleTransfer
??rxAntEnable
loraMode
waitOnBusy
executeOpcode
executeOpcodeRead
writeBuffer
readBuffer
setModulationParams

stays -- setPacketParams
stays -- reset

calibrate
calibrate_image
*/

bool sx126x::begin(long frequency) {
    int res;
    if (!_preinit_done) {
        if (!preInit()) {
            return false;
        }
    }
    
    if (_busy != -1) { pinMode(_busy, INPUT); }
    if (!_preinit_done) { if (!preInit()) { return false; } }
    if (_rxen != -1) { pinMode(_rxen, OUTPUT); }
    
    // Create Module and SX1262 instances
    Module* radioModule = new Module(_ss, _dio0, _reset, _busy, SPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));
    SX1262* radio = new SX1262(radioModule);
    
    // Setup standby mode
    //?? radio->standby();
    
    // Set SX1262 to LoRa mode with default parameters
    /*
    _sf = 0x07;       // SF7
    _bw = 0x04;       // 125 kHz
    _cr = 0x01;       // CR 4/5
    _ldro = 0x00;     // LDRO disabled
    */
    
    // Setup radio with frequency and default parameters
    //begin
    // https://jgromes.github.io/RadioLib/class_s_x1262.html#a9ceab9913d102c2fd657a1a91afaf9cc
    // LDRO disabled by default?? otherwise use forceLDRO(bool enable)
    // we'll set it auto
    int state = radio->begin(
        (float)frequency / 1e6,  // Convert Hz to MHz
        (float)_bw/ 1000.0f,                     // Bandwidth in kHz
        7,                       // Spreading factor
        5,                       // Coding rate 4/5
        SYNC_WORD_6X,            // Sync word
        2,                       // Power
        _preambleLength,         // Preamble length
        _tcxo ? 1.8 : 0.0,       // TCXO voltage
        true                     // Use DCDC instead of LDO (Make it optional)
    );
    Serial.write("Radio initialisation successful.\r\n");
    Serial.printf("Configured frequency to: %d MHZ\n", (float)frequency / 1e6);
    Serial.printf("Configured Bandwidth to: %d kHz\n", (float)_bw/ 1000.0f);
    
    if (state != RADIOLIB_ERR_NONE) {
        return false;
    }

    //OCP to 140 (defaults 60)
    res = radio->setCurrentLimit(140.0);
    Serial.printf("Current limit set result %d", res);

    //Does RadioLib handle - setRxTxFallbackMode??
    
    // Setup DIO2 as RF switch if needed
    #if DIO2_AS_RF_SWITCH
        radio->setDio2AsRfSwitch();
    #endif

    radio->setRfSwitchPins(_rxen, RADIOLIB_NC);

    //RxGain
    radio->setRxBoostedGainMode(true);

    //LDRO will be enabled automatically when symbol length exceeds 16 ms
    radio->autoLDRO();
  
    standby();
    //enableCrc();

    //setModulationParams(_sf, _bw, _cr, _ldro);
    // OLD _preambleLength is handled on begin()
    // setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
    // NEW
    setPacketParams(_crcMode, _implicitHeaderMode, _payloadLength);

    #if HAS_LORA_PA
        #if LORA_PA_GC1109
        // Enable Vfem_ctl for supply to
        // PA power net.
        pinMode(LORA_PA_PWR_EN, OUTPUT);
        digitalWrite(LORA_PA_PWR_EN, HIGH);

        // Enable PA LNA and TX standby
        pinMode(LORA_PA_CSD, OUTPUT);
        digitalWrite(LORA_PA_CSD, HIGH);

        // Keep PA CPS low until actual
        // transmit. Does it save power?
        // Who knows? Will have to measure.
        // Note from the future: Nope.
        // Power consumption is the same,
        // and turning it on and off is
        // not something that it likes.
        // Keeping it high for now.
        pinMode(LORA_PA_CPS, OUTPUT);
        digitalWrite(LORA_PA_CPS, HIGH);

        // On Heltec V4, the PA CTX pin
        // is driven by the SX1262 DIO2
        // pin directly, so we do not
        // need to manually raise this.
        #endif
    #endif
    
    _frequency = frequency;
    _radio_online = true;
    _preinit_done = true;
    Serial.write("Done!!.\r\n");
    
    return true;
}

void sx126x::end() {
    if (radio) {
        radio->sleep();
    }
    if (radioModule) {
        delete radio;
        delete radioModule;
        radio = nullptr;
        radioModule = nullptr;
    }
    _preinit_done = false;
    _radio_online = false;
}

int sx126x::beginPacket(int implicitHeader) {
    if (!_radio_online) return 0;
    
    // Set header type for this packet
    if (implicitHeader) {
        radio->implicitHeader(_payloadLength);
    } else {
        radio->explicitHeader();
    }
    
    _packetIndex = 0;
    return 1;
}

int sx126x::endPacket(bool wait) {
    if (!_radio_online || _packetIndex == 0) return 0;
    
    // RadioLib should handle RF switching for transmit
    
    int state = radio->transmit(_packet, _packetIndex);
    
    // Return to receive mode
    receive();
    
    return (state == RADIOLIB_ERR_NONE) ? 1 : 0;
}

unsigned long preamble_detected_at = 0;
extern long lora_preamble_time_ms;
extern long lora_header_time_ms;
bool false_preamble_detected = false;

bool sx126x::dcd() {
  //TODO: Is there a better way???
  uint16_t irq = radio->getIrqFlags();
  uint32_t now = millis();

  bool header_detected = false;
  bool carrier_detected = false;

  if ((irq & RADIOLIB_IRQ_HEADER_VALID) != 0) { header_detected = true; carrier_detected = true; }
  else { header_detected = false; }

  if ((irq & RADIOLIB_IRQ_PREAMBLE_DETECTED) != 0) {
    carrier_detected = true;
    if (preamble_detected_at == 0) { preamble_detected_at = now; }
    if (now - preamble_detected_at > lora_preamble_time_ms + lora_header_time_ms) {
      preamble_detected_at = 0;
      if (!header_detected) { false_preamble_detected = true; }
      radio->clearIrqFlags(RADIOLIB_IRQ_PREAMBLE_DETECTED);
    }
  }

  // TODO: Maybe there's a way of unlatching the RSSI
  // status without re-activating receive mode?
  if (false_preamble_detected) { receive(); false_preamble_detected = false; }
  return carrier_detected;
}


//removed currentRssiRaw; packetRssiRaw;  ; packetSnr ;packetFrequencyError

//need packetRssi, packetSnrRaw
int sx126x::packetSnrRaw() {
  /*
  if (!_radio_online) return 0;  
  uint32_t packetStatus = radio->getPacketStatus();
  uint8_t snrRaw = (packetStatus >> 8) & 0xFF; // Second byte contains SNR  
  */
  return getSNR();
}

int sx126x::packetRssi() {
  if (!_radio_online) return 0.0;
  uint8_t rssiRaw = getRSSI(); // First byte contains RSSI
  return -(int)rssiRaw / 2;
}

int sx126x::packetRssi(uint8_t pkt_snr_raw) {
  // OLD TODO: May need more calculations here
  return packetRssi();
}

int sx126x::currentRssi() {
    if (!_radio_online) return 0;
    return (int)radio->getRSSI(true); // instantaneous RSSI
}

int sx126x::getRSSI() {
    if (!_radio_online) return 0;
    return (int)radio->getRSSI();
}

float sx126x::getSNR() {
    if (!_radio_online) return 0.0;
    return radio->getSNR();
}

void sx126x::setFrequency(long frequency) {
  //frequency is in hz ie: 869525000
    if (!_radio_online) return;
    
    // Store the frequency in Hz as expected by RNode
    _frequency = frequency;
    
    // Convert Hz to MHz for RadioLib (divide by 1,000,000)
    float freq_mhz = (float)frequency / 1000000.0f;
    radio->setFrequency(freq_mhz);
}

uint32_t sx126x::getFrequency() {
    return _frequency;
}

void sx126x::setTxPower(int level, int outputPin) {
    if (!_radio_online) return;
    _txp = level;
    radio->setOutputPower(level);
}

uint8_t sx126x::getTxPower() {
    return _txp;
}

long sx126x::getSignalBandwidth() {
	return (float)_bw;
}

void sx126x::setSpreadingFactor(int sf) {
    if (!_radio_online) return;
    _sf = sf;
    radio->setSpreadingFactor(sf);
}

void sx126x::setSignalBandwidth(long sbw) {
    if (!_radio_online) return;
    
    // Store the bandwidth setting as used by RNode
    _bw = (uint8_t)sbw;
    
    // Convert Hz to kHz for RadioLib (divide by 1000)
    float bandwidth_khz = (float)sbw / 1000.0f;
    
    radio->setBandwidth(bandwidth_khz);
}

void sx126x::setCodingRate4(int cr) {
    if (!_radio_online) return;
    _cr = cr;
    radio->setCodingRate(cr + 4); // RNode uses 1-4, RadioLib uses 5-8
}

/*
void sx126x::enableLowDataRateOptimization() {
    if (!_radio_online) return;
    _ldro = 0x01;
    radio->forceLDRO(true);
}

void sx126x::disableLowDataRateOptimization() {
    if (!_radio_online) return;
    _ldro = 0x00;
    radio->forceLDRO(false);
}
*/

void sx126x::setPreambleLength(long preamble) {
    if (!_radio_online) return;
    _preambleLength = preamble;
    radio->setPreambleLength(preamble);
}

void sx126x::setSyncWord(uint16_t sw) {
    if (!_radio_online) return;
    // SX1262 only uses the upper byte for LoRa sync word
    radio->setSyncWord((uint8_t)(sw >> 8));
}

void sx126x::setPacketParams(int crc, int implicitHeader, int payloadLength) {
    if (!_radio_online) return;
    _crcMode = crc;
    _implicitHeaderMode = implicitHeader;
    _payloadLength = payloadLength;
    
    //Radiolib set its on by default and RNode looks like never disables it
    //if it fail i'll comment it
    radio->setCRC(crc == 1);
    
    if (implicitHeader) {
        radio->implicitHeader(_payloadLength);
    } else {
        radio->explicitHeader();
    }
}

void sx126x::enableCrc() {
    _crcMode = 1;
    setPacketParams(_crcMode, _implicitHeaderMode, _payloadLength);
}

void sx126x::disableCrc() {
    _crcMode = 0; 
    setPacketParams(_crcMode, _implicitHeaderMode, _payloadLength);
}

size_t sx126x::write(uint8_t byte) {
    return write(&byte, sizeof(byte));
}

size_t sx126x::write(const uint8_t *buffer, size_t size) {
    if (!_radio_online) return 0;
    
    size_t remaining = sizeof(_packet) - _packetIndex;
    size_t toWrite = (size < remaining) ? size : remaining;
    
    if (toWrite > 0) {
        memcpy(&_packet[_packetIndex], buffer, toWrite);
        _packetIndex += toWrite;
    }
    
    return toWrite;
}

void sx126x::onReceive(void(*callback)(int)) {
    _onReceive = callback;
    if (callback) {
		//TODO - do we need this?
        pinMode(_dio0, INPUT);
        radio->setDio1Action(handleDio0Rise);
        receive();
    } else {
        radio->clearDio1Action();
    }
}

//REVIEW
void sx126x::receive(int size) {
    if (!_radio_online) return;
    
    // Handle RF switching for receive
	/*
    if (_rxen != -1) {
        digitalWrite(_rxen, HIGH); // Enable RX
    }
	*/
    
    radio->startReceive();
}

void sx126x::standby() {
    if (!_radio_online) return;
    radio->standby();
}

void sx126x::sleep() {
    if (!_radio_online) return;
    radio->sleep();
}

//removed enableTCXO; disableTCXO

void sx126x::reset() {
    if (!_radio_online) return;
    radio->reset();
}

void sx126x::handleDio0Rise() {
    if (!_radio_online || !_onReceive) return;
    
    // Get IRQ status
    uint16_t irq = radio->getIrqFlags();
    
    if (irq & RADIOLIB_IRQ_RX_DONE) {
        // Clear IRQ flags
        radio->clearIrqFlags(RADIOLIB_IRQ_RX_DONE | RADIOLIB_IRQ_CRC_ERR | RADIOLIB_IRQ_HEADER_ERR);
        
        // Read packet data
        size_t len = radio->getPacketLength();
        if (len > sizeof(_packet)) len = sizeof(_packet);
        
        int state = radio->readData(_packet, len);
        if (state == RADIOLIB_ERR_NONE) {
            _packetIndex = len;
            _receiving = true;
            
            // Call the receive callback
            _onReceive(len);
            
            _receiving = false;
        }
    }
}

int sx126x::available() {
    return _receiving ? (_packetIndex) : 0;
}

int sx126x::read() {
    if (!_receiving || _packetIndex == 0) return -1;
    
    int result = _packet[0];
    _packetIndex--;
    
    // Shift remaining bytes
    if (_packetIndex > 0) {
        memmove(_packet, &_packet[1], _packetIndex);
    }
    
    return result;
}

int sx126x::peek() {
    if (!_receiving || _packetIndex == 0) return -1;
    return _packet[0];
}

void sx126x::flush() {
    _packetIndex = 0;
}