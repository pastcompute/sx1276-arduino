/*
  Copyright (c) 2015 Andrew McDonnell <bugs@andrewmcdonnell.net>

  This file is part of SentriFarm Radio Relay.

  SentriFarm Radio Relay is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  SentriFarm Radio Relay is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with SentriFarm Radio Relay.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "sx1276reg.h"
#include "sx1276.h"
#include <SPI.h>
#if defined(ESP8266)
#include <ets_sys.h>
#else
#define ICACHE_FLASH_ATTR
#endif

#include "elapsedMillis.h"

// Somewhat arbitrary starting point
#define DEFAULT_BW_HZ 125000
#define DEFAULT_SPREADING_FACTOR 9
#define DEFAULT_CODING_RATE 6

#define SPI_CS_DELAY_US 5
#define TOA_TX_MARGIN_MS 5

#define VERBOSE 1
#define VERBOSE_W 0
#define VERBOSE_R 0
#define VERBOSE_TX_TIMING 0


#ifdef TEENSYDUINO
#define Serial Serial1
#endif

#if VERBOSE
#include <stdio.h>
#define DEBUG(x ...) { char buf[128]; snprintf(buf, sizeof(buf), x); Serial.print(buf); }
#else
#define DEBUG(x ...)
#endif

#undef PASTE
#define PASTE(x, y) x ## y
#define BW_TO_SWITCH(number) case number : return PASTE(SX1276_LORA_BW_, number)
#define BW_FR_SWITCH(number) case PASTE(SX1276_LORA_BW_, number) : return number;

inline byte BandwidthToBitfield(unsigned bandwidth_hz)
{
  switch (bandwidth_hz) {
    BW_TO_SWITCH(7800);
    BW_TO_SWITCH(10400);
    BW_TO_SWITCH(15600);
    BW_TO_SWITCH(20800);
    BW_TO_SWITCH(31250);
    BW_TO_SWITCH(41700);
    BW_TO_SWITCH(62500);
    BW_TO_SWITCH(125000);
    BW_TO_SWITCH(250000);
    BW_TO_SWITCH(500000);
    default:
      // Error
      // Note we try and avoid this by validating inputs to methods
      return 0;
  }
}

inline unsigned BitfieldToBandwidth(byte bitfield)
{
  switch (bitfield) {
    BW_FR_SWITCH(7800);
    BW_FR_SWITCH(10400);
    BW_FR_SWITCH(15600);
    BW_FR_SWITCH(20800);
    BW_FR_SWITCH(31250);
    BW_FR_SWITCH(41700);
    BW_FR_SWITCH(62500);
    BW_FR_SWITCH(125000);
    BW_FR_SWITCH(250000);
    BW_FR_SWITCH(500000);
    default: return 0;
  }
}

inline unsigned CodingRateToBitfield(byte coding_rate)
{
  switch(coding_rate) {
    case 5: return SX1276_LORA_CODING_RATE_4_5;
    case 6: return SX1276_LORA_CODING_RATE_4_6;
    case 7: return SX1276_LORA_CODING_RATE_4_7;
    case 8: return SX1276_LORA_CODING_RATE_4_8;
    default:
      // Error
      // Note we try and avoid this by validating inputs to methods
      return 0;
  }
}

ICACHE_FLASH_ATTR
SX1276Radio::SX1276Radio(int cs_pin, const SPISettings& spi_settings, bool inAir9b)
  : cs_pin_(cs_pin),
    spi_settings_(spi_settings),
    inAir9b_(inAir9b),
    symbol_timeout_(366), // in theory 3s, empirically 1.51s @ sf9 and bw 125000 although often, shorter maybe due to esp8266 timekeeping wierdness not accurately recording elapased time
    preamble_(0x8),
    max_tx_payload_bytes_(0x80),
    max_rx_payload_bytes_(0x80),
    bandwidth_hz_(DEFAULT_BW_HZ),
    bandwidth_idx_(0xff),
    spreading_factor_(DEFAULT_SPREADING_FACTOR),
    coding_rate_(DEFAULT_CODING_RATE),
    carrier_hz_(0),
    lna_gain_(SX1276_LORA_LNA_GAIN_G4),
    rssi_dbm_(-255),
    rx_snr_db_(-255),
    rx_warm_(false),
    tx_warm_(false),
    dead_(true),
    cached_tx_payload_length_(0),
    cached_tx_toa_(0),
    rxPollFunction(NULL)
{
  // Note; we want DEBUG ( Serial) here because this happens before Serial is initialised,
  // and it hangs the ESP8266
  // DEBUG("SX1276: CS pin=%d\n", cs_pin_);
}

//ICACHE_FLASH_ATTR
void SX1276Radio::ReadRegister(byte reg, byte& result)
{
  SPI.beginTransaction(spi_settings_);
  digitalWrite(cs_pin_, LOW);
  SPI.transfer(reg);
  result = SPI.transfer(0);
  digitalWrite(cs_pin_, HIGH);
  SPI.endTransaction();
#if VERBOSE_R
  if (reg != SX1276REG_IrqFlags) {
    DEBUG("[R] %02x --> %02x\n\r", reg, result);
  }
#endif
}

//ICACHE_FLASH_ATTR
inline byte SX1276Radio::DoRegister(byte reg, byte val) const
{
  digitalWrite(cs_pin_, LOW);
  delayMicroseconds(SPI_CS_DELAY_US);
  SPI.transfer(reg);
  byte result = SPI.transfer(val);
  digitalWrite(cs_pin_, HIGH);
  return result;
}


//ICACHE_FLASH_ATTR
void SX1276Radio::WriteRegister(byte reg, byte val, byte& result, bool verify)
{
  SPI.beginTransaction(spi_settings_);
  result = DoRegister(reg + SX1276_SPI_WRITE_MASK, val);
  SPI.endTransaction();
#if VERBOSE_W
  DEBUG("[W] %02x <-- %02x --> %02x\n\r", reg, val, result);
#endif
  if (verify) {
    delay(10);
    byte newval = 0;
    ReadRegister(reg, newval);
    if (newval != val) {
      DEBUG("[W] %02x <- %02x failed, got %02x\n\r", reg, val, newval);
    }
  }
}

//ICACHE_FLASH_ATTR
void SX1276Radio::WriteBulk(byte reg, const byte *val, byte count)
{
  SPI.beginTransaction(spi_settings_);
  for (byte n=0; n < count; n++) {
    DoRegister(reg + SX1276_SPI_WRITE_MASK, val[n]);
    delayMicroseconds(SPI_CS_DELAY_US);
  }
  SPI.endTransaction();
#if VERBOSE_W
  DEBUG("[WB] %02x <-- (%d bytes)\n\r", reg, count);
#endif
}

ICACHE_FLASH_ATTR
byte SX1276Radio::ReadVersion()
{
  byte v;
  ReadRegister(SX1276REG_Version, v);
  return v;
}

ICACHE_FLASH_ATTR
void SX1276Radio::ConfigureBandwidth() {
  // IMPORTANT: Testing of 2015-09-13 was accidentally done using 4/5
  // because we forgot to shift the CR. Which also explains why we never get CRC errors!
  // because implicit header could have been on
  // 125kHz, 4/6, explicit header
  const byte bwb = BandwidthToBitfield(bandwidth_hz_);
  byte v = (bwb << 4) | (CodingRateToBitfield(coding_rate_) << 1) | 0x0; // 0x0 --> Explicit header mode
  WriteRegister(SX1276REG_ModemConfig1, v);
  bandwidth_idx_ = bwb;

  // Errata - sensitivity optimisation
  if (bwb == SX1276_LORA_BW_500000) {
    WriteRegister(SX1276REG_RegSeqConfig1, 0x02);
    WriteRegister(SX1276REG_RegTimer2Coef, 0x64);
  } else {
    WriteRegister(SX1276REG_RegSeqConfig1, 0x03);
  }
}

ICACHE_FLASH_ATTR
void SX1276Radio::ConfigureGain()
{
  // Setup the LNA:
  // To start with: midpoint gain (gain E G1 (max)..G6 (min)) bits 7-5 and no boost, while we are testing in the lab...
  // Boost, bits 0..1 - 00 = default LNA current, 11 == 150% boost, but we probably dont want that
  WriteRegister(SX1276REG_Lna, ((lna_gain_ & 0x7)<< 5) | (0x00));
}

ICACHE_FLASH_ATTR
void SX1276Radio::ConfigureSpreadingFactor()
{
  byte v = (spreading_factor_ << 4) | (0 << SX1276_BITSHIFT_TX_CONTINUOUS_MODE)| (1 << SX1276_BITSHIFT_RX_PAYLOAD_CRC_ON) | ((symbol_timeout_ >> 8) & 0x03);
  WriteRegister(SX1276REG_ModemConfig2, v);
  v = symbol_timeout_ & 0xff;
  WriteRegister(SX1276REG_SymbTimeoutLsb, v);
}


ICACHE_FLASH_ATTR
bool SX1276Radio::Begin()
{
  byte v;

  // Sleep Mode from any unknown mode: clear lower bits
  ReadRegister(SX1276REG_OpMode, v);
  WriteRegister(SX1276REG_OpMode, v & 0xf8, true);

  // Sleep
  WriteRegister(SX1276REG_OpMode, SX1276_OPMODE_SLEEP, true);

  // LoRa, Standby
  WriteRegister(SX1276REG_OpMode, SX1276_OPMODE_STANDBY, true);

  // Switch to maximum current mode (0x1B == 240mA), and enable overcurrent protection
  WriteRegister(SX1276REG_Ocp, (1<<5) | 0x0B); // 0b is default (100mA), 1b max

  // Verify operating mode
  ReadRegister(SX1276REG_OpMode, v);
  if (v != SX1276_OPMODE_STANDBY) {
    DEBUG("Unable to enter LoRa Standby mode. v=0x%02x\n\r", v);
    return false;
  }
  dead_ = false;

  ConfigureBandwidth();
  ConfigureSpreadingFactor();

  // LED on DIO3 (bits 0..1 of register): Valid header : 01
  // Pin header DIO1 : Rx timeout: 00
  // Pin header DIO0 (bits 6..7) : Tx done: 01
  WriteRegister(SX1276REG_DioMapping1, 0x41);

  // Pin header DIO5 (bits 5-4): Clk Out: 10
  WriteRegister(SX1276REG_DioMapping2, 0x20);

  if (inAir9b_) {
    // with boost off, we lose signal from outside a meter or two... even when "max" power.
    // Close range rx rssi ~ -99 to -115
    // maybe the rubber duckies are no good?
    // boost on and signal is too powerful when at max
    // Close range rx rssi ~ -30 to -40 (power -6) to -45 (power -12) to -48 (power -15)
    WriteRegister(SX1276REG_PaConfig, 0xf3); // inAir9b - max power, enable the boost, backoff max to 1 (almost lowest power)
    WriteRegister(SX1276REG_PaDac, 0x87);
  } else {
    WriteRegister(SX1276REG_PaConfig, 0x7f); // max power, no boost is available
    WriteRegister(SX1276REG_PaDac, 0x84);
  }

  // Preamble size
  WriteRegister(SX1276REG_PreambleLSB, preamble_);

  // If a carrier preset, then enable it immediately
  // Otherwise, we will be at the current hardware default, whatever that is
  if (carrier_hz_ > 0) {
    SetCarrier(carrier_hz_);
  }
  return true;
}

ICACHE_FLASH_ATTR
void SX1276Radio::ReadCarrier(uint32_t& carrier_hz)
{
  uint8_t v;
  uint64_t Frf = 0;
  ReadRegister(SX1276REG_FrfMsb, v);
  Frf = uint32_t(v) << 16;
  ReadRegister(SX1276REG_FrfMid, v);
  Frf = Frf | (uint32_t(v) << 8);
  ReadRegister(SX1276REG_FrfLsb, v);
  Frf = Frf | (uint32_t(v));

  int64_t actual_hz = (32000000ULL * Frf) >> 19;
  carrier_hz = actual_hz;
}

ICACHE_FLASH_ATTR
void SX1276Radio::ReadBandwidth(uint32_t& bandwidth_hz)
{
  byte v;
  ReadRegister(SX1276REG_ModemConfig1, v);
  v = ((v >> 4) & 0x0f);
  bandwidth_hz = BitfieldToBandwidth(v);
  bandwidth_idx_ = v;
}

ICACHE_FLASH_ATTR
void SX1276Radio::ReadSpreadingFactor(byte& sf)
{
  byte v;
  ReadRegister(SX1276REG_ModemConfig2, v);
  sf = spreading_factor_ = ((v >> 4) & 0xf);
}

ICACHE_FLASH_ATTR
void SX1276Radio::SetSpreadingFactor(byte sf)
{
  if (sf < 6) { sf = 7; }
  if (sf > 12) { sf = 7; }
  spreading_factor_ = sf;
  if (dead_) { return; }
  ConfigureSpreadingFactor();
}

ICACHE_FLASH_ATTR
void SX1276Radio::SetBandwidth(byte bwIndex) {
  uint32_t bw = BitfieldToBandwidth(bwIndex);
  if (bw == 0) {
    bwIndex = SX1276_LORA_BW_125000;
    bw = BitfieldToBandwidth(bwIndex);
  }
  bandwidth_hz_ = bw;
  if (dead_) { return; }
  ConfigureBandwidth();
}

ICACHE_FLASH_ATTR
void SX1276Radio::SetLNAGain(byte gain)
{
  if (gain < SX1276_LORA_LNA_GAIN_G1) {
    gain = SX1276_LORA_LNA_GAIN_G1;
  }
  if (gain > SX1276_LORA_LNA_GAIN_G6) {
    gain = SX1276_LORA_LNA_GAIN_G6;
  }
  lna_gain_ = gain;
  if (dead_) { return; }
  ConfigureGain();
}


ICACHE_FLASH_ATTR
void SX1276Radio::SetCarrier(uint32_t carrier_hz)
{
  carrier_hz_ = carrier_hz;
  if (dead_) { return; }
  rx_warm_ = false;
  // Carrier frequency
  // { F(Xosc) x F } / 2^19
  // Resolution is 61.035 Hz | Xosc=32MHz, default F=0x6c8000 --> 434 MHz
  // AU ISM Band >= 915 MHz
  // Frequency latches when Lsb is written
  byte v;
  uint64_t Frf = (uint64_t)carrier_hz * (1 << 19) / 32000000ULL;
  v = Frf >> 16;
  WriteRegister(SX1276REG_FrfMsb, v);
  v = Frf >> 8;
  WriteRegister(SX1276REG_FrfMid, v);
  v = Frf & 0xff;
  WriteRegister(SX1276REG_FrfLsb, v);
}

/// Calcluates the estimated time on air for a given simple payload
/// based on the formulae in the SX1276 datasheet
// Initial enhancement:
// * use BW & SF from last Begin() or last time they changed
//   so would fail if user ever bypasses this class
// Future enhancement:
// * We could calculate 5 constants only when SF or BW changes
//   Pro: reduces CPU cycles each call
//   Con: need 5 extra fields
// * We could cache past results keyed by length
//   Pro: reduces CPU cycles
//   Con: over time might grow up to 128 (or 255) results x (each time BW or SF changes unless invalidated)
ICACHE_FLASH_ATTR
int SX1276Radio::PredictTimeOnAir(byte payload_len) const
{
  // TOA = (6.F+4.25F+8+ceil( (8*payload_len-4*SF+28+16)/(4*SF))*CR) * (1 << SF) / BW;

  // Floating point in the ESP8266 is done in software and is thus expensive

  int N = (payload_len << 3) - 4 * spreading_factor_ + 28 + 16;
  // FUTURE : if header is disabled enabled, k = k - 20;
  int D = 4 * spreading_factor_;
  // FUTURE : if low data rate optimisation enabled, D = D - 2

  // To avoid float, we need to add 1 if there was a remainder
  N = N / D;  if (N % D) { N++; }

  N = N * coding_rate_;
  if (N<0) { N=0; }
  N += 8;

  // N is now set to the number of symbols
  // Add the preamble (preamble + 4.25) then multiply by symbol time Tsym
  // Rsym = BW / 2^Sf  Tsym = 2^sf / BW
  // To avoid float we add them after converting to milliseconds
  int Tpreamble = (1000 * preamble_ + 4250) * (1 << spreading_factor_) / bandwidth_hz_;
  int Tpayload = 1000 * N * (1 << spreading_factor_) / bandwidth_hz_;

  return Tpreamble + Tpayload;
}

ICACHE_FLASH_ATTR
void SX1276Radio::Standby()
{
  WriteRegister(SX1276REG_OpMode, SX1276_OPMODE_STANDBY);
  delay(10);
  rx_warm_ = false;
}

ICACHE_FLASH_ATTR
void SX1276Radio::SetPowerLimit(byte ocpTrim)
{
  // TODO improve checking
  WriteRegister(SX1276REG_Ocp, (1<<5) | (ocpTrim & 0xf));
}

ICACHE_FLASH_ATTR
bool SX1276Radio::TransmitMessage(const void *payload, byte len, bool withStandby)
{
#if VERBOSE_TX_TIMING
  elapsedMillis t0;
#endif
  if (len > max_tx_payload_bytes_) {
    len = max_tx_payload_bytes_;
    DEBUG("MESSAGE TOO LONG! TRUNCATED\n\r");
  }
  // We dont yet support optimal interleaved rx/tx
  rx_warm_ = false;
  // each SPI transaction costs 7ms on an XMC1100
  // so we need to reduce unnecessary ops, and then combine whats left into a single transaction
  // this lowers the time of this function once up and running, down to ~8ms + ToA
  if (withStandby) {
    // LoRa, return to standby mode if was receiving or other mode
    // this is redundant when already in standby mode, (incl. when we just transmitted)
    WriteRegister(SX1276REG_OpMode, SX1276_OPMODE_STANDBY);
    tx_warm_ = false;
    delay(10);
  }

  // Reset the TX FIFO
  const byte FIFO_START = 0xff - max_tx_payload_bytes_ + 1;
  if (!tx_warm_) {
    WriteRegister(SX1276REG_FifoTxBaseAddr, FIFO_START);
    WriteRegister(SX1276REG_MaxPayloadLength, max_tx_payload_bytes_);
    WriteRegister(SX1276REG_IrqFlagsMask, 0xf7); // write a 1 to IRQ to ignore; f7 --> TxDoneMask is only one active
    ClearInterrupts();
  } else {
    // Assume we cleared the IRQ at end of last tx
  }
  // only set payload len when it changes
  if (cached_tx_payload_length_ != len || !tx_warm_) {
    cached_tx_payload_length_ = len;
    cached_tx_toa_ = PredictTimeOnAir(len);
    DEBUG("Update ToA(%d) -> %dms\n\r", len, cached_tx_toa_);
    WriteRegister(SX1276REG_PayloadLength, len);
  }
  tx_warm_ = true;

  // OK, leaky abstraction - fall back to bare SPI code to remove the final ~ 21ms of latency
  byte v;
  SPI.beginTransaction(spi_settings_);
  DoRegister(SX1276REG_FifoAddrPtr + SX1276_SPI_WRITE_MASK, FIFO_START);
  delayMicroseconds(SPI_CS_DELAY_US);
  for (byte n=0; n < len; n++) {
    DoRegister(SX1276REG_Fifo + SX1276_SPI_WRITE_MASK, ((const byte *)payload)[n]);
    delayMicroseconds(SPI_CS_DELAY_US);
  }
  v = DoRegister(SX1276REG_FifoAddrPtr, 0);
  delayMicroseconds(SPI_CS_DELAY_US);
  DoRegister(SX1276REG_OpMode + SX1276_SPI_WRITE_MASK, SX1276_OPMODE_TX);
  delayMicroseconds(SPI_CS_DELAY_US);
#if VERBOSE_W
  DEBUG("[W] %02x <-- %02x\n\r", SX1276REG_FifoAddrPtr, FIFO_START);
  DEBUG("[WB] %02x <-- (%d bytes)\n\r", SX1276REG_Fifo, len);
#endif
#if VERBOSE_R
  DEBUG("[R] %02x --> %02x\n\r", SX1276REG_FifoAddrPtr, v);
#endif
  if (v != FIFO_START + len) {
    // This has been observed in the wild, so leaving this here for now...
    DEBUG("FIFO write pointer mismatch, expected %02x got %02x\n\r", FIFO_START + len, v);
  }
  // Wait until TX DONE, or timeout
  // We make the timeout an estimate based on predicted TOA
  // (An alternative to this polling loop is to have an interrupt instead, but then that depends on circuit)
  bool ok = false;
  uint32_t toa = cached_tx_toa_;
  elapsedMillis tPollStart;
  do {
    v = DoRegister(SX1276REG_IrqFlags, 0);
    if (v & (1<<3)) {
      ok = true;
      break;
    }
    if (tPollStart > (toa + TOA_TX_MARGIN_MS)) { break; }
    delayMicroseconds(SPI_CS_DELAY_US);
  } while (true);
  delayMicroseconds(SPI_CS_DELAY_US);
  DoRegister(SX1276REG_IrqFlags + SX1276_SPI_WRITE_MASK, 0xff);
  delayMicroseconds(SPI_CS_DELAY_US);
  SPI.endTransaction();
#if VERBOSE_R
  DEBUG("[R] %02x --> %02x\n\r", SX1276REG_IrqFlags, v);
#endif
#if VERBOSE_W
  DEBUG("[W] %02x <-- %02x\n\r", SX1276REG_IrqFlags, 0xff);
#endif
#if VERBOSE_TX_TIMING
  DEBUG("%d %d\n\r", (int)t0, (int)tPollStart);
#endif
  // On the way out, we default to staying in LoRa mode
  // the caller can the choose to return to standby
  if (!ok) {
    DEBUG("TX TIMEOUT!\n\r");
  }
  return ok;
}

const uint8_t RX_BASE_ADDR = 0x0;

ICACHE_FLASH_ATTR
void SX1276Radio::ReceiveInit()
{
  if (rx_warm_) return;

  // LoRa, Standby
  WriteRegister(SX1276REG_OpMode, SX1276_OPMODE_STANDBY);
  delay(10);

  // Setup the LNA:
  ConfigureGain();

  WriteRegister(SX1276REG_MaxPayloadLength, max_rx_payload_bytes_);
  WriteRegister(SX1276REG_PayloadLength, max_rx_payload_bytes_);

  // Reset RX FIFO
  WriteRegister(SX1276REG_FifoRxBaseAddr, RX_BASE_ADDR);
  WriteRegister(SX1276REG_FifoAddrPtr, RX_BASE_ADDR);

  // RX mode
  WriteRegister(SX1276REG_IrqFlagsMask, 0x0f);

  rx_warm_ = true;
  tx_warm_ = false;
}

ICACHE_FLASH_ATTR
bool SX1276Radio::ReceiveMessage(byte buffer[], byte size, byte& received, bool& crc_error)
{
  if (size < 1) { return false; }
  if (size < max_rx_payload_bytes_) {
    //DEBUG("BUFFER MAYBE TOO SHORT! DATA POTENTIALLY MAY BE LOST!\n\r");
  }

  // In most use cases we probably want to to this once then stay 'warm'
  ReceiveInit();

  ClearInterrupts();

  bool isSingleMode = true;
  WriteRegister(SX1276REG_OpMode, SX1276_OPMODE_RXSINGLE); // RX Single mode

  // Now we block, until symbol timeout or we get a message
  // Which in practice means polling the IRQ flags
  // and for the purpose of the ESP8266, we need to yield occasionally
  // optional callback to allow serial port processing without appearing to hang

  bool done = false;
  bool symbol_timeout = false;
  crc_error = false;
  byte flags = 0;
  byte stepping = 0;
  bool shouldAbort = false;
  do {
    flags = 0;
    ReadRegister(SX1276REG_IrqFlags, flags);
    if (flags & (1 << 4)) {
      // valid header
    }
    if (flags & (1 << 6)) {
      done = true;
      break;
    }
    symbol_timeout = flags & (1 << 7);
    if (!symbol_timeout) {
      if (rxPollFunction) { rxPollFunction(shouldAbort); }
      if (++stepping >= 2) {
        stepping = 0;
        yield();
      }
    }
  } while (!symbol_timeout && !shouldAbort);

  byte v = 0;
  byte stat = 0;

  rssi_dbm_ = -255;
  rx_snr_db_ = -255;
  ReadRegister(SX1276REG_Rssi, v); rssi_dbm_ = -157 + v;

  if (!done) { return false; }

  int rssi_packet = 255;
  int snr_packet = -255;
  int coding_rate = 0;
  ReadRegister(SX1276REG_PacketRssi, v); rssi_packet = -157 + v;
  ReadRegister(SX1276REG_PacketSnr, v); snr_packet = (v & 0x80 ? int(v) - 256 : v) / 4; // 2's comp div 4? really?
  ReadRegister(SX1276REG_ModemStat, stat);
  coding_rate = stat >> 5;
  switch (coding_rate) {
    case 0x1: coding_rate = 5; break;
    case 0x2: coding_rate = 6; break;
    case 0x3: coding_rate = 7; break;
    case 0x4: coding_rate = 8; break;
    default: coding_rate = 0;  break;
  }
  rx_snr_db_ = snr_packet;

  byte payloadSizeBytes = 0xff;
  uint16_t headerCount = 0;
  uint16_t packetCount = 0;
  uint8_t byptr = 0;
  ReadRegister(SX1276REG_FifoRxNbBytes, payloadSizeBytes);
  ReadRegister(SX1276REG_RxHeaderCntValueMsb, v); headerCount = (uint16_t)v << 8;
  ReadRegister(SX1276REG_RxHeaderCntValueLsb, v); headerCount |= v;
  if (!isSingleMode) {
    ReadRegister(SX1276REG_RxPacketCntValueMsb, v); packetCount = (uint16_t)v << 8;
    ReadRegister(SX1276REG_RxPacketCntValueLsb, v); packetCount |= v;
  }
  // Note: SX1276REG_FifoRxByteAddrPtr == last addr written by modem
  ReadRegister(SX1276REG_FifoRxByteAddrPtr, byptr);

  payloadSizeBytes--; // DONT KNOW WHY, I THINK FifoRxNbBytes points 1 down

  bool crcErrorDetected = !!(flags & (1 << 5));
  bool overflow = (int)payloadSizeBytes > size;

  if (!crcErrorDetected && !overflow) {
    for (unsigned n=0; n < payloadSizeBytes; n++) {
      ReadRegister(SX1276REG_Fifo, v);
      buffer[n] = v;
    }
    received = payloadSizeBytes;
  }

  DEBUG("[DBUG] ");
  DEBUG("RX rssi_pkt=%d ", rssi_packet);
  DEBUG("snr_pkt=%d ", snr_packet);
  DEBUG("stat=%02x ", (unsigned)stat);
  DEBUG("sz=%d ", (unsigned)payloadSizeBytes);
  DEBUG("ptr=%d ", (unsigned)byptr);
  DEBUG("errors=%d,%d ", crcErrorDetected, overflow);
  DEBUG("hdrcnt=%d pktcnt=%d\n\r", (unsigned)headerCount, (unsigned)packetCount);

  // check CRC ...
  if (crcErrorDetected) {
    DEBUG("CRC Error. Packet rssi=%ddBm snr=%d cr=4/%d\n\r", rssi_packet, snr_packet, coding_rate);
    return false;
  }

  if (overflow) {
    DEBUG("Buffer size %d not large enough for packet size %d!\n\r", size, (int)payloadSizeBytes);
    return false;
  }

  return true;
}
