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

#ifndef SX1276_H__
#define SX1276_H__

#include <Arduino.h>
#include <SPI.h>

// Ideally I'd try and share with the Linux version
// But thats a future Yak shaving task if warranted
// Amongst other things, we skip a lot of the error handling in this simplified version
// Like, we dont try and detect if its plugged in wrong (and dont have to care about Bus Pirate errors, etc)

/// This class provides a very simple model of the inAir9 SX1276 LoRa module operating in LoRa mode
/// The model is sufficent to use with MQTT-SN in a point to point environment
/// that doesnt require management or adherence to such things as the LoRa LRSC specification
class SX1276Radio
{
public:
  typedef void (*rxPollFunction_t)(bool& abort);

  SX1276Radio(int cs_pin, const SPISettings& spi_settings, bool inAir9b = false);

  /// Revert to Standby mode and return all settings to a useful default
  /// The caller might choose to reset the module via GPIO prior to calling this
  /// The defaults are chosen somewhat arbitrarily at this stage, and will be tweaked
  /// to values useful to the authors use case!
  bool Begin();

  /// Read chip version byte
  byte ReadVersion();

  /// Get the currently tuned carrier frequency from the chip
  void ReadCarrier(uint32_t& carrier_hz);

  /// Get the currently set bandwidth from the chip
  void ReadBandwidth(uint32_t& bandwidth_hz);

  /// Get the currently set spreading factor from the chip
  void ReadSpreadingFactor(byte& sf);

  /// Get the maximum allowed payload in bytes
  uint8_t GetMaxPayload() const { return max_tx_payload_bytes_; }

  /// Set the carrier frequency
  void SetCarrier(uint32_t carrier_hz);

  /// Set the spreading factor
  /// 6 --> 64 chips/symbol through 12 --> 4096 chips/symbol
  /// ie chips/symbol = 2^sf
  /// at reset is 7 (128)
  /// invalid values are forced to 7
  void SetSpreadingFactor(byte sf);
  byte GetSpreadingFactor() const { return spreading_factor_; }

  /// Set bandwidth using the SX1276_LORA_BW_xxx constants
  void SetBandwidth(byte bwIndex);
  uint32_t GetBandwidthHz() const { return bandwidth_hz_; }
  byte GetBandwidthIndex() const { return bandwidth_idx_; }

  void SetLNAGain(byte gain);
  byte GetLNAGain() const { return lna_gain_; }

  void SetPowerLimit(byte ocpTrim);

  /// Get RSSi of last receive
  int GetLastRssi() const { return rssi_dbm_; }

  /// Get SNR of last receive
  int GetLastSnr() const { return rx_snr_db_; }

  /// Return the SX1276 to LoRa standby mode
  void Standby();

  /// Calcluates the estimated time on air in rounded up milliseconds for a given simple payload
  /// based on the formulae in the SX1276 datasheet
  int PredictTimeOnAir(byte payload_len) const;

  /// Send a message, and block until it is sent
  /// @param payload Message body
  /// @param len Length in bytes.
  /// @note Warning! if len > GetMaxPayload() then message is truncated!
  /// @note Assumes device properly configured, i.e. doesnt sanity check other registers
  /// @return false if retry timeout exceeded. Presently internally set to predicted TOA + a fudge factor
  bool TransmitMessage(const void *payload, byte len, bool withStandby=true);

  /// Clear any open IRQ
  inline void ClearInterrupts() { WriteRegister(SX1276REG_IrqFlags, 0xff); }

  /// Wait for a message, block until one is received or a symbol timeout occurs
  /// Useful in simple circumstances; may not perform well in high traffic scenarios
  /// @param buffer Buffer large enough to hold largest expected message
  /// @param size Size of buffer
  /// @note Assumes device properly configured, i.e. doesnt sanity check other registers
  /// @param crc_error Set to true if returned false because of crc not timeout
  /// @return false if timeout or crc error
  bool ReceiveMessage(byte buffer[], byte size, byte& received, bool& crc_error);

  void SetReceivePollCallback(rxPollFunction_t cbk) { rxPollFunction = cbk; }

  bool fault() const { return dead_; }

private:
  void ReadRegister(byte reg, byte& result);

  void WriteRegister(byte reg, byte val, byte& result, bool verify);
  inline void WriteRegister(byte reg, byte val, bool verify = false) { byte unused; WriteRegister(reg, val, unused, verify); }
  void WriteBulk(byte reg, const byte *val, byte count);
  byte DoRegister(byte reg, byte val) const;

  void ConfigureBandwidth();
  void ConfigureSpreadingFactor();
  void ConfigureGain();
  void ReceiveInit();
  bool ReceiveComplete(byte buffer[], byte size, bool flags, byte& received, bool& crc_error);

  // module settings
  int cs_pin_;
  SPISettings spi_settings_;
  bool inAir9b_;

  // radio settings
  uint16_t symbol_timeout_;      // In datasheet units
  uint16_t preamble_;            // In datasheet units
  uint8_t max_tx_payload_bytes_;
  uint8_t max_rx_payload_bytes_;
  uint32_t bandwidth_hz_;        // calculated
  uint8_t bandwidth_idx_;        // cached
  byte spreading_factor_;        // datasheet units: 6,7,8,9,10,11,12
  byte coding_rate_;             // datasheet units: 5,6,7,8
  uint32_t carrier_hz_;
  byte lna_gain_;                // datasheet units: 1 (G1 - highest) --> 6 (G6)

  // radio status
  int rssi_dbm_;
  int rx_snr_db_;
  bool rx_warm_;
  bool tx_warm_;
  bool dead_;

  uint8_t cached_tx_payload_length_;
  uint16_t cached_tx_toa_;

  rxPollFunction_t rxPollFunction;
};

#endif //SX1276_H__
