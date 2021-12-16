# The sentrifarm SX1276 Arduino Library

This Arduino library was developed as part of [sentrifarm](https://github.com/pastcompute/sentrifarm) and subsequently abstracted out to a standalone git repository.

Tested & used with the [inAir9](https://modtronix.com/product/inair9/) Lora module at 915 MHz ISM band.

References:
- https://www.semtech.com/products/wireless-rf/lora-core/sx1276
- [Direct link to Datasheet](https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R0000001Rbr/6EfVZUorrpoKFfvaF_Fkpgp5kzjiNyiAbqcpqh9qSjE)
- [Direct link to Errata Note](https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/2R000000HSPv/sqi9xX0gs6hgzl2LoPwCK0TS9GDPlMwsXmcNzJCMHjw?__hstc=212684107.a3633c62ce3f77274a091d959c8e8c35.1639639473552.1639639473552.1639639473552.1&__hssc=212684107.2.1639639473552&__hsfp=2317546800)

# Use Example

```
#include <Arduino.h>
#include <SPI.h>
#include "sx1276.h"

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); // double check
SX1276Radio radio(PIN_SX1276_CS, spiSettings);

#define PIN_SX1276_RST ... // setup pins for your board

void setup() {
  pinMode(PIN_SX1276_RST,  OUTPUT);
  pinMode(PIN_SX1276_CS,   OUTPUT);

  digitalWrite(PIN_SX1276_CS, HIGH);
  digitalWrite(PIN_SX1276_RST, HIGH);
  digitalWrite(PIN_SX1276_MISO, HIGH);
  digitalWrite(PIN_SX1276_MOSI, HIGH);
  digitalWrite(PIN_SX1276_SCK,  HIGH);

  // Reset the sx1276 module
  digitalWrite(PIN_SX1276_RST, LOW);
  delay(20); // spec states to pull low 100us then wait at least 5 ms
  digitalWrite(PIN_SX1276_RST, HIGH);
  delay(50);

  SPI.begin();
  Serial.print(F("SX1276: version=")); Serial.println(radio.ReadVersion());
  if (!radio.Begin()) {
    Serial.println(F("SX1276 init error"));
  } else {
    radio.SetCarrier(919000000);
    uint32_t carrier_hz = 0;
    radio.ReadCarrier(carrier_hz);
    Serial.print(F("Carrier: ")); Serial.println(carrier_hz);
    started_ok = true;
  }
  SPI.end();
}

void loop() {
  SPI.begin();
  byte buffer[128];
  bool crc_error = false;
  byte received = 0;
  elapsedMillis trx;
  memset(buffer, 0, sizeof(buffer));
  if (radio.ReceiveMessage(buffer, sizeof(buffer), received, crc_error))
  {
    radio.Standby();
    Serial.print(F("Rx "));
    Serial.print(received);  // will print junk if not ASCII...!
    Serial.print(F(" "));
    Serial.print(crc_error);
    Serial.println();
  } else if (crc_error) {
    radio.Standby();
    Serial.println("CRC error\n");
  } else {
    // no data, so loop around and keep waiting, no need to revert to Standby mode
  }
  SPI.end();
}
```

