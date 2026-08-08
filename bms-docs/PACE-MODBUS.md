# PACE BMS — Modbus RTU over RS485 (`Pace_Modbus`)

Driver for PACE BMS running the `PACE_MODBUS` protocol on RS485, as shipped in
Eco-Worthy and other rebranded LiFePO4 packs.

## How this differs from the `Pace` driver

The existing `Pace` driver speaks **paceic over RS232**. This driver speaks
**Modbus RTU over RS485**. They are not interchangeable: a pack whose display
shows `PROTOCOL → RS485 → PACE_MODBUS` does not answer paceic frames on RS485 at
any baud rate, so the existing driver never detects it.

Check the active protocol on the BMS display before choosing a driver.

## Connection

| | |
| --- | --- |
| Interface | RS485, RJ45 socket |
| Pinout | Pin 1 = B−, Pin 2 = A+ |
| Serial settings | 9600 8N1 |
| Function code | `0x03`, read holding registers |
| Default address | `0x01` |

A 2 m patch cable without a separate ground connection works.

The Modbus address comes from the DIP switches, encoded in binary: switch 1 = ON
gives address `0x01`, and the encoding runs up to `0x0F`. **After changing the
DIP switches the BMS has to be reset** (breaker or reset button), otherwise the
new address is not picked up.

For more than one pack on the same RS485 bus, give each a different DIP address
and list them in `config.ini`:

```ini
BMS_TYPE = Pace_Modbus
BATTERY_ADDRESSES = 0x01, 0x02
```

## Register map

All registers are 16 bit, read with function code `0x03`.

| Register | Content | Type | Scaling |
| --- | --- | --- | --- |
| 0 | Current | int16 | 10 mA, positive = charging |
| 1 | Pack voltage | uint16 | 10 mV |
| 2 | SOC | uint16 | % |
| 3 | SOH | uint16 | % |
| 4 | Remaining capacity | uint16 | 10 mAh |
| 5 | Full capacity | uint16 | 10 mAh |
| 6 | Design capacity | uint16 | 10 mAh |
| 7 | Charge cycles | uint16 | — |
| 9 | Warning bitmask | uint16 | bit assignment unknown |
| 10 | Protection bitmask | uint16 | bit assignment unknown |
| 11 | Status bitmask | uint16 | bit 8 charging, bit 9 discharging, bit 10 charge allowed, bit 11 discharge allowed |
| 12 | Balancer bits | uint16 | bit 0 = cell 1 |
| 15–30 | Cell voltage 1–16 | uint16 | mV |
| 31–34 | Battery temperature 1–4 | int16 | 0.1 °C |
| 35 | MOSFET temperature | int16 | 0.1 °C |
| 36 | Ambient temperature | int16 | 0.1 °C |
| 60–63 | Pack OV: alarm / protection / release / delay | uint16 | mV, mV, mV, 0.1 s |
| 64–67 | Cell OV: alarm / protection / release / delay | uint16 | mV, mV, mV, 0.1 s |
| 68–71 | Pack UV: alarm / protection / release / delay | uint16 | mV, mV, mV, 0.1 s |
| 72–75 | Cell UV: alarm / protection / release / delay | uint16 | mV, mV, mV, 0.1 s |

Register map from https://github.com/syssi/esphome-pace-bms.

Anything from `0x1000` upwards answers with Modbus exception `02`, illegal data
address.

The driver reads three blocks per cycle — 0–12, 15–30 and 31–36 — and never
writes.

## Verified readings

Captured on an Eco-Worthy ECO-LFP-4831400 (PowerMega 314), 51.2 V / 314 Ah, 16S,
BMS `P16S200A-61056-1.00`:

```
Req  01 03 0000 0008  ->  01 03 10 0000 1498 001E 0064 24C6 7AA8 7AA8 0000
Req  01 03 0010 0008  ->  01 03 10 0CDD 0CDD 0CDD 0CDD 0CDD 0CDD 0CDD 0CDD
Req  01 03 0020 0008  ->  01 03 10 00F0 00F5 00F0 00ED 0114 FFFF FFFF FFFF
Req  01 03 0040 0008  ->  01 03 10 0E10 0E42 0D34 000A AF00 A8C0 B860 000A
```

Decoded: 0 A, 52.72 V, SOC 30 %, SOH 100 %, 94.14 Ah remaining of 314.00 Ah,
0 cycles, cells at 3293 mV, temperatures 24.0 / 24.5 / 24.0 °C, MOSFET 23.7 °C,
ambient 27.6 °C, cell OV alarm 3600 mV, cell OV protection 3650 mV, release
3380 mV, delay 1.0 s, pack UV alarm 44.80 V, protection 43.20 V, release
47.20 V, delay 1.0 s.

## Limitations

* **No charge or discharge limits from the BMS.** No CCL/DCL register is known,
  so the driver keeps the configured `MAX_BATTERY_CHARGE_CURRENT` and
  `MAX_BATTERY_DISCHARGE_CURRENT` and lets the driver work out CVL/CCL/DCL.
* **No serial number or model register.** `unique_identifier()` therefore falls
  back to hardware version and capacity. With several identical packs, either
  vary the capacity slightly or set `USE_PORT_AS_UNIQUE_ID = True` in
  `config.ini`.
* **No alarm mapping.** Registers 9 and 10 hold warning and protection
  bitmasks, but their bit assignment is not documented in the source this driver
  was built from. Non-zero values are logged, not mapped onto protection flags.
* **Cell count is inferred**, not read: channels reporting a voltage between
  1000 mV and 5000 mV count as populated. Unwired channels report `0x0000` or
  `0xFFFF` and are discarded.
