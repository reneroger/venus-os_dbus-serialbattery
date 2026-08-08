# -*- coding: utf-8 -*-
"""
Tests for the Pace_Modbus battery management system driver.

The register values come from frames captured on an Eco-Worthy ECO-LFP-4831400
(PACE P16S200A-61056-1.00, 16S LiFePO4) over RS485 at 9600 8N1, slave address
0x01, function code 0x03:

    Req  01 03 0000 0008  -> 01 03 10 0000 1498 001E 0064 24C6 7AA8 7AA8 0000
    Req  01 03 0010 0008  -> 01 03 10 0CDD 0CDD 0CDD 0CDD 0CDD 0CDD 0CDD 0CDD
    Req  01 03 0020 0008  -> 01 03 10 00F0 00F5 00F0 00ED 0114 FFFF FFFF FFFF
    Req  01 03 0040 0008  -> 01 03 10 0E10 0E42 0D34 000A AF00 A8C0 B860 000A
"""

from __future__ import annotations

import pytest
from unittest.mock import patch

from bms.pace_modbus import Pace_Modbus


# Registers 0-7 as captured. Registers 8-12 were not covered by a capture, so
# they carry values that exercise decoding: no alarms, both FETs enabled,
# cells 1 and 3 balancing.
STATUS_REGISTERS = [
    0x0000,  # 0  current, 10 mA -> 0.00 A
    0x1498,  # 1  pack voltage, 10 mV -> 52.72 V
    0x001E,  # 2  soc -> 30 %
    0x0064,  # 3  soh -> 100 %
    0x24C6,  # 4  remaining capacity, 10 mAh -> 94.14 Ah
    0x7AA8,  # 5  full capacity, 10 mAh -> 314.00 Ah
    0x7AA8,  # 6  design capacity, 10 mAh -> 314.00 Ah
    0x0000,  # 7  charge cycles
    0x0000,  # 8  reserved
    0x0000,  # 9  warning bitmask
    0x0000,  # 10 protection bitmask
    0x0C00,  # 11 status: bit 10 charge allowed, bit 11 discharge allowed
    0x0005,  # 12 balancer bits: cell 1 and cell 3
]

# Registers 15-30. The capture covered cells 2-9 at 0x0CDD = 3293 mV; 16 cells
# at that level sum to 52.688 V, which matches the 52.72 V pack reading.
CELL_REGISTERS = [0x0CDD] * 16

# Registers 31-36. The capture started at register 32, so 31 is assumed to sit
# at the same 24.0 °C as its neighbours.
TEMPERATURE_REGISTERS = [
    0x00F0,  # 31 battery temperature 1 -> 24.0 °C
    0x00F0,  # 32 battery temperature 2 -> 24.0 °C
    0x00F5,  # 33 battery temperature 3 -> 24.5 °C
    0x00F0,  # 34 battery temperature 4 -> 24.0 °C
    0x00ED,  # 35 MOSFET temperature -> 23.7 °C
    0x0114,  # 36 ambient temperature -> 27.6 °C
]


def fake_read_registers(status=None, cells=None, temperatures=None):
    """
    Build a replacement for Pace_Modbus.read_registers backed by fixed blocks.

    Passing None for a block makes that read fail, the way a timeout or a Modbus
    exception reply does.
    """
    status = STATUS_REGISTERS if status is None else status
    cells = CELL_REGISTERS if cells is None else cells
    temperatures = TEMPERATURE_REGISTERS if temperatures is None else temperatures

    def read(self, start, count):
        if start == 0:
            return None if status is False else status[:count]
        if start == 15:
            return None if cells is False else cells[:count]
        if start == 31:
            return None if temperatures is False else temperatures[:count]
        # The BMS answers anything from 0x1000 upwards with exception 02.
        return None

    return read


@pytest.fixture
def battery():
    return Pace_Modbus("/dev/ttyTEST", 9600, b"\x01")


def test_slave_address_from_bytes():
    assert Pace_Modbus("/dev/ttyTEST", 9600, b"\x01").slave_address == 1
    assert Pace_Modbus("/dev/ttyTEST", 9600, b"\x0f").slave_address == 15
    # No address configured falls back to the documented default of 0x01.
    assert Pace_Modbus("/dev/ttyTEST", 9600, None).slave_address == 1


@pytest.mark.parametrize(
    "raw, expected",
    [
        (0x0000, 0),
        (0x7FFF, 32767),
        (0x8000, -32768),
        (0xFFFF, -1),
        (0xFC18, -1000),  # -10.00 A
    ],
)
def test_to_signed_int(raw, expected):
    assert Pace_Modbus.to_signed_int(raw) == expected


def test_test_connection_accepts_measured_values(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers()):
        assert battery.test_connection() is True


def test_test_connection_rejects_silent_port(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(status=False)):
        assert battery.test_connection() is False


def test_test_connection_rejects_implausible_voltage(battery):
    # 0x0400 = 1024 -> 10.24 V, far below any 16S pack.
    status = list(STATUS_REGISTERS)
    status[1] = 0x0400
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(status=status)):
        assert battery.test_connection() is False


def test_test_connection_rejects_zero_capacity(battery):
    status = list(STATUS_REGISTERS)
    status[5] = 0x0000
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(status=status)):
        assert battery.test_connection() is False


def test_status_values_match_capture(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers()):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    assert battery.current == 0.0
    assert battery.voltage == 52.72
    assert battery.soc == 30
    assert battery.soh == 100
    assert battery.capacity_remain == 94.14
    assert battery.capacity == 314.00
    assert battery.history.charge_cycles == 0
    assert battery.charge_fet is True
    assert battery.discharge_fet is True


def test_negative_current_is_decoded_as_discharge(battery):
    status = list(STATUS_REGISTERS)
    status[0] = 0xFC18  # -1000 -> -10.00 A
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(status=status)):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True
    assert battery.current == -10.00


def test_fets_reflect_status_bits(battery):
    status = list(STATUS_REGISTERS)
    status[11] = 0x0400  # charge allowed, discharge blocked
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(status=status)):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True
    assert battery.charge_fet is True
    assert battery.discharge_fet is False


def test_cell_voltages_match_capture(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers()):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    assert battery.cell_count == 16
    assert [cell.voltage for cell in battery.cells] == [3.293] * 16
    # The cell sum has to land within 1 % of the reported pack voltage.
    assert abs(sum(cell.voltage for cell in battery.cells) - battery.voltage) / battery.voltage < 0.01


def test_balancer_bits_map_to_cells(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers()):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    balancing = [index + 1 for index, cell in enumerate(battery.cells) if cell.balance]
    assert balancing == [1, 3]


@pytest.mark.parametrize("unpopulated", [0x0000, 0xFFFF])
def test_cell_count_ignores_unpopulated_channels(battery, unpopulated):
    # Which value an unwired channel reports is not confirmed on hardware, so
    # both candidates have to be discarded.
    cells = [0x0CDD] * 8 + [unpopulated] * 8
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(cells=cells)):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    assert battery.cell_count == 8
    assert [cell.voltage for cell in battery.cells] == [3.293] * 8


def test_get_settings_fails_without_any_cell(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(cells=[0xFFFF] * 16)):
        assert battery.get_settings() is False


def test_temperatures_match_capture(battery):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers()):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    assert battery.temperature_1 == 24.0
    assert battery.temperature_2 == 24.0
    assert battery.temperature_3 == 24.5
    assert battery.temperature_4 == 24.0
    assert battery.temperature_mos == 23.7


def test_unwired_temperature_sensor_is_skipped(battery):
    temperatures = list(TEMPERATURE_REGISTERS)
    temperatures[3] = 0xFFFF  # battery sensor 4 not connected
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(temperatures=temperatures)):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    assert battery.temperature_4 is None
    assert battery.temperature_3 == 24.5


def test_negative_temperature_is_decoded(battery):
    temperatures = list(TEMPERATURE_REGISTERS)
    temperatures[0] = 0xFFCE  # -50 -> -5.0 °C
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(temperatures=temperatures)):
        assert battery.get_settings() is True
        assert battery.refresh_data() is True

    assert battery.temperature_1 == -5.0


@pytest.mark.parametrize("failing_block", ["status", "cells", "temperatures"])
def test_refresh_data_fails_cleanly_on_timeout(battery, failing_block):
    with patch.object(Pace_Modbus, "read_registers", fake_read_registers()):
        assert battery.get_settings() is True

    with patch.object(Pace_Modbus, "read_registers", fake_read_registers(**{failing_block: False})):
        assert battery.refresh_data() is False
