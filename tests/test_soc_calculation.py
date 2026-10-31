# -*- coding: utf-8 -*-
"""
Tests for the reset to 0% in Battery.soc_calculation().

The reset must only depend on the lowest cell voltage staying at or below
MIN_CELL_VOLTAGE for longer than 60 seconds. Many BMS report small currents
as 0 A and the discharge current limit reaches 0 A at MIN_CELL_VOLTAGE, so the
battery that gets empty last arrives there with a reported current of 0 A.
"""

from __future__ import annotations

import pytest
from unittest.mock import patch

import battery as battery_module
import utils
from battery import Battery

CAPACITY = 125.0
EMPTY = utils.MIN_CELL_VOLTAGE - 0.01
RECOVERED = utils.MIN_CELL_VOLTAGE + 0.05


class FakeBattery(Battery):
    def __init__(self):
        super().__init__(port="/dev/null", baud=9600, address="\x00")
        self.capacity = CAPACITY
        self.min_cell = RECOVERED

    def test_connection(self) -> bool:
        return True

    def get_settings(self) -> bool:
        return True

    def refresh_data(self) -> bool:
        return True

    def get_min_cell_voltage(self):
        return self.min_cell


class Clock:
    def __init__(self):
        self.now = 1_000_000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    c = Clock()
    with patch.object(battery_module, "time", c):
        yield c


def start_at(clock, soc: float) -> FakeBattery:
    """Return a battery whose calculated SoC is initialized to soc."""
    bat = FakeBattery()
    bat.soc = soc
    bat.current_calc = 0.0
    bat.soc_calc = bat.soc_calculation()
    assert bat.soc_calc == pytest.approx(soc)
    return bat


def run(bat: FakeBattery, clock: Clock, seconds: int, min_cell: float, current: float, step: int = 5) -> None:
    """Advance the clock in steps and feed the driver like the main loop does."""
    bat.min_cell = min_cell
    bat.current_calc = current
    for _ in range(0, seconds, step):
        clock.now += step
        bat.soc_calc = bat.soc_calculation()


def test_reset_with_reported_current_of_zero(clock):
    bat = start_at(clock, 8.5)
    run(bat, clock, 60, EMPTY, 0.0)
    assert bat.soc_calc == pytest.approx(8.5)
    run(bat, clock, 10, EMPTY, 0.0)
    assert bat.soc_calc == 0


def test_reset_while_discharging(clock):
    bat = start_at(clock, 8.5)
    run(bat, clock, 70, EMPTY, -0.7)
    assert bat.soc_calc == 0


def test_reset_when_current_alternates_between_zero_and_discharge(clock):
    bat = start_at(clock, 8.5)
    for _ in range(7):
        run(bat, clock, 5, EMPTY, -0.6)
        run(bat, clock, 5, EMPTY, 0.0)
    assert bat.soc_calc == 0


def test_no_reset_above_min_cell_voltage(clock):
    bat = start_at(clock, 8.5)
    run(bat, clock, 600, RECOVERED, 0.0)
    assert bat.soc_calc == pytest.approx(8.5)


def test_timer_restarts_when_cell_voltage_recovers(clock):
    # https://github.com/mr-manuel/venus-os_dbus-serialbattery/issues/491
    bat = start_at(clock, 8.5)
    run(bat, clock, 30, EMPTY, 0.0)
    run(bat, clock, 120, RECOVERED, 0.0)
    run(bat, clock, 30, EMPTY, 0.0)
    assert bat.soc_calc == pytest.approx(8.5)
    run(bat, clock, 40, EMPTY, 0.0)
    assert bat.soc_calc == 0


def test_single_reset_log_while_staying_empty(clock, caplog):
    bat = start_at(clock, 8.5)
    with caplog.at_level("INFO"):
        run(bat, clock, 600, EMPTY, 0.0)
    assert bat.soc_calc == 0
    assert caplog.text.count("SOC set to 0%") == 1


def test_counting_continues_after_reset(clock):
    bat = start_at(clock, 8.5)
    run(bat, clock, 70, EMPTY, 0.0)
    assert bat.soc_calc == 0
    # charge 12.5 Ah = 10 % at 12.5 A for one hour
    run(bat, clock, 3600, RECOVERED, 12.5)
    assert bat.soc_calc == pytest.approx(10.0, abs=0.05)
