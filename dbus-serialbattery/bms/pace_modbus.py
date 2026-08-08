# -*- coding: utf-8 -*-

# NOTES
# Added by https://github.com/reneroger
#
# Driver for PACE BMS speaking Modbus RTU over RS485, as selected on the BMS
# display under PROTOCOL -> RS485 -> PACE_MODBUS.
#
# This is a different protocol from the existing `Pace` driver, which speaks
# paceic over RS232. A pack set to PACE_MODBUS does not answer paceic frames on
# RS485 at all, so the two drivers cannot replace each other.
#
# Register map from https://github.com/syssi/esphome-pace-bms, verified against
# an Eco-Worthy ECO-LFP-4831400 (PACE P16S200A-61056-1.00, 16S LiFePO4).

import struct
import sys

import ext.minimalmodbus as minimalmodbus
import serial
from battery import Battery, Cell
from utils import get_connection_error_message, logger, wrap_modbus_capture

# Function code 3, holding registers.
FUNCTION_CODE = 3

# Blocks read per refresh. Kept apart because registers 13/14 and 37+ are
# reserved and reading across them gains nothing.
REGISTER_STATUS = 0
REGISTER_STATUS_COUNT = 13
REGISTER_CELLS = 15
REGISTER_CELLS_COUNT = 16
REGISTER_TEMPERATURE = 31
REGISTER_TEMPERATURE_COUNT = 6

# A populated cell channel of a LiFePO4 pack always sits inside this window.
# Unpopulated channels report 0x0000 or 0xFFFF, both of which fall outside.
CELL_VOLTAGE_MIN_MV = 1000
CELL_VOLTAGE_MAX_MV = 5000

# Plausibility window for the pack voltage during connection test, in volts.
# Covers 8S to 16S packs from empty to full.
PACK_VOLTAGE_MIN = 20
PACK_VOLTAGE_MAX = 60

# A temperature channel that is not wired reports 0xFFFF.
TEMPERATURE_INVALID = 0xFFFF


class Pace_Modbus(Battery):
    def __init__(self, port, baud, address):
        super(Pace_Modbus, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE

        # The address goes into every Modbus frame, so packs with different DIP
        # switch settings can share one RS485 bus.
        self.address = address
        self.slave_address = int.from_bytes(address, "big") if address else 1

        self.mbdev = None

        # The BMS reports the cycle count, so don't let the driver derive it.
        self.history.exclude_values_to_calculate = ["charge_cycles"]

    BATTERYTYPE = "PACE Modbus"

    @staticmethod
    def to_signed_int(value: int) -> int:
        """
        Reinterpret a 16 bit register as a signed value.

        :param value: the raw register value
        :return: the signed value
        """
        return struct.unpack("<h", struct.pack("<H", value))[0]

    def get_modbus(self) -> minimalmodbus.Instrument:
        """
        Return the Modbus instrument for this battery, creating it on first use.

        :return: the instrument
        """
        if self.mbdev is not None:
            return self.mbdev

        mbdev = minimalmodbus.Instrument(
            self.port,
            slaveaddress=self.slave_address,
            mode="rtu",
            close_port_after_each_call=True,
            debug=False,
        )
        mbdev.serial.parity = serial.PARITY_NONE
        mbdev.serial.stopbits = serial.STOPBITS_ONE
        mbdev.serial.bytesize = serial.EIGHTBITS
        mbdev.serial.baudrate = self.baud_rate
        mbdev.serial.timeout = 0.5
        wrap_modbus_capture(mbdev, self.port)

        self.mbdev = mbdev
        return mbdev

    def read_registers(self, start: int, count: int):
        """
        Read a block of holding registers.

        Returns None instead of raising, so a timeout or a Modbus exception
        reply ends up as a logged error and a failed refresh rather than
        taking the driver down.

        :param start: first register address
        :param count: number of registers to read
        :return: list of register values, or None on failure
        """
        try:
            return self.get_modbus().read_registers(registeraddress=start, number_of_registers=count, functioncode=FUNCTION_CODE)
        except minimalmodbus.ModbusException as e:
            # The BMS answers out-of-range reads with exception 02, illegal
            # data address. Seen for every register at or above 0x1000.
            logger.debug(f"Modbus exception reading {count} registers from {start} on {self.port} address {self.slave_address}: {e}")
            return None
        except Exception as e:
            logger.debug(f"Failed to read {count} registers from {start} on {self.port} address {self.slave_address}: {e}")
            return None

    def test_connection(self) -> bool:
        """
        Check whether a PACE BMS in Modbus mode answers on this port and address.

        :return: True if success, False for failure
        """
        found = False
        try:
            status = self.read_registers(REGISTER_STATUS, 8)

            if status is None:
                logger.debug(f"No reply from {self.port} on address {self.slave_address}")
            else:
                voltage = status[1] / 100
                capacity = status[5] / 100
                soc = status[2]

                # There is no model or serial register to identify the BMS by,
                # so identification rests on the readings being plausible.
                if PACK_VOLTAGE_MIN <= voltage <= PACK_VOLTAGE_MAX and capacity > 0 and soc <= 100:
                    logger.info(f"Identified PACE Modbus BMS with {capacity} Ah at {voltage} V on address {self.slave_address}")
                    found = True
                else:
                    logger.debug(f"Implausible reply from {self.port}: {voltage} V, {capacity} Ah, {soc} % SOC")

            if found:
                found = self.get_settings() and self.refresh_data()

        except Exception:
            (
                exception_type,
                exception_object,
                exception_traceback,
            ) = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")
            found = False

        if not found:
            get_connection_error_message(self.online)

        return found

    def get_settings(self) -> bool:
        """
        Read the values that only have to be read once.

        :return: True if success, False for failure
        """
        status = self.read_registers(REGISTER_STATUS, REGISTER_STATUS_COUNT)
        if status is None:
            return False

        self.capacity = status[5] / 100

        cells = self.read_registers(REGISTER_CELLS, REGISTER_CELLS_COUNT)
        if cells is None:
            return False

        # No register reports how many cells are wired up, so count the
        # channels that report a voltage a cell could actually have.
        self.cell_count = sum(1 for millivolt in cells if CELL_VOLTAGE_MIN_MV <= millivolt <= CELL_VOLTAGE_MAX_MV)

        if self.cell_count == 0:
            logger.error(f"No cell voltages found on {self.port} address {self.slave_address}")
            return False

        # The BMS reports no charge or discharge limits of its own, so the
        # driver keeps the configured defaults and works out CVL/CCL/DCL itself.
        logger.info(f"PACE Modbus: {self.cell_count} cells, {self.capacity} Ah")

        # init the cell array once
        if len(self.cells) == 0:
            for _ in range(self.cell_count):
                self.cells.append(Cell(False))

        return True

    def refresh_data(self) -> bool:
        """
        Read everything that changes during operation.

        :return: True if success, False for failure
        """
        result = self.read_status_data()
        result = result and self.read_cell_data()
        result = result and self.read_temperature_data()
        return result

    def read_status_data(self) -> bool:
        """
        Read current, voltage, SOC, capacities, cycles and the status flags.

        :return: True if success, False for failure
        """
        status = self.read_registers(REGISTER_STATUS, REGISTER_STATUS_COUNT)
        if status is None:
            return False

        # Positive current means charging, which matches the driver's sign.
        self.current = self.to_signed_int(status[0]) / 100
        self.voltage = status[1] / 100
        self.soc = status[2]
        self.soh = status[3]
        self.capacity_remain = status[4] / 100
        self.capacity = status[5] / 100
        self.history.charge_cycles = status[7]

        # Register 11, bit 10 charge allowed, bit 11 discharge allowed.
        flags = status[11]
        self.charge_fet = bool(flags & (1 << 10))
        self.discharge_fet = bool(flags & (1 << 11))

        # Registers 9 and 10 hold the warning and protection bitmasks. Their bit
        # assignment is not covered by the register map this driver was built
        # from, so they are logged rather than mapped onto protection flags.
        if status[9] or status[10]:
            logger.info(f"PACE Modbus alarm bits: warning 0x{status[9]:04X}, protection 0x{status[10]:04X}")

        # Register 12, bit i set means cell i+1 is balancing.
        balance = status[12]
        for cell in range(min(self.cell_count, len(self.cells))):
            self.cells[cell].balance = bool(balance & (1 << cell))

        return True

    def read_cell_data(self) -> bool:
        """
        Read the individual cell voltages.

        :return: True if success, False for failure
        """
        cells = self.read_registers(REGISTER_CELLS, REGISTER_CELLS_COUNT)
        if cells is None:
            return False

        for cell in range(min(self.cell_count, len(self.cells))):
            millivolt = cells[cell]
            if CELL_VOLTAGE_MIN_MV <= millivolt <= CELL_VOLTAGE_MAX_MV:
                self.cells[cell].voltage = millivolt / 1000
            else:
                # Keep the last known value rather than publishing a spike.
                logger.debug(f"Ignoring implausible voltage {millivolt} mV on cell {cell + 1}")

        return True

    def read_temperature_data(self) -> bool:
        """
        Read the battery, MOSFET and ambient temperature sensors.

        :return: True if success, False for failure
        """
        temperatures = self.read_registers(REGISTER_TEMPERATURE, REGISTER_TEMPERATURE_COUNT)
        if temperatures is None:
            return False

        # Registers 31-34 are the battery sensors, 35 is the MOSFET. Sensor 0 is
        # the MOSFET slot in to_temperature(). Register 36 holds the ambient
        # temperature, for which the driver has no field.
        for sensor, register in enumerate([1, 2, 3, 4, 0]):
            raw = temperatures[sensor]
            if raw == TEMPERATURE_INVALID:
                continue
            self.to_temperature(register, self.to_signed_int(raw) / 10)

        return True
