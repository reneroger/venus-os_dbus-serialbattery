# -*- coding: utf-8 -*-

# Notes
# Added by https://github.com/KoljaWindeler

from battery import Battery, Cell
from utils import SOC_CALCULATION, read_serial_data, logger
from time import sleep
import sys


# Protocol versions that appear in the frame header. 25 is paceic, 20 is what
# the BMS display calls PYLON_485. Both use the same framing, so the driver can
# simply try them in turn.
PROTOCOL_VERSIONS = (b"25", b"20")

# Pack selector in the INFO field of the analog value request. 0xFF asks for all
# packs, which some firmwares answer with an empty data set -- those need the
# pack number instead.
PACK_INFO_ALL = b"FF"

# Settling time between the two reads in read_frame(). The header read leaves the
# rest of the frame in flight; querying again immediately gets no answer at all.
# 0.1 s was enough on the test hardware, 0.2 s leaves room.
FRAME_PROBE_PAUSE = 0.2


class Pace(Battery):
    def __init__(self, port, baud, address):
        super(Pace, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE
        self.unique_identifier_tmp = ""
        self.cell_count = 0
        self.address = address
        self.poll_interval = 5000
        self.cell_voltage_lp = 0.9

        # Both are settled once in detect_protocol() and then left alone.
        self.protocol_version = PROTOCOL_VERSIONS[0]
        self.pack_info = PACK_INFO_ALL

        # Reply length per command, learned from the first answer. See read_frame().
        self.frame_lengths = {}

    BATTERYTYPE = "PACE RS232"
    LENGTH_CHECK = 0  # ignored
    LENGTH_POS = 2  # ignored
    LENGTH_SIZE = "H"  # ignored

    @property
    def address_ascii(self) -> bytes:
        """The BMS address as two ASCII hex digits, as the frame carries it."""
        if isinstance(self.address, bytes):
            value = self.address[0] if len(self.address) > 0 else 0
        else:
            value = int(self.address)
        return f"{value:02X}".encode("ascii")

    def _build_command(self, cmd_part: bytes) -> bytes:
        # 1. Basis-Header (SOI + Version)
        soi_and_version = b"\x7e" + self.protocol_version

        # 2. Adresse  b'\x01' -> b'01')
        addr_ascii = self.address_ascii

        # 3. connect
        cmd_without_crc = soi_and_version + addr_ascii + cmd_part

        # 4. Checksu
        cal_chk = 0
        for i in range(1, len(cmd_without_crc)):
            cal_chk += cmd_without_crc[i]

        cal_chk = 0xFFFF - cal_chk % 65536 + 1
        crc_ascii = f"{cal_chk:04X}".encode("ascii")

        logger.debug((cmd_without_crc + crc_ascii + b"\x0d").hex(" "))
        return cmd_without_crc + crc_ascii + b"\x0d"

    @property
    def command_status(self):
        return self._build_command(b"4642E002" + self.pack_info)

    @property
    def command_software_version(self):
        return self._build_command(b"46C10000")

    @property
    def command_serial_nr(self):
        return self._build_command(b"46C20000")

    @property
    def command_fuses(self):
        return self._build_command(b"4644E00201")

    def read_frame(self, command: bytes):
        """
        Read a reply whose length is only known from the reply itself.

        read_serial_data() has to be told upfront how many bytes to wait for, but
        the length of a paceic reply depends on how many cells and temperature
        sensors the pack reports -- a 16S pack with 5 sensors answers with 132
        bytes, one with 6 sensors with 136. Asking for the wrong number makes the
        read time out.

        So the header is fetched first, which carries the payload length, and the
        result is remembered per command. Only the first read of each command
        costs an extra round trip.

        :param command: the complete frame to send
        :return: the reply, or False on failure
        """
        length = self.frame_lengths.get(command)

        if length is None:
            # 13 bytes cover SOI, version, address, CID1, CID2 and the length field.
            header = read_serial_data(
                command,
                self.port,
                self.baud_rate,
                self.LENGTH_POS,
                self.LENGTH_CHECK,
                13,
                self.LENGTH_SIZE,
                battery_online=self.online,
            )
            if not header or len(header) < 13 or header[0] != 0x7E:
                return False

            try:
                # LENID holds the payload length, plus 13 header and 5 trailing bytes.
                # One less than that is requested on purpose: read_serialport_data()
                # loops `while len(data) <= length`, so asking for the exact size
                # never satisfies the condition and runs into the timeout. Both
                # implementations read in chunks, so the full frame arrives anyway.
                # This is the reason for the "should be 58 ... so we use 57" note
                # on the fixed lengths this method replaces.
                length = 13 + int(header[10:13], 16) + 5 - 1
            except ValueError:
                logger.debug("PACE: could not read the payload length from the header")
                return False

            self.frame_lengths[command] = length
            logger.debug(f"PACE: reply length for this command is {length + 1} bytes")

            # Let the rest of the header reply drain before asking again.
            sleep(FRAME_PROBE_PAUSE)

        data = self.read_serial_data_pace(command, length)
        if data is False:
            # A pack that changed its cell or sensor count invalidates the length.
            self.frame_lengths.pop(command, None)
        return data

    @staticmethod
    def status_offsets(status_data) -> tuple:
        """
        Work out where the fields behind the variable-length blocks start.

        The analog value reply carries the cell voltages and then the temperature
        readings, both prefixed by their count. Everything after that -- current,
        voltage, capacities, cycles -- therefore sits at an offset that depends on
        how many cells and sensors the pack has. Packs with 5 and with 6
        temperature sensors are both in the field.

        :param status_data: the raw reply
        :return: cell count, temperature count, offset of the current field
        """
        cell_count = int(status_data[17:19], 16)
        temperature_count_offset = 19 + cell_count * 4
        temperature_count = int(status_data[temperature_count_offset : temperature_count_offset + 2], 16)
        current_offset = temperature_count_offset + 2 + temperature_count * 4
        return cell_count, temperature_count, current_offset

    def detect_protocol(self) -> bool:
        """
        Find the protocol version and pack selector this BMS answers to.

        Tries every combination until one returns an answer that actually
        contains cell voltages. A pack that only supports one of them replies to
        the others either not at all or with an empty data set.

        :return: True if a working combination was found, False otherwise
        """
        for version in PROTOCOL_VERSIONS:
            for pack_info in (PACK_INFO_ALL, self.address_ascii):
                self.protocol_version = version
                self.pack_info = pack_info

                status_data = self.read_frame(self.command_status)
                if status_data is False:
                    continue

                try:
                    cell_count, _, _ = self.status_offsets(status_data)
                    # An empty data set reports its cells as 0 mV.
                    first_cell = int(status_data[19:23], 16)
                except (ValueError, IndexError):
                    continue

                if cell_count > 0 and first_cell > 0:
                    logger.info(f"PACE: protocol v{version.decode()}, pack info {pack_info.decode()}, {cell_count} cells")
                    return True

                logger.debug(f"PACE: v{version.decode()} with pack info {pack_info.decode()} answered without measurements")

        # Leave the defaults in place so the log shows what was tried last.
        self.protocol_version = PROTOCOL_VERSIONS[0]
        self.pack_info = PACK_INFO_ALL
        return False

    def test_connection(self):
        # call a function that will connect to the battery, send a command and retrieve the result.
        # The result or call should be unique to this BMS. Battery name or version, etc.
        # Return True if success, False for failure
        try:
            if not self.detect_protocol():
                return False

            result = self.get_settings()
            result = result and self.read_status_data()
        except Exception:
            (
                exception_type,
                exception_object,
                exception_traceback,
            ) = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Exception occurred: {repr(exception_object)} of type {exception_type} in {file} line #{line}")
            result = False

        return result

    def get_settings(self):
        # After successful  connection get_settings will be call to set up the battery.
        # Set the current limits, populate cell count, etc
        # Return True if success, False for failure
        logger.debug("requesting settings")
        status_data = self.read_frame(self.command_status)
        # check if connection success
        if status_data is False:
            return False

        self.cell_count, _, current_offset = self.status_offsets(status_data)
        for i in range(0, self.cell_count):
            self.cells.append(Cell(False))

        # cycles
        self.cycles = int(status_data[current_offset + 18 : current_offset + 22], 16)

        # capacity
        self.capacity_remain = int(status_data[current_offset + 8 : current_offset + 12], 16) / 100

        # ######################### SOFTWARE VERSION #############################
        logger.debug("requesting software version")
        # 13 + 5 + 40, should be 58, but doesn't ever trigger, so we use 57, that works fine
        status_data = self.read_frame(self.command_software_version)
        # Some firmwares answer this with an empty frame, which is short. Reading
        # 20 fields out of it would run past the end.
        if status_data is False or len(status_data) < 13 + 40:
            logger.debug("PACE: no software version reported")
        else:
            h = ""
            for i in range(20):  # 20 byte
                h = h + chr(status_data[2 * i + 13]) + chr(status_data[2 * i + 14])
            sw_version = bytearray.fromhex(h).decode("utf-8")  # will be overridden
            logger.debug(sw_version)
            self.hardware_version = sw_version
        # ######################### SERIAL NR #############################
        logger.debug("requesting serial nr")
        # 13 + 5 + 40, should be 58, but doesn't ever trigger, so we use 57, that works fine
        status_data = self.read_frame(self.command_serial_nr)
        if status_data is False or len(status_data) < 13 + 40:
            logger.debug("PACE: no serial number reported")
        else:
            h = ""
            for i in range(20):  # 20 byte
                if not (chr(status_data[2 * i + 13]) == "2" and chr(status_data[2 * i + 14] == "0")):
                    h = h + chr(status_data[2 * i + 13]) + chr(status_data[2 * i + 14])
            serial = bytearray.fromhex(h).decode("utf-8")  # will be overridden
            logger.debug(serial)
            self.unique_identifier_tmp = serial

        # Set fet status once, because it is not available from the BMS
        self.charge_fet = True
        self.discharge_fet = True
        self.balance_fet = True

        return True

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure
        try:
            result = self.read_fuses_data()
            result = result and self.read_status_data()
            return result
        except Exception:
            return False

    def read_fuses_data(self):
        logger.debug("run read_fuses_data()")
        # same here, should be 96 but we have to request 95 .. have to check why
        status_data = self.read_frame(self.command_fuses)
        if status_data is False:
            return False
        # ## reset all warnings ###
        # low capacity alarm
        if not SOC_CALCULATION:
            self.protection.low_soc = 0  # not available at pace
        # pack over voltage alarm [done]
        self.protection.high_voltage = 0
        # over current alarm [done]
        self.protection.high_charge_current = 0
        # discharge over current alarm [done]
        self.protection.high_discharge_current = 0
        # unit undervoltage alarm [done]
        self.protection.low_cell_voltage = 0
        # core differential pressure alarm # not implemented
        self.protection.cell_imbalance = 0
        # MOSFET temperature alarm [done]
        self.protection.high_internal_temp = 0
        # battery overtemperature alarm OR overtemperature alarm in the battery box [done]
        self.protection.high_charge_temp = 0
        self.protection.low_temperature = 0
        # check if low/high temp alarm arise during discharging [done]
        self.protection.high_temperature = 0
        self.protection.low_temperature = 0
        # ## reset all warnings ###

        # location of data depends on number of detected cells and detected temp sensors
        cells = int(status_data[17:19], 16)
        temps = int(status_data[19 + cells * 2 : 21 + cells * 2], 16)

        # see protect state below
        # #######################
        #  logger.error("warning cells:"+str(cells))
        # for i in range(0,cells):
        #     logger.error("warning cell "+str(i)+":"+str(int(status_data[19+i*2:21+i*2])))
        #  logger.error("warning temps:"+str(temps))
        # for i in range(0,temps):
        #     logger.error("warning temps "+str(i)+":"+str(int(status_data[21+cells*2+i*2:23+cells*2+i*2])))
        # ########################

        #  See warn bytes below, lets ignore this here
        # pack_charge_current_warn =    int(status_data[23+cells*2+temps*2:23+cells*2+temps*2+2], 16)
        # pack_discharge_current_warn = int(status_data[25+cells*2+temps*2:25+cells*2+temps*2+2], 16)
        # pack_voltage_warn =           int(status_data[24+cells*2+temps*2:24+cells*2+temps*2+2], 16)

        # #### Protect State 1 #####
        # bit0: Cell voltage high
        # bit1: Cell voltage low
        # bit2: Pack voltage high
        # bit3: Pack voltage low
        # bit4: charge current alarm
        # bit5: dischange current alarm
        # bit6: Short circuite
        # discharge under voltage alarm
        protect_state1 = int(status_data[26 + cells * 2 + temps * 2 : 26 + cells * 2 + temps * 2 + 2], 16)
        if protect_state1.to_bytes(1, "big")[0] & b"\x01"[0]:
            self.protection.high_cell_voltage = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x02"[0]:
            self.protection.low_cell_voltage = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x04"[0]:
            self.protection.high_voltage = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x08"[0]:
            self.protection.low_voltage = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x10"[0]:
            self.protection.high_charge_current = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x20"[0]:
            self.protection.high_discharge_current = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x40"[0]:  # short -> over current
            self.protection.high_charge_current = 2
        if protect_state1.to_bytes(1, "big")[0] & b"\x80"[0]:  # discharge under voltage -> voltage low
            self.protection.low_voltage = 2

        # #### Protect State 2 #####
        # bit0: charge temp high
        # bit1: discharge temp high
        # bit2: charge temp low
        # bit3: discharge temp low
        # bit4: MOS temp high
        # bit5: outside temp high
        # bit6: outside temp low
        protect_state2 = int(status_data[27 + cells * 2 + temps * 2 : 27 + cells * 2 + temps * 2 + 2], 16)
        if protect_state2.to_bytes(1, "big")[0] & b"\x01"[0]:
            self.protection.high_charge_temp = 2
        if protect_state2.to_bytes(1, "big")[0] & b"\x02"[0]:
            self.protection.high_temperature = 2
        if protect_state2.to_bytes(1, "big")[0] & b"\x04"[0]:
            self.protection.low_temperature = 2
        if protect_state2.to_bytes(1, "big")[0] & b"\x08"[0]:
            self.protection.low_temperature = 2
        if protect_state2.to_bytes(1, "big")[0] & b"\x10"[0]:
            self.protection.high_internal_temp = 2
        if protect_state2.to_bytes(1, "big")[0] & b"\x20"[0]:
            self.protection.high_charge_temp = 2
            self.protection.high_temperature = 2
        if protect_state2.to_bytes(1, "big")[0] & b"\x40"[0]:
            self.protection.low_temperature = 2
            self.protection.low_temperature = 2

        # instruction_state =           int(status_data[28+cells*2+temps*2:28+cells*2+temps*2+2], 16)
        # control_state =               int(status_data[29+cells*2+temps*2:29+cells*2+temps*2+2], 16)

        # #### Faulty State #####
        # bit0: Charge MOS faulty
        # bit1: Dischange MOS faulty
        # bit2: NTC faulty
        # bit3: undefined
        # bit4: Cell faulty
        # bit5: Sample fault
        fault_state = int(status_data[30 + cells * 2 + temps * 2 : 30 + cells * 2 + temps * 2 + 2], 16)
        if fault_state.to_bytes(1, "big")[0] & b"\x01"[0]:
            logger.error("Charge MOS fault")
        if fault_state.to_bytes(1, "big")[0] & b"\x02"[0]:
            logger.error("Discharge MOS fault")
        if fault_state.to_bytes(1, "big")[0] & b"\x04"[0]:
            logger.error("NTC fault")
        if fault_state.to_bytes(1, "big")[0] & b"\x10"[0]:
            logger.error("Cell fault")
        if fault_state.to_bytes(1, "big")[0] & b"\x20"[0]:
            logger.error("Sample fault")

        # ##### Calance State #####
        balance_state1 = int(status_data[31 + cells * 2 + temps * 2 : 31 + cells * 2 + temps * 2 + 2], 16)
        # cell 0..7
        balance_state2 = int(status_data[32 + cells * 2 + temps * 2 : 32 + cells * 2 + temps * 2 + 2], 16)
        # cell 8..15
        mask1 = b"\x01"[0]
        mask2 = b"\x10"[0]
        for c in range(8):
            if len(self.cells) >= c:
                if balance_state1.to_bytes(1, "big")[0] & mask1:
                    self.cells[c].balance = True
                else:
                    self.cells[c].balance = False
            if len(self.cells) >= c + 8:
                if balance_state2.to_bytes(1, "big")[0] & mask2:
                    self.cells[c + 8].balance = True
                else:
                    self.cells[c + 8].balance = False
            mask1 = mask1 << 1
            mask2 = mask2 << 1

        # #### Warn State 1 #####
        # bit0: cell voltage high
        # bit1: cell voltage low
        # bit2: pack voltage high
        # bit3: pack voltage low
        # bit4: charge current high
        # bit5: discharge current high
        warn_state1 = int(status_data[33 + cells * 2 + temps * 2 : 33 + cells * 2 + temps * 2 + 2], 16)
        if protect_state1.to_bytes(1, "big")[0] & b"\x01"[0]:
            self.protection.high_cell_voltage = 1
        if protect_state1.to_bytes(1, "big")[0] & b"\x02"[0]:
            self.protection.low_cell_voltage = 1
        if protect_state1.to_bytes(1, "big")[0] & b"\x04"[0]:
            self.protection.high_voltage = 1
        if protect_state1.to_bytes(1, "big")[0] & b"\x08"[0]:
            self.protection.low_voltage = 1
        if protect_state1.to_bytes(1, "big")[0] & b"\x10"[0]:
            self.protection.high_charge_current = 1
        if protect_state1.to_bytes(1, "big")[0] & b"\x20"[0]:
            self.protection.high_discharge_current = 1

        # #### Warn State 2 #####
        # bit0: charge temp high
        # bit1: discharge temp high
        # bit2: charge temp low
        # bit3: discharge temp low
        # bit4: outside temp high
        # bit5: outside temp low
        # bit6: MOS temp high
        # bit7: low power warning
        warn_state2 = int(status_data[34 + cells * 2 + temps * 2 : 34 + cells * 2 + temps * 2 + 2], 16)
        if protect_state2.to_bytes(1, "big")[0] & b"\x01"[0]:
            self.protection.high_charge_temp = 1
        if protect_state2.to_bytes(1, "big")[0] & b"\x02"[0]:
            self.protection.high_temperature = 1
        if protect_state2.to_bytes(1, "big")[0] & b"\x04"[0]:
            self.protection.low_temperature = 1
        if protect_state2.to_bytes(1, "big")[0] & b"\x08"[0]:
            self.protection.low_temperature = 1
        if protect_state2.to_bytes(1, "big")[0] & b"\x10"[0]:
            self.protection.high_internal_temp = 1
        if protect_state2.to_bytes(1, "big")[0] & b"\x20"[0]:
            self.protection.temp_low_internal = 1
        if protect_state2.to_bytes(1, "big")[0] & b"\x40"[0]:
            self.protection.high_charge_temp = 1
            self.protection.high_temperature = 1

        # logger.debug("Pack charg current warn "+str(pack_charge_current_warn))
        # logger.debug("Pack voltage current warn "+str(pack_voltage_warn))
        # logger.debug("Pack discharg current warn "+str(pack_discharge_current_warn))
        logger.debug("Protect state 1 " + str(protect_state1))
        logger.debug("Protect state 2 " + str(protect_state2))
        # logger.debug("Instruction state "+str(instruction_state))
        # logger.debug("control state "+str(control_state))
        logger.debug("fault state " + str(fault_state))
        logger.debug("balance state 1 " + str(balance_state1))
        logger.debug("balance state 2 " + str(balance_state2))
        logger.debug("warn state 1 " + str(warn_state1))
        logger.debug("warn state 2 " + str(warn_state2))

        return True

    def read_status_data(self):
        logger.debug("run read_status_data()")
        status_data = self.read_frame(self.command_status)
        # check if connection success
        if status_data is False:
            return False

        #        logger.error("sucess we have data")
        #        be = ''.join(format(x, ' 02X') for x in status_data)
        #        logger.error(be)

        self.cell_count, temperature_sensor_count, current_offset = self.status_offsets(status_data)
        logger.debug("Cellcount: " + str(self.cell_count))

        for i in range(0, self.cell_count):
            n_v = int(status_data[19 + i * 4 : 19 + i * 4 + 4], 16) / 1000
            if self.cells[i].voltage is None or self.cells[i].voltage == 0:
                self.cells[i].voltage = n_v
                logger.debug("NOT low passing " + str(self.cells[i].voltage))
            else:
                self.cells[i].voltage = self.cell_voltage_lp * self.cells[i].voltage
                self.cells[i].voltage += (1.0 - self.cell_voltage_lp) * n_v
                logger.debug("low passing " + str(n_v) + " to " + str(self.cells[i].voltage))
            logger.debug("Cell Voltage [" + str(i) + "]: " + str(self.cells[i].voltage))

        temperature_offset = 19 + self.cell_count * 4 + 2
        logger.debug("Temp sensor count: " + str(temperature_sensor_count))
        for i in range(0, temperature_sensor_count):
            v = round((int(status_data[temperature_offset + i * 4 : temperature_offset + i * 4 + 4], 16) / 10) - 273, 1)
            logger.debug("Temperature [" + str(i) + "]: " + str(v))
            if i < 4:  # 0,1,2,3 are internal temps
                self.to_temperature(i + 1, v)
            if i == 4:  # mosfet
                self.to_temperature(0, v)

        # Battery voltage
        self.voltage = int(status_data[current_offset + 4 : current_offset + 8], 16) / 1000

        # Battery ampere
        if status_data[current_offset] & 0b1000000:
            self.current = (int(status_data[current_offset : current_offset + 4], 16) - 65535) / 100
        else:
            self.current = int(status_data[current_offset : current_offset + 4], 16) / 100

        # cycles
        self.cycles = int(status_data[current_offset + 18 : current_offset + 22], 16)

        # capacity
        self.capacity_remain = int(status_data[current_offset + 8 : current_offset + 12], 16) / 100
        self.capacity = int(status_data[current_offset + 14 : current_offset + 18], 16) / 100
        logger.debug("Capacity: " + str(self.capacity))
        logger.debug("Remaing capacity: " + str(self.capacity_remain))

        # SOC
        self.soc = self.capacity_remain * 100 / self.capacity

        # TODO?
        # self.charge_fet = True
        # self.discharge_fet = True
        # self.balance_fet = True
        # self.balancing = True

        # logging
        for c in range(self.cell_count):
            logger.debug("Cell " + str(c) + " voltage: " + str(self.cells[c].voltage) + "V")
        logger.debug("voltage: " + str(self.voltage) + "V")
        logger.debug("Current: " + str(self.current))
        logger.debug("SOC: " + str(self.soc) + "%")
        return True

    def unique_identifier(self) -> str:
        """
        Used to identify a BMS when multiple BMS are connected
        """
        return self.unique_identifier_tmp

    def get_min_cell(self):
        min_voltage = 9999
        min_cell = None
        for c in range(min(len(self.cells), self.cell_count)):
            if self.cells[c].voltage is not None and min_voltage > self.cells[c].voltage:
                min_voltage = self.cells[c].voltage
                min_cell = c
        return min_cell

    def get_max_cell(self):
        max_voltage = 0
        max_cell = None
        for c in range(min(len(self.cells), self.cell_count)):
            if self.cells[c].voltage is not None and max_voltage < self.cells[c].voltage:
                max_voltage = self.cells[c].voltage
                max_cell = c
        return max_cell

    def read_serial_data_pace(self, command: str, length: int) -> bool:
        """
        use the read_serial_data() function to read the data and then do BMS specific checks (crc, start bytes, etc)
        :param command: the command to be sent to the bms
        :return: True if everything is fine, else False
        """
        data = read_serial_data(
            command,
            self.port,
            self.baud_rate,
            self.LENGTH_POS,  # ignored
            self.LENGTH_CHECK,  # ignored
            length,
            self.LENGTH_SIZE,  # ignored
            battery_online=self.online,
        )

        # Check for the failure case before logging: read_serial_data() returns
        # False on timeout, and False has no .hex().
        if data is False:
            return False

        logger.debug("received:")
        logger.debug(data.hex(" "))

        if data[0] == 0x7E:
            logger.debug("SOI found")
            logger.debug("Version: " + chr(data[1]) + chr(data[2]))
            logger.debug("Adr: " + chr(data[3]) + chr(data[4]))
            received_addr_int = int(data[3:5], 16)
            current_addr_int = self.address[0] if isinstance(self.address, bytes) else int(self.address)
            if received_addr_int != current_addr_int:
                logger.error(">>> ERROR: return address incorrect expected " + str(current_addr_int) + " but got " + str(received_addr_int))
                return False

            payload_length = int(data[10:13], 16)
            if len(data) >= (13 + payload_length + 5):
                # CRC check
                cal_chk = 0
                for i in range(1, len(data) - 5):
                    cal_chk += data[i]
                cal_chk = 0xFFFF - cal_chk % 65536 + 1
                # CRC check
                if cal_chk == int(data[-5:-1], 16):
                    logger.debug("CRC correct, return data")
                    return data
                else:
                    logger.error(">>>ERROR: CRC incorrect")
            else:
                logger.error(">>> ERROR: length incorrect, expected " + str(13 + payload_length + 5) + " but received " + str(len(data)))
                return False
        else:
            logger.error(">>> ERROR: Incorrect Startbyte ")
            return False
