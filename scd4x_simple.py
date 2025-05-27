"""SCD4x Sensirion module"""

from collections import namedtuple
import micropython
import time
import math
from machine import I2C, SPI, Pin
import struct

serial_number_scd4x = namedtuple("serial_number_scd4x", "word_0 word_1 word_2")
measured_values_scd4x = namedtuple("measured_values_scd4x", "CO2 T RH")


def _calc_crc(sequence) -> int:
    """Wrapper for a short call."""
    return _crc8(sequence, polynomial=0x31, init_value=0xFF)


def _crc8(sequence: bytes, polynomial: int, init_value: int = 0x00, final_xor=0x00):
    mask = 0xFF
    crc = init_value & mask
    for item in sequence:
        crc ^= item & mask
        for _ in range(8):
            if crc & 0x80:
                crc = mask & ((crc << 1) ^ polynomial)
            else:
                crc = mask & (crc << 1)
    return crc ^ final_xor


def _mpy_bl(value: int) -> int:
    if 0 == value:
        return 0
    return 1 + int(math.log2(abs(value)))


class BusAdapter:
    def __init__(self, bus: [I2C, SPI]):
        self.bus = bus

    def get_bus_type(self) -> type:
        return type(self.bus)

    def read_register(self, device_addr: [int, Pin], reg_addr: int, bytes_count: int) -> bytes:
        raise NotImplementedError

    def write_register(self, device_addr: [int, Pin], reg_addr: int, value: [int, bytes, bytearray],
                       bytes_count: int, byte_order: str):
        raise NotImplementedError

    def read(self, device_addr: [int, Pin], n_bytes: int) -> bytes:
        raise NotImplementedError

    def read_to_buf(self, device_addr: [int, Pin], buf: bytearray) -> bytes:
        raise NotImplementedError

    def write(self, device_addr: [int, Pin], buf: bytes):
        raise NotImplementedError

    def write_const(self, device_addr: [int, Pin], val: int, count: int):
        if 0 == count:
            return
        # bl = val.bit_length()     # bit_length() MicroPython
        bl = _mpy_bl(val)
        if bl > 8:
            raise ValueError(f"The value must take no more than 8 bits! Current: {bl}")
        _max = 16
        if count < _max:
            _max = count
        repeats = count // _max
        b = bytearray([val for _ in range(_max)])
        for _ in range(repeats):
            self.write(device_addr, b)
        remainder = count - _max * repeats
        if remainder:
            b = bytearray([val for _ in range(remainder)])
            self.write(device_addr, b)

    def read_buf_from_memory(self, device_addr: [int, Pin], mem_addr, buf, address_size: int):
        raise NotImplementedError

    def write_buf_to_memory(self, device_addr: [int, Pin], mem_addr, buf):
        raise NotImplementedError


class I2cAdapter(BusAdapter):
    def __init__(self, bus: I2C):
        super().__init__(bus)

    def write_register(self, device_addr: int, reg_addr: int, value: [int, bytes, bytearray],
                       bytes_count: int, byte_order: str):
        buf = None
        if isinstance(value, int):
            buf = value.to_bytes(bytes_count, byte_order)
        if isinstance(value, (bytes, bytearray)):
            buf = value

        return self.bus.writeto_mem(device_addr, reg_addr, buf)

    def read_register(self, device_addr: int, reg_addr: int, bytes_count: int) -> bytes:
        return self.bus.readfrom_mem(device_addr, reg_addr, bytes_count)

    def read(self, device_addr: int, n_bytes: int) -> bytes:
        return self.bus.readfrom(device_addr, n_bytes)

    def read_to_buf(self, device_addr: int, buf: bytearray) -> bytes:
        self.bus.readfrom_into(device_addr, buf)
        return buf

    def write(self, device_addr: int, buf: bytes):
        return self.bus.writeto(device_addr, buf)

    def read_buf_from_memory(self, device_addr: int, mem_addr, buf, address_size: int = 1):
        self.bus.readfrom_mem_into(device_addr, mem_addr, buf)
        return buf

    def write_buf_to_memory(self, device_addr: int, mem_addr, buf):
        return self.bus.writeto_mem(device_addr, mem_addr, buf)


@micropython.native
def _check_value(value: [int, None], valid_range: [range, tuple], error_msg: str) -> [int, None]:
    if value is None:
        return value
    if value not in valid_range:
        raise ValueError(error_msg)
    return value


class Device:
    def __init__(self, adapter: BusAdapter, address: [int, Pin], big_byte_order: bool):
        self.adapter = adapter
        self.address = address
        # for I2C. byte order in register of device
        self.big_byte_order = big_byte_order
        # for SPI ONLY. SPI.firstbit can be SPI.MSB or SPI.LSB
        self.msb_first = True

    def _get_byteorder_as_str(self) -> tuple:
        """Return byteorder as string"""
        if self.is_big_byteorder():
            return 'big', '>'
        return 'little', '<'

    def pack(self, fmt_char: str, *values) -> bytes:
        if not fmt_char:
            raise ValueError("Invalid fmt_char parameter!")
        bo = self._get_byteorder_as_str()[1]
        return struct.pack(bo + fmt_char, values)

    def unpack(self, fmt_char: str, source: bytes, redefine_byte_order: str = None) -> tuple:
        if not fmt_char:
            raise ValueError("Invalid fmt_char parameter!")
        bo = self._get_byteorder_as_str()[1]
        if redefine_byte_order is not None:
            bo = redefine_byte_order[0]
        return struct.unpack(bo + fmt_char, source)

    @micropython.native
    def is_big_byteorder(self) -> bool:
        return self.big_byte_order


class DeviceEx(Device):

    def read_reg(self, reg_addr: int, bytes_count=2) -> bytes:
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

    # BaseSensor
    def write_reg(self, reg_addr: int, value: [int, bytes, bytearray], bytes_count) -> int:
        byte_order = self._get_byteorder_as_str()[0]
        return self.adapter.write_register(self.address, reg_addr, value, bytes_count, byte_order)

    def read_reg_16(self, address: int, signed: bool = False) -> int:
        _raw = self.read_reg(address, 2)
        return self.unpack("h" if signed else "H", _raw)[0]

    def write_reg_16(self, address: int, value: int):
        self.write_reg(address, value, 2)

    def read(self, n_bytes: int) -> bytes:
        return self.adapter.read(self.address, n_bytes)

    def read_to_buf(self, buf) -> bytes:
        return self.adapter.read_to_buf(self.address, buf)

    def write(self, buf: bytes):
        return self.adapter.write(self.address, buf)

    def read_buf_from_mem(self, address: int, buf, address_size: int = 1):
        return self.adapter.read_buf_from_memory(self.address, address, buf, address_size)

    def write_buf_to_mem(self, mem_addr, buf):
        return self.adapter.write_buf_to_memory(self.address, mem_addr, buf)


class Iterator:
    def __iter__(self):
        return self

    def __next__(self):
        raise NotImplementedError


class IBaseSensorEx:
    def get_conversion_cycle_time(self) -> int:
        raise NotImplemented

    def start_measurement(self):
        raise NotImplemented

    def get_measurement_value(self, value_index: int):
        raise NotImplemented

    def get_data_status(self):
        raise NotImplemented

    def is_single_shot_mode(self) -> bool:
        raise NotImplemented

    def is_continuously_mode(self) -> bool:
        raise NotImplemented


class SCD4xSensirion(IBaseSensorEx, Iterator):
    """Class for work with Sensirion SCD4x sensor"""

    def __init__(self, adapter: BusAdapter, address=0x62,
                 this_is_scd41: bool = True, check_crc: bool = True):
        """If check_crs is True, then each data packet received from the sensor is checked for correctness by
        calculating the checksum.
        If this_is_scd41 == True then methods for SCD41 will be available,
        otherwise GENERAL methods for SCD40/41 will be available!"""
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=True)
        self._buf_3 = bytearray((0 for _ in range(3)))
        self._buf_9 = bytearray((0 for _ in range(9)))
        self.check_crc = check_crc
        # power mode
        self._low_power_mode = False
        # measurement mode (single shot, continuous)
        self._single_shot_mode = False
        self._continuous_mode = False
        self._rht_only = False
        self._isSCD41 = this_is_scd41
        self.byte_order = self._connection._get_byteorder_as_str()

    def _get_local_buf(self, bytes_for_read: int) -> [None, bytearray]:
        if bytes_for_read not in (0, 3, 9):
            raise ValueError(f"Invalid value for bytes_for_read: {bytes_for_read}")
        if not bytes_for_read:
            return None
        if 3 == bytes_for_read:
            return self._buf_3
        return self._buf_9

    def _to_bytes(self, value: int, length: int) -> bytes:
        byteorder = self.byte_order[0]
        return value.to_bytes(length, byteorder)

    def _send_command(self, cmd: int, value: [bytes, None],
                      wait_time: int = 0, bytes_for_read: int = 0,
                      crc_index: range = None,
                      value_index: tuple = None) -> [bytes, None]:
        _conn = self._connection
        raw_cmd = self._to_bytes(cmd, 2)
        raw_out = raw_cmd
        if value:
            raw_out += value
            raw_out += self._to_bytes(_calc_crc(value), length=1)
        _conn.write(raw_out)
        if wait_time:
            time.sleep_ms(wait_time)
        if not bytes_for_read:
            return None
        b = self._get_local_buf(bytes_for_read)
        _conn.read_to_buf(buf=b)
        _check_value(len(b), (bytes_for_read,),
                     f"Invalid buffer length for cmd: {cmd}. Received {len(b)} out of {bytes_for_read}")
        if self.check_crc:
            crc_from_buf = [b[i] for i in crc_index]  # build list of CRC from buf
            calculated_crc = [_calc_crc(b[rng.start:rng.stop]) for rng in value_index]
            if crc_from_buf != calculated_crc:
                raise ValueError(f"Invalid CRC! Calculated{calculated_crc}. From buffer {crc_from_buf}")
        return b

    # Advanced features
    def save_config(self):
        """Configuration settings such as temperature offset, sensor altitude are stored by default only in volatile memory
        (RAM) and will be lost after a power cycle. The method saves the current configuration in the EEPROM of the
        SCD4x, saving it when the power is turned off. To avoid unnecessary wear on the EEPROM, the method should only
        be called if necessary(!) and if actual configuration changes have been made.
        EEPROM is guaranteed to withstand at least 2000 write cycles to failure (!)"""
        cmd = 0x3615
        self._send_command(cmd, None, 800)

    def get_id(self) -> serial_number_scd4x:
        """Return 3 words of unique serial number can be used to identify
        the chip and to verify the presence of the sensor."""
        cmd = 0x3682
        b = self._send_command(cmd, None, 0, bytes_for_read=9,
                               crc_index=range(2, 9, 3), value_index=(range(2), range(3, 5), range(6, 8)))
        _gen = ((b[i] << 8) | b[i + 1] for i in range(0, 9, 3))
        return serial_number_scd4x(word_0=next(_gen), word_1=next(_gen), word_2=next(_gen))

    def soft_reset(self):
        """I deliberately did not use the perfom_factory_reset command, so that it would be impossible to spoil the
        sensor programmatically, since the number of write cycles to the internal FLASH memory of the
        sensor is limited!"""
        cmd = 0x3632
        self._send_command(cmd, None, 1200)

    def exec_self_test(self) -> bool:
        """"The feature can be used as an end-of-line test to check sensor functionality and the customer power
        supply to the sensor. Returns True when the test is successful."""
        cmd = 0x3639
        length = 3
        b = self._send_command(cmd, None, wait_time=10_000,  # yes, wait 10 seconds!
                               bytes_for_read=length, crc_index=range(2, 3), value_index=(range(2),))
        res = self._connection.unpack("H", b)[0]
        return 0 == res

    def reinit(self) -> None:
        """The reinit command reinitializes the sensor by reloading user settings from EEPROM.
        Before sending the reinit command, the stop_measurement method must be called.
        If the reinit command does not trigger the desired re-initialization,
        a power-cycle should be applied to the SCD4x."""
        cmd = 0x3646
        self._send_command(cmd, None, 20)

    # On-chip output signal compensation
    def set_temperature_offset(self, offset: float):
        """The temperature offset has no influence on the SCD4x CO 2 accuracy. Setting the temperature offset of the SCD4x
        inside the customer device correctly allows the user to leverage the RH and T output signal. Note that the
        temperature offset can depend on various factors such as the SCD4x measurement mode, self-heating of close
        components, the ambient temperature and air flow.
        The method should be called only in IDLE sensor mode!
        𝑇 𝑜𝑓𝑓𝑠𝑒𝑡_𝑎𝑐𝑡𝑢𝑎𝑙 = 𝑇 𝑆𝐶𝐷40 − 𝑇 𝑅𝑒𝑓𝑒𝑟𝑒𝑛𝑐𝑒 + 𝑇 𝑜𝑓𝑓𝑠𝑒𝑡_ 𝑝𝑟𝑒𝑣𝑖𝑜𝑢𝑠"""
        cmd = 0x241D
        offset_raw = self._to_bytes(int(374.49142857 * offset), 2)
        self._send_command(cmd, offset_raw, 1)

    def get_temperature_offset(self) -> float:
        """The method should be called only in IDLE sensor mode!"""
        cmd = 0x2318
        b = self._send_command(cmd, None, wait_time=1, bytes_for_read=3, crc_index=range(2, 3), value_index=(range(2),))
        temp_offs = self._connection.unpack("H", b)[0]
        return 0.0026702880859375 * temp_offs

    def set_altitude(self, masl: int):
        """Reading and writing sensor height must be done when the SCD4x is in standby mode. As a rule, the height of the
        sensor is set once after the installation of the device. To save the configuration to EEPROM, you must execute
        the save_config method. By default, the sensor height is set to 0 meters above sea level (masl).
        The method should be called only in IDLE sensor mode!"""
        cmd = 0x2427
        masl_raw = self._to_bytes(masl, 2)
        self._send_command(cmd, masl_raw, 1)

    def get_altitude(self) -> int:
        """The method should be called only in IDLE sensor mode!"""
        cmd = 0x2322
        b = self._send_command(cmd, None, wait_time=1, bytes_for_read=3, crc_index=range(2, 3), value_index=(range(2),))
        return self._connection.unpack("H", b)[0]

    def set_ambient_pressure(self, pressure: float):
        """The method can be called during periodic measurements to enable continuous pressure compensation.
        Note that setting the ambient pressure using set_ambient_pressure overrides any pressure compensation based
        on the previously set sensor height. The use of this command is highly recommended for applications with
        significant changes in ambient pressure to ensure sensor accuracy."""
        cmd = 0xE000
        press_raw = self._to_bytes(int(pressure // 100), 2)  # Pascal // 100
        self._send_command(cmd, press_raw, 1)

    # Field calibration
    def force_recalibration(self, target_co2_concentration: int) -> int:
        """Please read '3.7.1 perform_forced_recalibration'. target_co2_concentration [ppm CO2]"""
        _check_value(target_co2_concentration, range(2 ** 16),
                     f"Invalid target CO2 concentration: {target_co2_concentration} ppm")
        cmd = 0x362F
        target_raw = self._to_bytes(target_co2_concentration, 2)
        b = self._send_command(cmd, target_raw, 400, 3, crc_index=range(2, 3), value_index=(range(2),))
        return self._connection.unpack("h", b)[0]

    def is_auto_calibration(self) -> bool:
        """Please read '3.7.3 get_automatic_self_calibration_enabled'"""
        cmd = 0x2313
        b = self._send_command(cmd, None, 1, 3, crc_index=range(2, 3), value_index=(range(2),))
        return 0 != self._connection.unpack("H", b)[0]

    def set_auto_calibration(self, value: bool):
        """Please read '3.7.2 set_automatic_self_calibration_enabled'"""
        cmd = 0x2416
        value_raw = self._to_bytes(int(value), length=2)
        self._send_command(cmd, value_raw, wait_time=1)

    def start_measurement(self, start: bool, single_shot: bool = False, rht_only: bool = False):
        """Used to start or stop periodic measurements. single_shot = False. rht_only is not used!
        And also to start a SINGLE measurement. single_shot = True. rht_only is used!
        If rht_only == True then the sensor does not calculate CO2 and it will be zero! See get_meas_data() method
        start is used only when False == single_shot (periodic mode)"""
        if single_shot:
            return self._single_shot_meas(rht_only)
        return self._periodic_measurement(start)

    # Basic Commands
    def _periodic_measurement(self, start: bool):
        """Start periodic measurement. In low power mode, signal update interval is approximately 30 seconds.
        In normal power mode, signal update interval is approximately 5 seconds.
        If start == True then measurement started, else stopped.
        To read the results, use the get_meas_data method."""
        wt = 0
        if start:
            cmd = 0x21AC if self._low_power_mode else 0x21B1
        else:  # stop periodic measurement
            cmd = 0x3F86
            wt = 500
        self._send_command(cmd, None, wt)
        self._continuous_mode = start
        self._single_shot_mode = False
        self._rht_only = False

    def get_measurement_value(self, value_index: int = 0) -> [None, measured_values_scd4x]:
        """Read sensor data output. The measurement data can only be read out once per signal update interval
        as the buffer is emptied upon read-out. See get_conversion_cycle_time()!"""
        cmd = 0xEC05
        val_index = (range(2), range(3, 5), range(6, 8))
        b = self._send_command(cmd, None, 1, bytes_for_read=9,
                               crc_index=range(2, 9, 3), value_index=val_index)
        words = [self._connection.unpack("H", b[val_rng.start:val_rng.stop])[0] for val_rng in val_index]
        #       CO2 [ppm]           T, Celsius              Relative Humidity, %
        return measured_values_scd4x(CO2=words[0], T=-45 + 0.0026703288 * words[1], RH=0.0015259022 * words[2])

    def get_data_status(self) -> bool:
        """Return data ready status"""
        cmd = 0xE4B8
        b = self._send_command(cmd, None, 1, 3, crc_index=range(2, 3), value_index=(range(2),))
        result = 0 != (self._connection.unpack(fmt_char="H", source=b)[0] & 0x7FF)
        return result

    @micropython.native
    def get_conversion_cycle_time(self) -> int:
        """returns the data conversion time of the sensor, depending on its settings. ms."""
        if self.is_single_shot_mode() and self.is_rht_only():
            return 50
        return 5000

    # SCD41 only
    def set_power(self, value: bool):
        """Please read '3.10.3 power_down' and '3.10.4 wake_up'"""
        if not self._isSCD41:
            return
        cmd = 0x36F6 if value else 0x36E0
        wt = 20 if value else 1
        self._send_command(cmd, None, wt)

    def _single_shot_meas(self, rht_only: bool = False):
        """Only for SCD41. Single shot measurement!
        After calling this method, the results will be ready in about 5 seconds!
        To read the results, use the get_meas_data method.
        SCD41 features a single shot measurement mode, i.e. allows for on-demand measurements.
        Please see '3.10 Low power single shot (SCD41)'"""
        if not self._isSCD41:
            return
        cmd = 0x2196 if rht_only else 0x219D
        self._send_command(cmd, None, 0)
        self._continuous_mode = False
        self._single_shot_mode = True
        self._rht_only = rht_only

    def is_single_shot_mode(self) -> bool:
        return self._single_shot_mode

    def is_continuously_mode(self) -> bool:
        return self._continuous_mode and not self.is_single_shot_mode()

    def is_rht_only(self) -> bool:
        return self._rht_only

    # Iterator
    def __iter__(self):
        return self

    def __next__(self) -> [None, measured_values_scd4x]:
        if self.is_single_shot_mode():
            return None
        if self.is_continuously_mode() and self.get_data_status():
            return self.get_measurement_value(0)
        return None


class SimpleSCD4x(SCD4xSensirion):
    """
    Simple SCD4x sensor class providing most common and basic setup out of the box.
    Returns initialized SCD4x sensor with 0 temperature offset, altitude set to 120 meters,
    auto calibration disabled
    """

    def __init__(
        self,
        *,
        scl: int,
        sda: int,
        i2c_frequency: int = 400_000,
        altitude: int = 120,
        temperature_offset: float = 0.0,
        address=0x62,
        this_is_scd41: bool = True,
        check_crc: bool = True
    ):
        i2c = I2C(0, scl=Pin(scl), sda=Pin(sda), freq=i2c_frequency)
        adapter = I2cAdapter(i2c)
        super().__init__(adapter=adapter, address=address, this_is_scd41=this_is_scd41, check_crc=check_crc)
        self.set_altitude(altitude)
        self.set_temperature_offset(temperature_offset)
        self.set_auto_calibration(value=False)

    def run_measurements(self):
        """Locking measurements function. Locks main thread until measurements are ready."""
        wt = self.get_conversion_cycle_time()
        self.start_measurement(start=False, single_shot=True, rht_only=False)
        time.sleep_ms(int(1.5 * wt))
        return self.get_measurement_value()

    def ensure_idle(self):
        attempts = 20
        for _ in range(attempts):
            time.sleep(2)
            try:
                self.get_id()
                return True
            except OSError:
                continue
        raise RuntimeError(f"Failed to ensure SCD4x is in IDLE mode after {attempts} attempts.")

    def sleep(self):
        try:
            self.set_power(False)
        except OSError:
            pass
        self._low_power_mode = True

    def wake_up(self):
        try:
            self.set_power(True)
        except OSError:
            pass
        self.ensure_idle()
        self._low_power_mode = False
        return True
