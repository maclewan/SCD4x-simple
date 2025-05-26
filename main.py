import time

from scd4x_simple import SCD4xSensirion, I2cAdapter
from machine import I2C, Pin


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    adaptor = I2cAdapter(i2c)
    # sensor
    sen = SCD4xSensirion(adaptor)
    # Force return sensor in IDLE mode!
    # Принудительно перевожу датчик в режим IDLE!
    sen.start_measurement(start=False, single_shot=False)
    sid = sen.get_id()
    print(f"Sensor id: {sid}")
    # t_offs = 0.0
    # Warning: To change or read sensor settings, the SCD4x must be in idle mode!!!
    # Otherwise an EIO exception will be raised!
    # print(f"Set temperature offset sensor to {t_offs} Celsius")
    # sen.set_temperature_offset(t_offs)
    t_offs = sen.get_temperature_offset()
    print(f"Get temperature offset from sensor: {t_offs} Celsius")
    masl = 160  # Meter Above Sea Level
    print(f"Set my place M.A.S.L. to {masl} meter")
    sen.set_altitude(masl)
    masl = sen.get_altitude()
    print(f"Get M.A.S.L. from sensor: {masl} meter")
    # data ready
    if sen.get_data_status():
        print("Measurement data can be read!")  # Данные измерений могут быть прочитаны!
    else:
        print("Measurement data missing!")
    
    # отключаю автоматическую калибровку! Для искусственных помещений (indoor) она должна быть отключена!
    sen.set_auto_calibration(value=False)
    
    if sen.is_auto_calibration():
        print("The automatic self-calibration is ON!")
    else:
        print("The automatic self-calibration is OFF!")

    sen.start_measurement(start=True, single_shot=False)      # periodic start
    wt = sen.get_conversion_cycle_time()
    print(f"conversion cycle time [ms]: {wt}")
    print("Periodic measurement started")
    repeat = 50
    multiplier = 2
    for i in range(repeat):
        time.sleep_ms(wt)
        _meas_data = sen.get_measurement_value()
        print(f"{_meas_data}")
    
    print(20 * "_")
    if sen.is_single_shot_mode():
        print("Single shot mode!")
    if sen.is_continuously_mode():
        print("Continuously mode!")
    print(20 * "_")
    print("Reading using an iterator!")
    for counter, _meas_data in enumerate(sen):
        time.sleep_ms(wt)
        if not _meas_data is None:
            print(f"CO2 [ppm]: {_meas_data.CO2}; T [°C]: {_meas_data.T}; RH [%]: {_meas_data.RH}")
        else:
            print("Measurement data missing!")
        if repeat == counter:
            break

    print(20 * "*_")
    print("Using single shot mode!")
    # Force return sensor in IDLE mode!
    # Принудительно перевожу датчик в режим IDLE!
    sen.start_measurement(start=False, single_shot=False)
    cnt = 0
    while True:
        sen.start_measurement(start=False, single_shot=True, rht_only=False)
        time.sleep_ms(multiplier * wt)      # 3x period
        _meas_data = sen.get_measurement_value()
        print(f"CO2 [ppm]: {_meas_data.CO2}; T [°C]: {_meas_data.T}; RH [%]: {_meas_data.RH}")
        cnt += 1
        if cnt > repeat:
            break

    # Принудительно перевожу датчик в режим IDLE!
    # sen.set_measurement(start=False, single_shot=False)
    sen.start_measurement(start=False, single_shot=True, rht_only=True)   # rht only mode!
    wt = sen.get_conversion_cycle_time()
    print(20 * "*_")
    # Использование режима измерения по запросу! Только относительная влажность и температура измеряются датчиком!
    # относительная влажность + температура. CO2 равна нулю или не изменяется!!!
    print("Using single shot mode! RH + T only! (Temp + RH. CO2 always zero or does not change!!)")
    while True:
        time.sleep_ms(multiplier * wt)      # 3x period
        _meas_data = sen.get_measurement_value()
        print(f"CO2 [ppm]: {_meas_data.CO2}; T [°C]: {_meas_data.T}; RH [%]: {_meas_data.RH}")
        sen.start_measurement(start=False, single_shot=True, rht_only=True)   # rht only mode!
