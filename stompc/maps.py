from classes import Pump,MapConfig


##################################
### BASELINE MAP WITH ONE PUMP ###
##################################
def get_baseline_one_pump_config():
    pump = Pump(-4.5,-7.2)
    fake_pump = Pump(0,-8.5)
    config = MapConfig(pumps=[pump], fake_pumps=[fake_pump], n_cells_in_area=24480)
    return config




##################################
### BASELINE MAP WITH ONE PUMP ###
##################################
def get_baseline_big_room_config():
    pump = Pump(0,14)

    config = MapConfig(pumps=[pump], fake_pumps=[], n_cells_in_area=24480)
    return config


def get_baseline_tetris_room_config():
    pump = Pump(4,-4)
    fake_pump = Pump(0,-7)

    config = MapConfig(pumps=[pump], fake_pumps=[fake_pump], n_cells_in_area=26928)
    return config


###################################
### BASELINE MAP WITH TWO PUMPS ###
###################################
def get_baseline_two_pumps_config():
    pump = Pump(3.6, 4.2)
    pump2 = Pump(2.2, 4.4)
    config = MapConfig(pumps=[pump, pump2])
    return config