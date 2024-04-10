from classes import Pump,MapConfig


##################################
### BASELINE MAP WITH ONE PUMP ###
##################################
def get_baseline_one_pump_config():
    pump = Pump(3.5,4.2)
    config = MapConfig(pumps=[pump])
    return config


###################################
### BASELINE MAP WITH TWO PUMPS ###
###################################
def get_baseline_two_pumps_config():
    pump = Pump(3.6, 4.2)
    pump2 = Pump(2.2, 4.4)
    config = MapConfig(pumps=[pump, pump2])
    return config