#!/usr/bin/env python3
import os
import time

from docopt import docopt

import car as car
# from donkeycar.parts.datastore import TubHandler
from car.parts.actuator import PCA9685, PWMSteering, PWMThrottle


def drive(cfg):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    # Initialize car
    # V = car.vehicle.Vehicle()
    #
    # class MyController:
    #     '''
    #     a simple controller class that outputs a constant steering and throttle.
    #     '''
    #
    #     def run(self):
    #         steering = 0.0
    #         throttle = 0.1
    #         return steering, throttle
    #
    # V.add(MyController(), outputs=['angle', 'throttle'])
    #
    # # Drive train setup
    # # Channel 1
    # steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    # steering = PWMSteering(controller=steering_controller,
    #                        left_pulse=cfg.STEERING_LEFT_PWM,
    #                        right_pulse=cfg.STEERING_RIGHT_PWM)
    #
    # #Channel 0
    # throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    # throttle = PWMThrottle(controller=throttle_controller,
    #                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
    #                        zero_pulse=cfg.THROTTLE_STOPPED_PWM,
    #                        min_pulse=cfg.THROTTLE_REVERSE_PWM)
    #
    # V.add(steering, inputs=['angle'])
    # V.add(throttle, inputs=['throttle'])
    #
    # # run the vehicle for 20 seconds
    # V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
    #         max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
#    args = docopt(__doc__)
    # cfg = car.load_config(config_path='/home/dicong/piCar/cfg_basic.py')
    # drive(cfg)
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA
    import busio

    i2c = busio.I2C(SCL, SDA)

    pwm = PCA9685(i2c)
    pwm.frequency = 60
    # self.pwm.i2c_device = (get_bus, address)

    from adafruit_servokit import ServoKit

    kit = ServoKit(channels=16)
    kit.servo[0].angle = 20
    kit.continuous_servo[1].throttle = 0.1
    import time

    t_end = time.time() + 5
    i = 5
    while time.time() < t_end:
        kit.servo[0].angle = 20 + i
        kit.continuous_servo[1].throttle = 0.1
        i += 5
        time.sleep(1)

#	drive(cfg)
