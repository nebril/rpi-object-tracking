import logging
from multiprocessing import Value, Process, Manager

import pantilthat as pth
import signal
import sys
import time

from rpi_deep_pantilt.detect.camera import run_pantilt_detect
from rpi_deep_pantilt.control.pid import PIDController
from rpi_deep_pantilt.control.firing import FireController

logging.basicConfig()
LOGLEVEL = logging.getLogger().getEffectiveLevel()

RESOLUTION = (320, 320)

SERVO_MIN = -90
SERVO_MAX = 90

CENTER = (
    RESOLUTION[0] // 2,
    RESOLUTION[1] // 2
)

PIXEL_TARGET_TRESHOLD = 30

# function to handle keyboard interrupt


def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

    # disable the servos
    pth.servo_enable(1, False)
    pth.servo_enable(2, False)

    # exit
    sys.exit()


def in_range(val, start, end):
    # determine the input vale is in the supplied range
    return (val >= start and val <= end)


def set_servos(pan, tilt):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        pan_angle = -1 * pan.value
        tilt_angle = tilt.value

        # if the pan angle is within the range, pan
        if in_range(pan_angle, SERVO_MIN, SERVO_MAX):
            pth.pan(pan_angle)
        else:
            logging.info(f'pan_angle not in range {pan_angle}')

        if in_range(tilt_angle, SERVO_MIN, SERVO_MAX):
            pth.tilt(tilt_angle)
        else:
            logging.info(f'tilt_angle not in range {tilt_angle}')


def pid_process(output, p, i, d, box_coord, origin_coord, action):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PIDController(p.value, i.value, d.value)
    p.reset()

    # loop indefinitely
    while True:
        error = origin_coord - box_coord.value
        output.value = p.update(error)
        # logging.info(f'{action} error {error} angle: {output.value}')

# ('person',)
#('orange', 'apple', 'sports ball')


def pantilt_process_manager(
    model_cls,
    labels=('person',),
    rotation=0
):

    pth.servo_enable(1, True)
    pth.servo_enable(2, True)
    with Manager() as manager:
        # set initial bounding box (x, y)-coordinates to center of frame
        center_x = manager.Value('i', 0)
        center_y = manager.Value('i', 0)

        center_x.value = RESOLUTION[0] // 2
        center_y.value = RESOLUTION[1] // 2

        # pan and tilt angles updated by independent PID processes
        pan = manager.Value('i', 0)
        tilt = manager.Value('i', 0)

        # PID gains for panning

        pan_p = manager.Value('f', 0.05)
        # 0 time integral gain until inferencing is faster than ~50ms
        pan_i = manager.Value('f', 0.1)
        pan_d = manager.Value('f', 0)

        # PID gains for tilting
        tilt_p = manager.Value('f', 0.15)
        # 0 time integral gain until inferencing is faster than ~50ms
        tilt_i = manager.Value('f', 0.2)
        tilt_d = manager.Value('f', 0)

        detect_processr = Process(target=run_pantilt_detect,
                                  args=(center_x, center_y, labels, model_cls, rotation))

        pan_process = Process(target=pid_process,
                              args=(pan, pan_p, pan_i, pan_d, center_x, CENTER[0], 'pan'))

        tilt_process = Process(target=pid_process,
                               args=(tilt, tilt_p, tilt_i, tilt_d, center_y, CENTER[1], 'tilt'))

        servo_process = Process(target=set_servos, args=(pan, tilt))

        detect_processr.start()
        pan_process.start()
        tilt_process.start()
        servo_process.start()

        detect_processr.join()
        pan_process.join()
        tilt_process.join()
        servo_process.join()


if __name__ == '__main__':
    pantilt_process_manager()


def run_target_acquisition(center_x, center_y, rotation, target_acquired, target_engaged):
    while True:
        time.sleep(0.1)
        if center_x.value == -1 or center_y.value == -1:
            target_engaged.value = False
            target_acquired.value = False
            continue
        else:
            target_acquired.value = True
            logging.debug("target acquired")

        if abs(center_x.value - CENTER[0]) < PIXEL_TARGET_TRESHOLD and abs(center_y.value - CENTER[1]) < PIXEL_TARGET_TRESHOLD:
            target_engaged.value = True
            logging.debug("target engaged")
        else:
            target_engaged.value = False

def run_fire_control(target_acquired, target_engaged):
    controller = FireController()
    while True:
        time.sleep(0.1)

        if target_acquired.value == True:
            controller.ReadyToFire()
        if target_engaged.value == True:
            controller.Fire()
        if target_acquired.value == False:
            controller.Unacquire()
        if target_engaged.value == False:
            controller.Disengage()

def seek_process_manager(
    model_cls,
    labels=('person',),
    rotation=0
):

    with Manager() as manager:
        # set initial bounding box (x, y)-coordinates to center of frame
        center_x = manager.Value('i', 0)
        center_y = manager.Value('i', 0)

        center_x.value = RESOLUTION[0] // 2
        center_y.value = RESOLUTION[1] // 2

        # pan and tilt angles updated by independent PID processes
        #pan = manager.Value('i', 0)
        #tilt = manager.Value('i', 0)

        # PID gains for panning

        #pan_p = manager.Value('f', 0.05)
        ## 0 time integral gain until inferencing is faster than ~50ms
        #pan_i = manager.Value('f', 0.1)
        #pan_d = manager.Value('f', 0)

        ## PID gains for tilting
        #tilt_p = manager.Value('f', 0.15)
        ## 0 time integral gain until inferencing is faster than ~50ms
        #tilt_i = manager.Value('f', 0.2)
        #tilt_d = manager.Value('f', 0)

        target_acquired = manager.Value('b', False)
        target_engaged = manager.Value('b', False)

        detect_processr = Process(target=run_pantilt_detect,
                                  args=(center_x, center_y, labels, model_cls, rotation))

        target_aquisition_processr = Process(target=run_target_acquisition,
                                             args=(center_x, center_y, rotation, target_acquired, target_engaged))

        fire_control_processr = Process(target=run_fire_control,
                                             args=(target_acquired, target_engaged))

        #pan_process = Process(target=pid_process,
        #                      args=(pan, pan_p, pan_i, pan_d, center_x, CENTER[0], 'pan'))

        #tilt_process = Process(target=pid_process,
        #                       args=(tilt, tilt_p, tilt_i, tilt_d, center_y, CENTER[1], 'tilt'))

        #servo_process = Process(target=set_servos, args=(pan, tilt))

        detect_processr.start()
        target_aquisition_processr.start()
        fire_control_processr.start()
        #pan_process.start()
        #tilt_process.start()
        #servo_process.start()

        detect_processr.join()
        target_aquisition_processr.join()
        fire_control_processr.join()
        #pan_process.join()
        #tilt_process.join()
        #servo_process.join()