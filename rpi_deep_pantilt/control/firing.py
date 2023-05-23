import logging
import RPi.GPIO as GPIO
import time

logging.basicConfig()
LOGLEVEL = logging.getLogger().getEffectiveLevel()

CHANNEL = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(CHANNEL, GPIO.OUT, initial=GPIO.LOW)

MIN_SWITCH_TIME = 1.0 

class FireController:
    lastFiredSwitchTime = time.perf_counter()
    firing = False

    def Fire(self):
        if self.firing:
            return

        currentTime = time.perf_counter()

        logging.warning(currentTime - self.lastFiredSwitchTime)
        if currentTime - self.lastFiredSwitchTime > MIN_SWITCH_TIME:
            logging.warning("pew")
            GPIO.output(CHANNEL, GPIO.HIGH)
            self.firing = True
            self.lastFiredSwitchTime = currentTime

    def ReadyToFire(self):
        pass
        #logging.warning("arming")

    def Disengage(self):
        if not self.firing:
            return

        currentTime = time.perf_counter()

        logging.warning(currentTime - self.lastFiredSwitchTime)
        if currentTime - self.lastFiredSwitchTime > MIN_SWITCH_TIME:
            GPIO.output(CHANNEL, GPIO.LOW)
            self.firing = False
            self.lastFiredSwitchTime = currentTime

    def Unacquire(self):
        pass