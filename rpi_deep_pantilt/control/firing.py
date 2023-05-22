import logging

logging.basicConfig()
LOGLEVEL = logging.getLogger().getEffectiveLevel()

class FireController:
    def Fire(self):
        logging.warning("pew")

    def ReadyToFire(self):
        logging.warning("arming")