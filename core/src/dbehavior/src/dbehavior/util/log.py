import logging
import os
import time

logger = logging.getLogger('behavior')
logger.setLevel(logging.DEBUG)

logFolder = os.path.expanduser('~/dancer-log')
if not os.path.exists(logFolder):
    os.makedirs(logFolder)
logFile = os.path.join(logFolder, time.strftime("%y%m%d%H%M%S.log"))

formatter = logging.Formatter('[%(levelname)s][%(asctime)s] %(message)s',
                              '%m-%d %H:%M:%S')

fileHandler = logging.FileHandler(logFile)
fileHandler.setFormatter(formatter)
logger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
logger.addHandler(consoleHandler)


def debug(*args, **kwargs):
    logger.debug(*args, **kwargs)


def info(*args, **kwargs):
    logger.info(*args, **kwargs)


def warning(*args, **kwargs):
    logger.warning(*args, **kwargs)


def error(*args, **kwargs):
    logger.error(*args, **kwargs)


def critical(*args, **kwargs):
    logger.critical(*args, **kwargs)
