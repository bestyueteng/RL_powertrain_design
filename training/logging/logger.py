import os
import sys
import logging

class LoggerWriter:
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level
    
    def write(self, message):
        if message.strip():
            self.level(message)
    
    def flush(self):
        pass


def setup_logging(log_file):
    if os.path.exists(log_file):
        os.remove(log_file)
    
    logging.basicConfig(
        filename=log_file,
        filemode='w',
        format='%(asctime)s - %(levelname)s - %(message)s',
        level=logging.INFO
    )
    logger = logging.getLogger()
    # Optionally, also log to stdout
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    return logger