import datetime


NOTSET = 0
DEBUG = 10
INFO = 20
WARNING = 30
ERROR = 40
CRITICAL = 50

HEADER = '\033[95m'
BLUE = '\033[94m'
CYAN = '\033[96m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
END = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'


def cprint(string):
    date = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}"
    print(date + string)

class logger:
    def __init__(self, level=INFO, name=__name__) -> None:
        self.level = level
        self.name = name
        self.info(f"Logger '{self.name}' with level {self.level}")
    
    def debug(self, message):
        if self.level <= DEBUG:
            cprint(f" {CYAN}[DEBUG]{END} " + message)  
    
    def info(self, message):
        if self.level <= INFO:
            cprint(f" {GREEN}[INFO]{END} " + message)
    
    def warning(self, message):
        if self.level <= WARNING:
            cprint(f" {YELLOW}[WARNING]{END} " + message)
    
    def error(self, message):
        if self.level <= ERROR:
            cprint(f" {RED}[ERROR]{END} " + message)        

if __name__ == "__main__":
    log = logger(level=DEBUG)
    log.debug("Test debug")
    log.info("Test info")
    log.warning("Test warning")
    log.error("Test error")
    