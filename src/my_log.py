import datetime


NOTSET = 0
DEBUG = 10
INFO = 20
WARNING = 30
ERROR = 40
CRITICAL = 50

CYAN = '\033[96m'
HEADER = '\033[95m'
BLUE = '\033[94m'
YELLOW = '\033[93m'
GREEN = '\033[92m'
RED = '\033[91m'
GRAY = '\033[90m'
END = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'


def cprint(level, msg, align=False):
    date = f"{GRAY}{datetime.datetime.now():%Y-%m-%d %H:%M:%S}{END}"
    prefix = ""
    if level == DEBUG:
        prefix = f" {CYAN}[{align * ' '}DEBUG{align * ' '}]{END} "
    elif level == INFO:
        prefix = f" {GREEN}[{align * ' '}INFO{align * '  '}]{END} "
    elif level == WARNING:
        prefix = f" {YELLOW}[WARNING]{END} "
    elif level == ERROR:
        prefix = f" {RED}[{align * ' '}ERROR{align * ' '}]{END} "
    elif level == CRITICAL:
        prefix = f" {BOLD}{RED}[!CRITICAL!]{END} "
        
    print(date + prefix, end="")
    print(msg, end="")  # separate prints
    print(END)


class Logger:
    def __init__(self, level: int = INFO, name: str = __name__, align: bool = False) -> None:
        self.level = level
        self.name = name
        self.align = align
        #  self._print_logger_info()
    
    def _print_logger_info(self):
        self.info(f"Logger '{BLUE}{self.name}{END}' with level {BLUE}{self.level}{END}")
        
    def debug(self, message):
        if self.level <= DEBUG:
            cprint(level=10, msg=message, align=self.align)
    
    def info(self, message):
        if self.level <= INFO:
            cprint(level=20, msg=message, align=self.align)
    
    def warning(self, message):
        if self.level <= WARNING:
            cprint(level=30, msg=message, align=self.align)
    
    def error(self, message):
        if self.level <= ERROR:
            cprint(level=40, msg=message, align=self.align)
    
    def critical(self, message):
        if self.level <= CRITICAL:
            cprint(level=50, msg=message, align=self.align)


if __name__ == "__main__":
    log = Logger(level=DEBUG, align=True)
    log.debug("Test debug")
    log.info("Test info")
    log.warning("Test warning")
    log.error("Test error")
    log.critical("Test critical")
