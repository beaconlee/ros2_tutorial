import logging
import inspect

# 定义颜色代码
class LogColors:
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


# 自定义日志处理器
class ColoredFormatter(logging.Formatter):
    def __init__(self, fmt, datefmt=None):
        super().__init__(fmt=fmt, datefmt=datefmt)
        self.fmt = fmt
        self.datefmt = datefmt
        self.FORMATS = {
            logging.DEBUG: f"{LogColors.BLUE}{self.fmt}{LogColors.RESET}",
            logging.INFO: f"{LogColors.GREEN}{self.fmt}{LogColors.RESET}",
            logging.WARNING: f"{LogColors.YELLOW}{self.fmt}{LogColors.RESET}",
            logging.ERROR: f"{LogColors.RED}{self.fmt}{LogColors.RESET}",
            logging.CRITICAL: f"{LogColors.RED}{self.fmt}{LogColors.RESET}",
        }

    def format(self, record):
        # 使用自定义字段替换 filename 和 lineno
        record.custom_filename = record.__dict__.get("custom_filename", record.filename)
        record.custom_lineno = record.__dict__.get("custom_lineno", record.lineno)

        record.levelname = record.levelname[0]  # 缩写日志级别为首字母
        record.custom_filename = record.custom_filename.split("/")[-1]
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt=self.datefmt)
        return formatter.format(record)


# 单例的 CustomLogger 实现
class CustomLogger:
    _instance = None

    def __new__(cls, name):
        if cls._instance is None:
            cls._instance = super(CustomLogger, cls).__new__(cls)
            cls._instance._initialize(name)
        return cls._instance

    def _initialize(self, name):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)

        if not self.logger.handlers:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.DEBUG)

            log_format = "[%(levelname)s] %(asctime)s %(custom_filename)s:%(custom_lineno)d] %(message)s"
            date_format = "%Y-%m-%d %H:%M:%S"
            formatter = ColoredFormatter(fmt=log_format, datefmt=date_format)

            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

    def _log_with_caller_info(self, level, message):
        # 使用 inspect 获取调用者的信息
        frame = inspect.stack()[2]
        module = inspect.getmodule(frame[0])
        filename = module.__file__ if module else "unknown"
        lineno = frame.lineno

        # 使用自定义字段保存文件名和行号
        extra = {"custom_filename": filename, "custom_lineno": lineno}
        self.logger.log(level, message, extra=extra)

    def debug(self, message):
        self._log_with_caller_info(logging.DEBUG, f"{message}")

    def info(self, message):
        self._log_with_caller_info(logging.INFO, f"{message}")

    def warning(self, message):
        self._log_with_caller_info(logging.WARNING, f"{message}")

    def error(self, message):
        self._log_with_caller_info(logging.ERROR, f"{message}")

    def critical(self, message):
        self._log_with_caller_info(logging.CRITICAL, f"{message}")

    def set_loglevel(self, level):
        self.logger.setLevel(level)


log = CustomLogger("beacon")
log.set_loglevel(logging.DEBUG)
