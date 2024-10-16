import logging

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
        self.datefmt = datefmt  # 保存 datefmt 以供 format 方法使用
        self.FORMATS = {
            logging.DEBUG: f"{LogColors.BLUE}{self.fmt}{LogColors.RESET}",
            logging.INFO: f"{LogColors.GREEN}{self.fmt}{LogColors.RESET}",
            logging.WARNING: f"{LogColors.YELLOW}{self.fmt}{LogColors.RESET}",
            logging.ERROR: f"{LogColors.RED}{self.fmt}{LogColors.RESET}",
            logging.CRITICAL: f"{LogColors.RED}{self.fmt}{LogColors.RESET}",
        }

    def format(self, record):
        # 对齐日志级别，固定宽度为 8（比如 'CRITICAL' 是 8 个字符）
        record.levelname = record.levelname.ljust(8)
        log_fmt = self.FORMATS.get(record.levelno)
        # formatter = logging.Formatter(log_fmt)
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

        # 防止重复添加处理器
        if not self.logger.handlers:
            # 创建控制台处理器并设置级别为 DEBUG
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.DEBUG)

            # 定义日志输出格式，使用 logging 的格式化占位符
            # 如果不加 % 和 s 的话不会格式化替换输出
            # log_format = "{asctime} - {levelname} - {filename}:{lineno} - {message}"
            log_format = "%(asctime)s %(filename)s:%(lineno)d %(levelname)s: %(message)s"
            date_format = "%d %H:%M:%S"  # 设置时间格式

            formatter = ColoredFormatter(fmt=log_format, datefmt=date_format)

            # 将格式器添加到处理器
            console_handler.setFormatter(formatter)

            # 将处理器添加到 logger
            self.logger.addHandler(console_handler)

    def debug(self, message):
        self.logger.debug(f"{message}")

    def info(self, message):
        self.logger.info(f"{message}")

    def warning(self, message):
        self.logger.warning(f"{message}")

    def error(self, message):
        self.logger.error(f"{message}")

    def critical(self, message):
        self.logger.critical(f"{message}")
