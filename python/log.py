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
    def __init__(self, fmt):
        super().__init__()
        self.fmt = fmt
        self.FORMATS = {
            logging.DEBUG: f"{LogColors.BLUE}{self.fmt}{LogColors.RESET}",
            logging.INFO: f"{LogColors.GREEN}{self.fmt}{LogColors.RESET}",
            logging.WARNING: f"{LogColors.YELLOW}{self.fmt}{LogColors.RESET}",
            logging.ERROR: f"{LogColors.RED}{self.fmt}{LogColors.RESET}",
            logging.CRITICAL: f"{LogColors.RED}{self.fmt}{LogColors.RESET}",
        }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

# 创建日志工具类
class CustomLogger:
    def __init__(self, name):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)

        # 创建控制台处理器并设置级别为 DEBUG
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)

        # 定义日志输出格式，使用 logging 的格式化占位符
        log_format = "%(asctime)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"
        formatter = ColoredFormatter(fmt=log_format)

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

# 使用示例
if __name__ == "__main__":
    log = CustomLogger(__name__)

    log.debug("This is a debug message")
    log.info("This is an info message")
    log.warning("This is a warning message")
    log.error("This is an error message")
    log.critical("This is a critical message")
