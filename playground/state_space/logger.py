import logging
import datetime


class MyFormatter(logging.Formatter):
    converter = datetime.datetime.fromtimestamp

    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            s = ct.strftime("%Y-%m-%dT%H:%M:%S,%f")
            # s = "%s,%03d" % (t, record.msecs)
        return s


def make_logger(name, level):
    logger = logging.getLogger(name)
    logger.setLevel(level)

    log_format = "%(levelname)s\t%(asctime)s\t[%(name)s, %(filename)s:%(lineno)d]\t%(message)s"
    formatter = MyFormatter(log_format)
    print_handle = logging.StreamHandler()
    print_handle.setLevel(level)
    print_handle.setFormatter(formatter)
    logger.addHandler(print_handle)

    return logger

