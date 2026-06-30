# logging_config.py
import logging
import os


def setup_logging(log_dir="logs"):
    os.makedirs(log_dir, exist_ok=True)

    fmt = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(name)s | %(filename)s:%(lineno)d | %(message)s"
    )

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # file handler
    fh = logging.FileHandler(os.path.join(log_dir, "app.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)

    # console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(fmt)

    root.handlers.clear()
    root.addHandler(fh)
    root.addHandler(ch)
