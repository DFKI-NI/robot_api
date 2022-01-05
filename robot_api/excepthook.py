from typing import Optional, Type
from types import TracebackType
import sys


class Excepthook:
    _sys_excepthook = sys.excepthook
    _expected_exception: Optional[BaseException] = None

    @staticmethod
    def _excepthook(exception_type: Type[BaseException], exception_value: BaseException,
            traceback: TracebackType) -> None:
        if exception_value == Excepthook._expected_exception:
            # Print exception without traceback for _expected_exception.
            print(f"{exception_type.__name__}: {exception_value}")
        else:
            # Otherwise, use default sys.excepthook.
            Excepthook._sys_excepthook(exception_type, exception_value, traceback)

    @staticmethod
    def expect(exception: BaseException) -> BaseException:
        """Suppress traceback if exception is raised as expected. Otherwise, use sys.excepthook by default."""
        Excepthook._expected_exception = exception
        sys.excepthook = Excepthook._excepthook
        return exception
