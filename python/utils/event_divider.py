# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import time
from threading import Event


class EventDivider:
    def __init__(self, event: Event, factor: int):
        self._event = event
        self._factor = factor

    def __call__(self):
        count = 0

        while count < self._factor:
            if not self._event.wait(1):
                return False

            count += 1
            self._event.clear()
            time.sleep(0.001)

        return True
