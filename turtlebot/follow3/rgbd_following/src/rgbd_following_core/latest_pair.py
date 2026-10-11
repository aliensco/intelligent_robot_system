"""Single pending observation: receive continuously, process only fresh work."""
import threading
from .depth import finite


class LatestPair(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.pending = None
        self.stamps = None
        self.overwritten = 0

    def put(self, pair, stamps):
        if len(stamps) != 3 or not all(finite(s) and s > 0 for s in stamps):
            return False
        with self.lock:
            if self.stamps is not None and any(s <= old for s, old in zip(stamps, self.stamps)):
                return False
            if self.pending is not None:
                self.overwritten += 1
            self.pending = pair
            self.stamps = tuple(stamps)
            self.ready.set()
        return True

    def take(self, timeout=.1):
        self.ready.wait(timeout)
        with self.lock:
            pair = self.pending
            self.pending = None
            # Producer also owns this lock when setting the event; a newly
            # queued observation cannot lose its wake-up between take/clear.
            self.ready.clear()
            return pair
