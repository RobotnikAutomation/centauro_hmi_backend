import json
import threading
import time
from collections import Counter


class InboundStats:
    """Thread-safe inbound message counters for throttled diagnostics."""

    def __init__(self):
        self._lock = threading.Lock()
        self._started_at = time.monotonic()
        self._total_messages = 0
        self._total_errors = 0
        self._interval_messages = 0
        self._interval_bytes = 0
        self._interval_errors = 0
        self._by_source = Counter()
        self._by_command = Counter()
        self._last_by_command = {}

    def record(self, source, command, raw, payload=None, error=False):
        raw_size = len(raw.encode('utf-8')) if isinstance(raw, str) else len(raw)
        command = command or '<invalid>'
        with self._lock:
            self._total_messages += 1
            self._interval_messages += 1
            self._interval_bytes += raw_size
            self._by_source[source] += 1
            self._by_command[command] += 1
            if error:
                self._total_errors += 1
                self._interval_errors += 1
            if payload is not None:
                self._last_by_command[command] = payload

    def take_interval(self):
        now = time.monotonic()
        with self._lock:
            elapsed = max(now - self._started_at, 1e-9)
            result = {
                'elapsed': elapsed,
                'messages': self._interval_messages,
                'bytes': self._interval_bytes,
                'errors': self._interval_errors,
                'total_messages': self._total_messages,
                'total_errors': self._total_errors,
                'by_source': dict(self._by_source),
                'by_command': dict(self._by_command),
                'last_by_command': dict(self._last_by_command),
            }
            self._started_at = now
            self._interval_messages = 0
            self._interval_bytes = 0
            self._interval_errors = 0
            self._by_source.clear()
            self._by_command.clear()
            self._last_by_command.clear()
        return result


def compact_payload(payload, max_chars=180):
    text = json.dumps(payload, separators=(',', ':'), ensure_ascii=False)
    return text if len(text) <= max_chars else text[:max_chars - 1] + '…'
