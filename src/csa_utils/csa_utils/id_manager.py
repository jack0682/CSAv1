# id_manager.py

import time
import uuid

class FrameCounter:
    def __init__(self, start=0):
        self.frame_id = start

    def next(self):
        self.frame_id += 1
        return self.frame_id

    def reset(self):
        self.frame_id = 0

class RunIDManager:
    def __init__(self, prefix="run"):
        ts = int(time.time())
        self.run_id = f"{prefix}_{ts}"

    def get(self):
        return self.run_id

class UniqueIDPool:
    def __init__(self):
        self.issued = set()

    def allocate(self):
        while True:
            uid = str(uuid.uuid4())[:8]
            if uid not in self.issued:
                self.issued.add(uid)
                return uid

    def release(self, uid):
        self.issued.discard(uid)
