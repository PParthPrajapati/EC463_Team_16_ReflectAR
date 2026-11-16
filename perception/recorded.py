"""Plays a tools/record_spin.py recording back as if it were the RealSense: the same interface
as realsense.RealSenseCamera (colour frames, depth aligned to them, factory intrinsics), at the
recording's own pace, so the whole pipeline and the renderer can be run on a person who is not
in front of the camera.  server.py --recording ../test_runs/spin_... [--frames 0:900]
"""
import json, os, threading, time
import cv2


class RecordedCamera:
    def __init__(self, folder, frames=None, loop=True):
        meta = json.load(open(os.path.join(folder, "meta.json")))
        self.folder, self.loop = folder, loop
        self.intrinsics, self.depth_scale = meta["intrinsics"], meta["depth_scale"]
        self.frames = meta["frames"]
        if frames:                                       # "first:last" frame indices
            a, b = (int(x) if x else None for x in frames.split(":"))
            self.frames = [f for f in self.frames if (a is None or f["i"] >= a) and (b is None or f["i"] <= b)]
        self.usb = "3 (recording)"
        self.desc = f"recording {os.path.basename(os.path.normpath(folder))}, {len(self.frames)} frames ({meta.get('desc', '')})"
        self.frame, self.depth, self.stamp, self.seq = None, None, 0.0, 0
        self.recv = self.read_recv = 0.0
        self.index = -1                                  # recording frame index of the frame read() last returned
        self._next = None
        self.depth_hz = self.color_hz = 0.0
        self.cond = threading.Condition()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            t0, first = time.time(), self.frames[0]["t"]
            for k, f in enumerate(self.frames):
                color = cv2.imread(os.path.join(self.folder, f"c_{f['i']:05d}.jpg"))
                depth = cv2.imread(os.path.join(self.folder, f"d_{f['i']:05d}.png"), cv2.IMREAD_UNCHANGED)
                due = t0 + (f["t"] - first)
                time.sleep(max(due - time.time(), 0.0))
                now = time.time()
                with self.cond:
                    self.frame, self._next, self.stamp, self.seq = color, (depth, f["i"]), now, self.seq + 1
                    self.recv = now
                    self.cond.notify_all()
                if k % 30 == 29:
                    self.color_hz = self.depth_hz = 30.0 / max(now - self._t30, 1e-3) if hasattr(self, "_t30") else 0.0
                if k % 30 == 0:
                    self._t30 = now
            if not self.loop:
                return
            time.sleep(1.2)                              # long enough for the tracker to start afresh

    def read(self, last_seq):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=10.0 if self.seq == 0 else 1.0)
            if self._next is not None:
                self.depth, self.index = self._next
            self.read_recv = self.recv
            return self.frame, self.stamp, self.seq
