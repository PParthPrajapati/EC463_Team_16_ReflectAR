"""Record colour + aligned depth from the RealSense while the user turns in a few poses.

Writes test_runs/spin_<time>/: c_00000.jpg (colour), d_00000.png (16-bit depth aligned to the
colour pixels), meta.json (intrinsics, depth scale, per-frame capture time and pose segment).
Spoken prompts say what to do, so you don't have to watch the terminal. tools/replay_spin.py
turns the recording into the tracking stream the renderer would have received.

Run from perception/ (macOS needs sudo):  sudo .venv/bin/python tools/record_spin.py
"""
import json, os, queue, subprocess, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2
import realsense

SEGMENTS = [   # (name, seconds, spoken prompt)
    ("still", 6, "Stand two metres from the camera, facing it, arms relaxed. Hold still."),
    ("arms_down", 14, "Arms relaxed at your sides. Turn slowly all the way around, once."),
    ("arms_out", 14, "Hold your arms out a little from your sides. Turn slowly all the way around."),
    ("hands_hips", 14, "Hands on your hips. Turn slowly all the way around."),
    ("t_pose", 14, "Arms straight out to the sides. Turn slowly all the way around."),
    ("arms_up", 10, "Face the camera. Raise both arms above your head, then lower them, twice."),
    ("crossed", 10, "Face the camera. Cross your arms on your chest, then reach forward towards the camera."),
    ("sidestep", 10, "Arms relaxed. Step side to side, then turn left and right quickly."),
]
MAX_FPS = 30.0


def speak(text):
    """Start speaking; returns the process. Under sudo, root has no access to the user's audio
    session (the speech would be silent), so the voice runs as the user who called sudo."""
    cmd = ["say", "-r", "170", text]
    uid, user = os.environ.get("SUDO_UID"), os.environ.get("SUDO_USER")
    if uid and user:
        cmd = ["launchctl", "asuser", uid, "sudo", "-u", user] + cmd
    print(f"\n>>> {text}\n", flush=True)
    try:
        return subprocess.Popen(cmd)
    except OSError:
        return None


def say(text, wait=True):
    p = speak(text)
    if p and wait:
        p.wait()


def open_camera(cfg, tries=4):
    """macOS's camera service (UVCAssistant) claims the colour interface as soon as the camera
    enumerates, and librealsense then fails with "failed to set power state". Stop the service
    (launchd restarts it on demand) and open the camera straight away, without a reset, which
    would hand it back to the service."""
    # the service comes back within a fraction of a second and claims the camera again, so it
    # is kept down until our streams are running (after that the camera is ours)
    stop = threading.Event()

    def keep_down():
        while not stop.is_set():
            subprocess.run(["killall", "-9", "UVCAssistant"], capture_output=True)
            time.sleep(0.05)

    if sys.platform == "darwin":
        threading.Thread(target=keep_down, daemon=True).start()
        time.sleep(0.3)
    os.environ["RS_NO_RESET"] = "1"
    err = None
    try:
        for attempt in range(tries):
            if attempt:
                try:
                    realsense._reset(realsense.rs.context().query_devices()[0])
                except (RuntimeError, IndexError) as e:
                    print(f"[record] reset failed ({e})", flush=True)
                    time.sleep(3)
            try:
                return realsense.RealSenseCamera({**cfg, "start_tries": 1})
            except RuntimeError as e:
                err = e
                print(f"[record] attempt {attempt + 1}/{tries}: {e}", flush=True)
    finally:
        stop.set()
    raise SystemExit(f"Could not open the RealSense: {err}")


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(os.path.dirname(here), "test_runs", time.strftime("spin_%Y%m%d_%H%M%S"))
    os.makedirs(out)
    cfg = json.load(open(os.path.join(here, "calib", "config.json")))["camera"].get("realsense", {})
    cam = open_camera(cfg)
    print(cam.desc, flush=True)

    q = queue.Queue()

    def writer():
        while True:
            item = q.get()
            if item is None:
                return
            i, color, depth = item
            cv2.imwrite(os.path.join(out, f"c_{i:05d}.jpg"), color, [cv2.IMWRITE_JPEG_QUALITY, 94])
            cv2.imwrite(os.path.join(out, f"d_{i:05d}.png"), depth, [cv2.IMWRITE_PNG_COMPRESSION, 1])

    th = threading.Thread(target=writer)
    th.start()
    frames, seq, n, last = [], 0, 0, 0.0
    say("Camera ready. You have ten seconds to get in position, two metres from the camera.")
    time.sleep(10)
    for name, secs, prompt in SEGMENTS:
        t_prompt = time.time()
        speaking = speak(prompt)
        end = None
        while True:
            frame, stamp, seq = cam.read(seq)
            now = time.time()
            if end is None and (speaking is None or speaking.poll() is not None):
                end = now + secs                         # the segment's time starts when the prompt ends
            if end is not None and now > end:
                break
            if frame is None or stamp - last < 1.0 / MAX_FPS - 0.002:
                continue
            last = stamp
            q.put((n, frame, cam.depth))
            frames.append({"i": n, "t": stamp, "segment": name, "prompting": end is None})
            n += 1
        print(f"[segment] {name} done, {n} frames so far ({time.time() - t_prompt:.0f} s)", flush=True)
    say("Done. Thank you.", wait=False)
    q.put(None)
    th.join()
    json.dump({"intrinsics": cam.intrinsics, "depth_scale": cam.depth_scale, "desc": cam.desc,
               "segments": [s[0] for s in SEGMENTS], "frames": frames},
              open(os.path.join(out, "meta.json"), "w"))
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if uid:                                              # recorded as root: hand the files back
        for f in [out] + [os.path.join(out, x) for x in os.listdir(out)]:
            os.chown(f, int(uid), int(gid))
    dur = frames[-1]["t"] - frames[0]["t"] if frames else 0
    print(f"wrote {n} frames ({n / max(dur, 1e-3):.1f} fps over {dur:.0f} s) to {out}", flush=True)


if __name__ == "__main__":
    main()
