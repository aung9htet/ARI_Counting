"""Relay one shutdown signal while preserving the application's stdin."""
import signal
import subprocess
import sys


def reset_child_signals():
    # Bash background jobs inherit SIGINT ignored; restore it before exec.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)


child = subprocess.Popen(sys.argv[1:], start_new_session=True,
                         preexec_fn=reset_child_signals)
stopping = False


def stop(signum, frame):
    global stopping
    if not stopping and child.poll() is None:
        stopping = True
        child.send_signal(signal.SIGINT)


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
code = child.wait()
sys.exit(code if code >= 0 else 128 - code)
