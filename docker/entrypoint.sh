#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/ari_ws/install/setup.bash
source /opt/experiment_ws/install/setup.bash
if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
fi
mkdir -p /data
ari_xvfb_pid=''
ari_display_dir=''
ari_cleanup() {
    if [[ -n "$ari_xvfb_pid" ]]; then
        kill "$ari_xvfb_pid" 2>/dev/null || true
        wait "$ari_xvfb_pid" 2>/dev/null || true
    fi
    if [[ -n "$ari_display_dir" ]]; then rm -rf "$ari_display_dir"; fi
}
trap ari_cleanup EXIT
if [[ "${ARI_HEADLESS:-0}" == 1 && "${LIBGL_ALWAYS_SOFTWARE:-0}" != 0 ]]; then
    # Keep X alive until ROS launch has stopped its children and flushed bags.
    ari_display_dir=$(mktemp -d /tmp/ari-display.XXXXXX)
    # Fixed display numbers allow Linux abstract sockets when the host's
    # /tmp/.X11-unix bind mount is read-only. -displayfd cannot do that.
    ari_display_ready=0
    for ari_display_number in {99..119}; do
        Xvfb ":$ari_display_number" -screen 0 1280x1024x24 -nolisten tcp \
            >"$ari_display_dir/xvfb.log" 2>&1 &
        ari_xvfb_pid=$!
        sleep 0.2
        if kill -0 "$ari_xvfb_pid" 2>/dev/null; then
            export DISPLAY=":$ari_display_number"
            if timeout 3 glxinfo -B >"$ari_display_dir/glx.log" 2>&1; then
                ari_display_ready=1
                break
            fi
            kill "$ari_xvfb_pid" 2>/dev/null || true
        fi
        wait "$ari_xvfb_pid" 2>/dev/null || true
        ari_xvfb_pid=''
    done
    if [[ "$ari_display_ready" != 1 ]]; then
        tail -25 "$ari_display_dir/xvfb.log" >&2
        echo 'Xvfb did not become ready' >&2
        exit 1
    fi
fi
# Separate the application's session from terminal/container signals. Send one
# SIGINT to ROS launch; it coordinates its children rather than receiving a
# second interrupt while the gateway is finalizing the recording.
python3 /usr/local/bin/ari-run-application "$@" <&0 &
ari_application_pid=$!
ari_stop() {
    trap '' INT TERM
    kill -TERM "$ari_application_pid" 2>/dev/null || true
}
trap ari_stop INT TERM
set +e
wait "$ari_application_pid"
ari_exit_code=$?
if kill -0 "$ari_application_pid" 2>/dev/null; then
    wait "$ari_application_pid"
    ari_exit_code=$?
fi
exit "$ari_exit_code"
