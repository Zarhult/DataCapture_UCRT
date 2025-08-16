#!/bin/bash
set -Eeuo pipefail

READY_FILE=/tmp/capture_ready.txt

usage() {
	echo "Usage: $0 OUT_DIR"
	echo "Example: $0 /data/run_$(date +%Y%m%d_%H%M%S)"

}

# Accept custom output dir, otherwise create with date/time
OUT_DIR="${1:-}"
if [[ -z "$OUT_DIR" ]]; then
	OUT_DIR="$HOME/joint_data/run_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$OUT_DIR/camera"

echo "Writing:"
echo " bag    -> $OUT_DIR/rosbag"
echo " images -> $OUT_DIR/camera"

# Clean exit on Ctrl-C or kill
cleanup() {
    echo "Stopping processes..."
    kill -TERM "$cap_pid" "$ros2_pid" 2>/dev/null || true
    wait "$cap_pid" "$ros2_pid" 2>/dev/null
    rm -f "$READY_FILE"
}
trap cleanup SIGINT SIGTERM EXIT

# Start both processes in the background
echo "Starting camera capture..."
python3 ~/DataCapture_UCRT/capture.py --fps 30 --exposure 30000 --flip_boson --record --indefinite --base_dir "$OUT_DIR/camera" >> "$OUT_DIR/camera.log" 2>&1 &
echo "Camera setup started."
cap_pid=$!

if [[ ! -e "$READY_FILE" ]]; then
	while read -r path _ name; do
		[[ "$path$name" == "$READY_FILE" ]] && break
	done < <(inotifywait -q -m -e create -e close_write -e moved_to --format '%w %e %f' "$(dirname "$READY_FILE")")
fi
echo "Camera ready."
echo "Starting lidar capture..."
#ros2 bag record -o "$OUT_DIR/rosbag" /ouster/points /ouster/signal_image /ouster/imu >> "$OUT_DIR/rosbag.log" 2>&1 &
ros2 bag record -o "$OUT_DIR/rosbag" /ouster/points >> "$OUT_DIR/rosbag.log" 2>&1 &
echo "Started lidar capture."
ros2_pid=$!

#on_ctrl_c() {
#	trap - INT EXIT #prevent re-entry and avoid running EXIT trap below
#	kill -INT -- -"$cap_pid" -"$ros2_pid" 2>/dev/null || true # send SIGINT to each child's process group (negative pid)
#	wait "$cap_pid" "$ros2_pid" 2>/dev/null # wait for clean finish
#	exit 130 # exit code for Ctrl-C
#}
#trap on_ctrl_c INT
#
#graceful_stop() {
#	trap - TERM EXIT
#	kill -TERM -- -"$cap_pid" -"$ros2_pid" 2>/dev/null || true
#
#	s=0
#	f=0
#	for _ in 1 2 3 4 5; do
#		kill -0 "$cap_pid" 2>/dev/null || s=1
#		kill -0 "$ros2_pid" 2>/dev/null || f=1
#		[ "$s" = 1 ] && [ "$f" = 1 ] && break
#		sleep 1
#	done
#
#	#final escalation if still hanging
#	kill -KILL -- -"$cap_pid" -"$ros2_pid" 2>/dev/null || true
#	#clean up tmp file if capture.py failed to do so
#	[ -f "$READY_FILE" ] && rm "$READY_FILE"
#}
#trap graceful_stop TERM EXIT

wait "$cap_pid" "$ros2_pid"
