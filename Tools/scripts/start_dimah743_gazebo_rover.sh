#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
gazebo_root="/home/lh/ardupilot_gazebo_classic"
world_file="${gazebo_root}/worlds/ardupilot_rover_dimah743.world"
override_file="/tmp/rover-sitl-dimah743-skid-override.parm"
log_dir="/tmp/ardupilot-gazebo"

if [[ ! -f "${gazebo_root}/build/libArduPilotPlugin.so" ]]; then
    echo "Missing ${gazebo_root}/build/libArduPilotPlugin.so"
    echo "Build it with: cmake -S ${gazebo_root} -B ${gazebo_root}/build && cmake --build ${gazebo_root}/build -j4"
    exit 1
fi

cat > "${override_file}" <<'EOF'
GPS1_TYPE 100
SERVO2_FUNCTION 0
SERVO3_FUNCTION 74
SERVO3_REVERSED 1
EOF

mkdir -p "${log_dir}"
tmux kill-session -t gazebo 2>/dev/null || true
tmux kill-session -t rover-sitl 2>/dev/null || true
pkill -TERM -f '[m]avproxy.py.*14550' 2>/dev/null || true
pkill -TERM -f '[a]rdurover.*-I0' 2>/dev/null || true
sleep 1
pkill -KILL -f '[m]avproxy.py.*14550' 2>/dev/null || true
pkill -KILL -f '[a]rdurover.*-I0' 2>/dev/null || true

gazebo_env="source /usr/share/gazebo/setup.sh && \
export GAZEBO_MODEL_DATABASE_URI= && \
export GAZEBO_PLUGIN_PATH=${gazebo_root}/build:\${GAZEBO_PLUGIN_PATH} && \
export GAZEBO_MODEL_PATH=${gazebo_root}/models:\${GAZEBO_MODEL_PATH} && \
export GAZEBO_RESOURCE_PATH=${gazebo_root}/worlds:\${GAZEBO_RESOURCE_PATH}"

tmux new-session -d -s gazebo -n server -c "${repo_root}" \
    "source /usr/share/gazebo/setup.sh && \
     export GAZEBO_MODEL_DATABASE_URI= && \
     export GAZEBO_PLUGIN_PATH=${gazebo_root}/build:\${GAZEBO_PLUGIN_PATH} && \
     export GAZEBO_MODEL_PATH=${gazebo_root}/models:\${GAZEBO_MODEL_PATH} && \
     export GAZEBO_RESOURCE_PATH=${gazebo_root}/worlds:\${GAZEBO_RESOURCE_PATH} && \
     gzserver --verbose ${world_file} 2>&1 | tee ${log_dir}/gzserver.log"

sleep 3

tmux new-window -t gazebo -n client -c "${repo_root}" \
    "${gazebo_env} && gzclient --verbose 2>&1 | tee ${log_dir}/gzclient.log"

tmux new-session -d -s rover-sitl -c "${repo_root}" \
    "export SITL_RITW_MINIMIZE=0; \
     Tools/autotest/sim_vehicle.py -v Rover -f gazebo-rover --no-rebuild -w --console --map \
       --add-param-file libraries/AP_HAL_ChibiOS/hwdef/dimah743/defaults.parm \
       --add-param-file ${override_file} \
       -m \"--out=tcpin:0.0.0.0:14550\""

echo "Started Gazebo Classic Rover SITL:"
echo "  tmux attach -t gazebo"
echo "  tmux attach -t rover-sitl"
echo "  Mission Planner TCP: 127.0.0.1:14550"
