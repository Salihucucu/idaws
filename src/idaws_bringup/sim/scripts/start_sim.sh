#!/usr/bin/env bash
# IDAWS Full Simulation
#
# Opens 3 terminals:
#   1. Gazebo (deniz.sdf)
#   2. ArduPilot SITL (Rover + MAVProxy)
#   3. ROS 2 IDAWS nodes + gz bridge (LiDAR → /scan, pymavlink → telemetry)
#
# Usage: bash src/idaws_bringup/sim/scripts/start_sim.sh

set -e

ARDUPILOT_DIR="${ARDUPILOT_DIR:-/home/ben/ardupilot}"
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Dünya ve Ezel modeli artık repoda versiyonlu (docker imajı da bunları kullanır).
# SITL_Models'a bağımlılık kalmadı; oradaki kopya sadece tarihsel.
WORLD_FILE="${IDAWS_WORLD:-$SIM_DIR/worlds/deniz.sdf}"
PARM_FILE="$SIM_DIR/config/sitl_rover.parm"
CUSTOM_LOCATION="51.566151,-4.034345,10.0,-135"
IDAWS_WS="/home/ben/projects/idaws"

echo "=== IDAWS USV Full Simulation ==="
echo ""

# 1. Gazebo
echo "[1/3] Gazebo..."
gnome-terminal --title="IDAWS Gazebo" -- bash -c "
  source ~/.bashrc 2>/dev/null
  # model://Ezel repodaki kopyadan çözülsün (SITL_Models gerekmez)
  export GZ_SIM_RESOURCE_PATH=\"$SIM_DIR/models:$SIM_DIR/worlds:\$GZ_SIM_RESOURCE_PATH\"
  gz sim -v4 -r $WORLD_FILE
  exec bash
" &

sleep 3

# 2. SITL
echo "[2/3] ArduPilot SITL..."
gnome-terminal --title="IDAWS SITL Rover" -- bash -c "
  source ~/.bashrc 2>/dev/null
  $ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py \
    -v Rover \
    --model JSON \
    --add-param-file $PARM_FILE \
    --console --map \
    --custom-location=$CUSTOM_LOCATION
  exec bash
" &

sleep 8

# 3. ROS 2 nodes (bridge + IDAWS)
echo "[3/3] ROS 2 IDAWS nodes..."
gnome-terminal --title="IDAWS ROS2 Nodes" -- bash -c "
  source ~/.bashrc 2>/dev/null
  source $IDAWS_WS/install/setup.bash 2>/dev/null
  ros2 launch idaws_bringup sim_launch.py
  exec bash
" &

echo ""
echo "=== 3 terminals opened ==="
echo "  [Gazebo]  deniz.sdf (LiDAR: /lidar, kamera: /camera/image_raw — gz topic)"
echo "  [SITL]    Rover + MAVProxy (udp:14550)"
echo "  [ROS 2]   bridge(/lidar→/scan, /camera/image_raw) + pymavlink + all nodes"
echo ""
echo "  Test:"
echo "    gz topic -l | grep -E 'lidar|camera'      # Gazebo tarafi yayin yapiyor mu"
echo "    ros2 topic hz /scan"
echo "    ros2 topic hz /camera/image_raw"
echo "    ros2 topic echo telemetry/gps"
echo "    http://localhost:5000                      # kamera + LiDAR paneli"
echo ""
