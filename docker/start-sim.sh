#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# IDAWS tam simülasyon — konteyner içi başlatıcı.
#
# Host'taki start_sim.sh üç gnome-terminal açar; konteynerde terminal yok, o
# yüzden dört bileşen tek süreç ağacında arka planda çalışır ve log'ları
# /idaws/logs altına yazılır. Bileşenler sırayla ve HAZIR OLDUKLARI DOĞRULANARAK
# başlatılır — sabit sleep'ler yavaş makinelerde yarış durumu üretiyordu.
#
#   1. Gazebo              (ArduPilotPlugin udp:9002'yi dinler)
#   2. ArduPilot SITL      (ardurover, JSON modeliyle Gazebo'ya bağlanır)
#   3. MAVProxy            (tcp:5760 → udp:14550, ROS tarafının beklediği uç)
#   4. ROS 2 node'ları     (gz köprüleri + IDAWS otonomi node'ları)
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

WS="${IDAWS_WS:-/idaws}"
SHARE="$WS/install/idaws_bringup/share/idaws_bringup"
WORLD="${IDAWS_WORLD:-$SHARE/sim/worlds/deniz.sdf}"
PARM="${IDAWS_PARM:-$SHARE/sim/config/sitl_rover.parm}"
HOME_POS="${IDAWS_HOME:-51.566151,-4.034345,10.0,-135}"
RUN_DIR="${IDAWS_RUN_DIR:-$WS/run}"
LOG_DIR="$WS/logs"

mkdir -p "$RUN_DIR" "$LOG_DIR"
PIDS=()
NAMES=()

log() { echo "[start-sim] $*"; }

# Izlenecek surec kaydi — kapanista SIGTERM gonderilecek ve olen bilesenin
# adi loglanacak.
track() { PIDS+=("$1"); NAMES+=("$2"); }

cleanup() {
    # ÖNCE node süreçlerine DOĞRUDAN sinyal — `ros2 launch` üzerinden değil.
    # Sebep: launch arka plan işi olarak başlatıldığından POSIX gereği kabuk
    # onun SIGINT'ini SIG_IGN yapıyor (SigIgn maskesinde SIGINT biti set) ve
    # CPython devraldığı SIG_IGN'i değiştirmediği için launch sinyali hiç
    # görmüyor. SIGTERM ise launch'un çocukları sert kapatmasına yol açıyor.
    # Her iki durumda da node'ların destroy_node()'u çalışmıyor, dolayısıyla
    # VideoWriter.release() çağrılmıyor ve mp4'lerin moov atom'u yazılmıyor —
    # teslim dosyaları oynatılamaz hâlde kalıyor.
    log "kapatiliyor — node'lara dogrudan SIGINT (mp4 kayitlari kapansin diye)"
    pkill -INT -f "$WS/install/idaws_nodes/lib/idaws_nodes/" 2>/dev/null || true

    # Node'lar kayıtlarını kapatana kadar bekle (azami 15 sn)
    for _ in $(seq 1 75); do
        pgrep -f "$WS/install/idaws_nodes/lib/idaws_nodes/" >/dev/null 2>&1 || break
        sleep 0.2
    done
    log "node'lar kapandi, kalan bilesenler durduruluyor"

    for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done

    # Kayıt dosyalarının kapanmasına süre tanı (compose stop_grace_period 30s)
    local waited=0
    while [ "$waited" -lt 100 ]; do
        local alive=0
        for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
        [ "$alive" -eq 0 ] && break
        sleep 0.2
        waited=$((waited + 1))
    done

    # Hâlâ duran varsa kademeli olarak sertleş
    for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    sleep 2
    for pid in "${PIDS[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
    log "kapandi"
}
trap cleanup EXIT INT TERM

# Bir koşul sağlanana kadar bekler; süre dolarsa hata verir.
wait_for() {
    local desc="$1" timeout="$2"; shift 2
    log "bekleniyor: $desc (azami ${timeout}s)"
    for _ in $(seq 1 "$((timeout * 2))"); do
        if "$@" >/dev/null 2>&1; then log "hazir: $desc"; return 0; fi
        sleep 0.5
    done
    log "HATA: $desc zamaninda hazir olmadi"
    return 1
}

udp_port_open()  { ss -lun 2>/dev/null | grep -q ":$1 "; }
tcp_port_open()  { ss -ltn 2>/dev/null | grep -q ":$1 "; }

# ───────────────────────── 1. Gazebo ─────────────────────────
if [ -n "${DISPLAY:-}" ]; then
    GZ_ARGS=(-v3 -r "$WORLD")            # GUI'li
    log "Gazebo baslatiliyor (GUI, world=$WORLD)"
else
    GZ_ARGS=(-v3 -s -r "$WORLD")         # headless (sadece sunucu)
    log "Gazebo baslatiliyor (headless, world=$WORLD)"
fi
gz sim "${GZ_ARGS[@]}" > "$LOG_DIR/gazebo.log" 2>&1 &
track $! "Gazebo"

# ArduPilotPlugin hazır olunca udp:9002'yi dinlemeye başlar — SITL'in
# bağlanabilmesi için bu şart.
wait_for "Gazebo ArduPilotPlugin (udp:9002)" 120 udp_port_open 9002

# ───────────────────────── 2. ArduPilot SITL ─────────────────────────
# sim_vehicle.py yerine ikili doğrudan çalıştırılıyor: konteynerde MAVProxy'yi
# ayrı yönetmek gerekiyor (TTY olmadan sim_vehicle.py MAVProxy'yi ayakta
# tutamıyor) ve ArduPilot kaynak ağacına ihtiyaç kalmıyor.
log "ArduPilot SITL (Rover) baslatiliyor"
cd "$RUN_DIR"
ardurover \
    --model JSON \
    --speedup "${IDAWS_SPEEDUP:-1}" \
    --slave 0 \
    --defaults "/opt/ardupilot/default_params/rover.parm,$PARM" \
    --sim-address=127.0.0.1 \
    -I0 \
    --home "$HOME_POS" \
    > "$LOG_DIR/sitl.log" 2>&1 &
track $! "ArduPilot SITL"

wait_for "SITL MAVLink (tcp:5760)" 90 tcp_port_open 5760

# ───────────────────────── 3. MAVProxy ─────────────────────────
# ROS tarafı udp:14550'yi bekliyor (sim_params.yaml).
log "MAVProxy baslatiliyor (tcp:5760 → udp:14550)"
mavproxy.py \
    --master tcp:127.0.0.1:5760 \
    --out udp:127.0.0.1:14550 \
    --daemon \
    --state-basedir="$RUN_DIR/mavproxy" \
    > "$LOG_DIR/mavproxy.log" 2>&1 &
# DIKKAT: MAVProxy --daemon ile FORK EDIYOR. Baslattigimiz PID hemen olur,
# gercek surec arka planda kalir. $! izlenirse "bilesen oldu" sanilip saglikli
# yigin gereksiz yere kapatiliyor. Gercek PID'i pgrep ile buluyoruz.
sleep 6
MAVPROXY_PID="$(pgrep -f 'mavproxy\.py .*--master tcp:127.0.0.1:5760' | head -1 || true)"
if [ -n "$MAVPROXY_PID" ]; then
    track "$MAVPROXY_PID" "MAVProxy"
    log "MAVProxy calisiyor (pid $MAVPROXY_PID)"
else
    log "UYARI: MAVProxy sureci bulunamadi — telemetri koprusu calismayabilir"
fi

# ───────────────────────── 4. ROS 2 node'ları ─────────────────────────
log "ROS 2 node'lari baslatiliyor (gz koprulari + IDAWS otonomi)"
# Boru hattı KULLANILMIYOR: `... | tee` deseninde $! tee'nin PID'ini verir ve
# kapanışta ros2 launch'a SIGTERM gitmez — mp4 kayıtları bozuk kalırdı.
# Süreç ikamesi ile hem log dosyaya yazılır hem $! doğru PID'i tutar.
ros2 launch idaws_bringup sim_launch.py > >(tee "$LOG_DIR/ros.log") 2>&1 &
track $! "ROS 2 node'lari"

log ""
log "════════════════════════════════════════════════════"
log " IDAWS simulasyonu calisiyor"
log "   Web paneli : http://localhost:5000"
log "   Log'lar    : $LOG_DIR/{gazebo,sitl,mavproxy,ros}.log"
log "   Durdurmak  : Ctrl-C  (veya docker compose down)"
log "════════════════════════════════════════════════════"
log ""

# Herhangi bir bileşen ölürse konteyner de kapansın ki sessizce yarım çalışan
# bir yığınla uğraşılmasın. `wait -n` KULLANILMIYOR: MAVProxy'nin gerçek süreci
# fork sonrası bizim çocuğumuz değil, ayrıca `wait -n` fork'ta hemen dönüp
# sağlıklı yığını kapatıyordu. Bunun yerine izlenen PID'ler yoklanıyor.
while true; do
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
            log "'${NAMES[$i]}' sonlandi (pid ${PIDS[$i]}) — yigin kapatiliyor"
            exit 1
        fi
    done
    sleep 2
done
