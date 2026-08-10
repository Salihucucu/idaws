#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# IDAWS imaj duman testi — konteyneri headless başlatıp yığının gerçekten
# ayağa kalktığını doğrular. Yeni imaj yayınlamadan önce çalıştırın.
#
#   ./docker/smoke-test.sh [imaj-adi]     (varsayilan: idaws:local)
# ─────────────────────────────────────────────────────────────────────────────
set -uo pipefail

IMAGE="${1:-idaws:local}"
NAME="idaws-smoke-$$"
TIMEOUT="${SMOKE_TIMEOUT:-180}"
FAILED=0

pass() { echo -e "  \033[32m✓\033[0m $*"; }
fail() { echo -e "  \033[31m✗\033[0m $*"; FAILED=1; }
info() { echo -e "\033[36m$*\033[0m"; }

cleanup() {
    echo
    info "konteyner kapatiliyor"
    docker stop -t 30 "$NAME" >/dev/null 2>&1 || true
    docker rm -f "$NAME"      >/dev/null 2>&1 || true
}
trap cleanup EXIT

info "1/5  Konteyner baslatiliyor (headless, yazilim render)"
# GPU BILEREK gecirilmiyor: headless'ta Ogre2 EGL cihazlarini tarayip ilk
# bulduguna baglaniyor ve calismayan bir karta denk gelince segfault ediyor
# (bizde /dev/dri/card2). Yazilim render yavas ama her makinede ayni sekilde
# calisiyor — duman testi icin dogru takas. GUI kullanimi X11 uzerinden
# gectigi icin bu yoldan etkilenmiyor.
docker run -d --name "$NAME" \
    --cap-add NET_ADMIN \
    -p 15000:5000 \
    -e DISPLAY= \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e GALLIUM_DRIVER=llvmpipe \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    "$IMAGE" >/dev/null || { fail "konteyner baslamadi"; exit 1; }

info "2/5  Yigin hazir olmasi bekleniyor (azami ${TIMEOUT}s)"
ready=0
for _ in $(seq 1 "$TIMEOUT"); do
    if curl -sf --max-time 2 http://127.0.0.1:15000/api/telemetry >/dev/null 2>&1; then
        ready=1; break
    fi
    if ! docker ps -q -f "name=$NAME" | grep -q .; then
        fail "konteyner erken kapandi"
        docker logs "$NAME" 2>&1 | tail -30
        exit 1
    fi
    sleep 1
done
[ "$ready" -eq 1 ] && pass "web paneli yanit veriyor" || { fail "web paneli acilmadi"; docker logs "$NAME" 2>&1 | tail -30; exit 1; }

# Sensör verisinin akması için biraz daha bekle
sleep 20

info "3/5  Telemetri"
TELEM=$(curl -sf --max-time 5 http://127.0.0.1:15000/api/telemetry)
echo "$TELEM" | grep -q '"heartbeat"' && pass "telemetri ucu calisiyor" || fail "telemetri yok"
echo "$TELEM" | grep -q 'armed=' && pass "MAVLink heartbeat aliniyor (SITL bagli)" \
    || fail "MAVLink heartbeat yok — SITL/MAVProxy zinciri kopuk"
echo "$TELEM" | grep -q '"camera_ok":true' && pass "kamera akiyor (gz→ROS kopru)" \
    || fail "kamera akmiyor"

info "4/5  LiDAR ve algi"
LIDAR=$(curl -sf --max-time 5 http://127.0.0.1:15000/api/lidar)
echo "$LIDAR" | grep -q '"ok":true' && pass "/scan koprusu calisiyor" || fail "/scan verisi yok"
PTS=$(echo "$LIDAR" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("point_count",0))' 2>/dev/null || echo 0)
[ "$PTS" -gt 0 ] && pass "LiDAR $PTS gecerli isin goruyor (dubalar menzilde)" \
    || fail "LiDAR hicbir sey gormuyor — dunya modelleri yuklenmemis olabilir"

MAP=$(curl -sf --max-time 5 http://127.0.0.1:15000/api/costmap)
echo "$MAP" | grep -q '"ok":true' && pass "cost map uretiliyor" || fail "cost map yok"

info "5/5  Temiz kapanis ve kayitlar"
docker stop -t 30 "$NAME" >/dev/null 2>&1
sleep 2
MP4=$(docker cp "$NAME:/idaws/logs/." /tmp/idaws-smoke-logs 2>/dev/null && \
      find /tmp/idaws-smoke-logs -name "*.mp4" 2>/dev/null | wc -l)
CSV=$(find /tmp/idaws-smoke-logs -name "*.csv" 2>/dev/null | wc -l)
[ "${MP4:-0}" -gt 0 ] && pass "$MP4 adet mp4 kaydi uretildi" || fail "mp4 kaydi yok"
[ "${CSV:-0}" -gt 0 ] && pass "telemetri CSV'si uretildi" || fail "CSV yok"
if [ "${MP4:-0}" -gt 0 ]; then
    BAD=0
    while read -r f; do
        python3 -c "
import cv2,sys
c=cv2.VideoCapture('$f')
sys.exit(0 if c.read()[0] else 1)" 2>/dev/null || BAD=1
    done < <(find /tmp/idaws-smoke-logs -name "*.mp4")
    [ "$BAD" -eq 0 ] && pass "mp4 kayitlari oynatilabilir (temiz kapanis)" \
        || fail "mp4 bozuk — kapanista SIGTERM node'lara ulasmamis"
fi
rm -rf /tmp/idaws-smoke-logs

echo
if [ "$FAILED" -eq 0 ]; then
    echo -e "\033[32mTUM TESTLER GECTI — imaj yayina hazir.\033[0m"
else
    echo -e "\033[31mBAZI TESTLER BASARISIZ.\033[0m  Log: docker logs $NAME"
    exit 1
fi
