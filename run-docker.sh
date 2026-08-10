#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# IDAWS — tak çalıştır başlatıcı
#
#   ./run-docker.sh            simülasyonu başlat (hazır imajı çeker)
#   ./run-docker.sh build      yerel kaynaktan derle ve başlat
#   ./run-docker.sh shell      çalışan konteynerde kabuk aç
#   ./run-docker.sh stop       durdur
#   ./run-docker.sh logs       log'ları izle
#
# Tek yaptığı şey: X11 iznini vermek, GPU'yu tespit etmek ve compose'u çağırmak.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
COMPOSE=(docker compose -f docker/docker-compose.yml)
CMD="${1:-up}"

red()  { echo -e "\033[31m$*\033[0m"; }
grn()  { echo -e "\033[32m$*\033[0m"; }
ylw()  { echo -e "\033[33m$*\033[0m"; }

# ─── Ön kontroller ───
command -v docker >/dev/null || { red "Docker kurulu degil."; exit 1; }
docker compose version >/dev/null 2>&1 || { red "'docker compose' eklentisi yok."; exit 1; }

if [ "$(uname -s)" != "Linux" ]; then
    red "Bu betik X11 paylasimi kullaniyor, sadece Linux'ta calisir."
    exit 1
fi

case "$CMD" in
  stop|down)
      "${COMPOSE[@]}" down
      grn "IDAWS durduruldu."
      exit 0 ;;
  logs)
      exec "${COMPOSE[@]}" logs -f ;;
  shell)
      exec docker exec -it idaws-sim bash ;;
esac

# ─── X11 izni ───
if [ -z "${DISPLAY:-}" ]; then
    ylw "DISPLAY bos — headless calisacak (Gazebo penceresi olmayacak)."
    ylw "Web paneli yine de http://localhost:5000 adresinde acilir."
else
    if command -v xhost >/dev/null; then
        xhost +local:docker >/dev/null 2>&1 \
            && grn "X11 izni verildi (xhost +local:docker)" \
            || ylw "xhost calistirilamadi; Gazebo penceresi acilmayabilir."
    else
        ylw "xhost bulunamadi (x11-xserver-utils paketi). GUI acilmayabilir."
    fi
fi

# ─── GPU tespiti ───
GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -q nvidia; then
    grn "NVIDIA GPU + container toolkit bulundu — donanim hizlandirma acik."
    export IDAWS_GPU_RUNTIME=nvidia
elif [ -d /dev/dri ]; then
    grn "Intel/AMD GPU (/dev/dri) bulundu — donanim hizlandirma acik."
else
    ylw "GPU bulunamadi — yazilim render kullanilacak, kamera FPS'i dusuk olur."
fi

# XAUTHORITY yoksa compose'un bind mount'u patlamasin
export XAUTHORITY="${XAUTHORITY:-/dev/null}"

# Konteyner bu kimlikle calisir; recordings/ altindaki teslim dosyalari
# host'ta senin kullanicina ait olur (root'a degil).
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

mkdir -p recordings

case "$CMD" in
  up)
      grn "IDAWS baslatiliyor (hazir imaj)..."
      "${COMPOSE[@]}" up --pull always idaws ;;
  build)
      grn "IDAWS yerel kaynaktan derleniyor (ilk sefer 30-45 dk surer)..."
      "${COMPOSE[@]}" --profile dev up --build idaws-dev ;;
  *)
      red "Bilinmeyen komut: $CMD"
      echo "Kullanim: $0 [up|build|shell|stop|logs]"
      exit 1 ;;
esac
