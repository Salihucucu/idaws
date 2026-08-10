#!/usr/bin/env bash
# IDAWS konteyner giriş noktası — ortamı hazırlar, sonra verilen komutu çalıştırır.
set -e

source /opt/ros/humble/setup.bash
if [ -f /idaws/install/setup.bash ]; then
    source /idaws/install/setup.bash
fi

# Qt/Gazebo bu değişkeni bekliyor; yoksa "XDG_RUNTIME_DIR not set" uyarısı verip
# geçici dizine düşüyor.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-idaws}"

# ── gz-transport keşfi ──
# gz-transport peer'ları multicast (224.0.0.7) ile bulur. Konteynerin loopback
# arayüzünde MULTICAST bayrağı kapalı olabilir; açabiliyorsak açıp keşfi
# tamamen konteyner içinde tutarız (host ağına sızmaz, en güvenli seçenek).
# Yetki yoksa (NET_ADMIN verilmemişse) keşif eth0 üzerinden yürür, o da çalışır.
if ip link set lo multicast on 2>/dev/null; then
    export GZ_IP=127.0.0.1
else
    echo "[entrypoint] lo multicast ayarlanamadi (NET_ADMIN yok) — kesif eth0 uzerinden."
fi

# ── Görüntü ──
if [ -n "$DISPLAY" ] && [ -S /tmp/.X11-unix/X"${DISPLAY#*:}" ] 2>/dev/null; then
    echo "[entrypoint] X11 baglantisi: DISPLAY=$DISPLAY"
elif [ -n "$DISPLAY" ]; then
    echo "[entrypoint] DISPLAY=$DISPLAY ayarli ama X soketi gorunmuyor;"
    echo "             host'ta 'xhost +local:docker' calistirmayi unutma."
else
    echo "[entrypoint] DISPLAY yok — headless mod (Gazebo GUI'siz calisacak)."
fi

mkdir -p "${IDAWS_RUN_DIR:-/idaws/run}" /idaws/logs

# ── Host kullanıcısı olarak çalış ──
# Konteyner root olarak çalışırsa recordings/ altındaki şartname teslimleri
# host'ta root'a ait olur ve kullanıcı silemez/açamaz. HOST_UID verilmişse
# aynı kimlikte bir kullanıcı oluşturup ona geçiyoruz; bu ayrıca X11 yetkisini
# de kolaylaştırıyor (cookie zaten o kullanıcının).
if [ -n "${HOST_UID:-}" ] && [ "${HOST_UID}" != "0" ] && [ "$(id -u)" = "0" ]; then
    HOST_GID="${HOST_GID:-$HOST_UID}"
    if ! getent group "$HOST_GID" >/dev/null 2>&1; then
        groupadd -g "$HOST_GID" idaws 2>/dev/null || true
    fi
    if ! getent passwd "$HOST_UID" >/dev/null 2>&1; then
        useradd -u "$HOST_UID" -g "$HOST_GID" -M -d /idaws/home -s /bin/bash idaws 2>/dev/null || true
    fi
    RUN_USER="$(getent passwd "$HOST_UID" | cut -d: -f1)"

    # Gazebo ~/.gz altına (ogre2 önbelleği, sim ayarları) ve ROS ~/.ros altına
    # yazar; ev dizini yazılabilir olmazsa ikisi de hata verir.
    mkdir -p /idaws/home "$XDG_RUNTIME_DIR"
    chown -R "$HOST_UID:$HOST_GID" /idaws/logs /idaws/run /idaws/home "$XDG_RUNTIME_DIR" 2>/dev/null || true
    chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true
    export HOME=/idaws/home

    echo "[entrypoint] host kullanicisi olarak calisiliyor: $RUN_USER ($HOST_UID:$HOST_GID)"
    exec setpriv --reuid "$HOST_UID" --regid "$HOST_GID" --init-groups "$@"
fi

# root olarak çalışıyorsak da ev dizini ve runtime dizini hazır olsun
export HOME="${HOME:-/root}"
mkdir -p "$XDG_RUNTIME_DIR" 2>/dev/null || true

exec "$@"
