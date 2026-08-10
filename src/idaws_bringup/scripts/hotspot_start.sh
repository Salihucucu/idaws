#!/usr/bin/env bash
# ──────────────────────────────────────────────────────
# IDAWS Hotspot Başlatma Scripti
# Jetson Wi-Fi donanımı üzerinden Access Point oluşturur.
# Yarışma anında kapalı tutulacaktır.
# ──────────────────────────────────────────────────────

set -euo pipefail

SSID="IDAWS-USV"
PASSWORD="idaws2026"
WIFI_IFACE="wlan0"
BAND="bg"       # 2.4 GHz — daha geniş kapsama
CHANNEL="6"

echo "[IDAWS] Mevcut Wi-Fi bağlantıları kapatılıyor..."
nmcli device disconnect "$WIFI_IFACE" 2>/dev/null || true

echo "[IDAWS] Hotspot oluşturuluyor: SSID=$SSID"
nmcli connection add \
    type wifi \
    ifname "$WIFI_IFACE" \
    con-name "idaws-hotspot" \
    autoconnect no \
    ssid "$SSID" \
    -- \
    wifi.mode ap \
    wifi.band "$BAND" \
    wifi.channel "$CHANNEL" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD" \
    ipv4.method shared \
    ipv4.addresses 10.42.0.1/24 \
    2>/dev/null || echo "[IDAWS] Bağlantı profili zaten mevcut, yeniden kullanılıyor."

echo "[IDAWS] Hotspot aktifleştiriliyor..."
nmcli connection up "idaws-hotspot"

echo ""
echo "════════════════════════════════════════"
echo "  IDAWS Hotspot Aktif"
echo "  SSID     : $SSID"
echo "  Şifre    : $PASSWORD"
echo "  IP       : 10.42.0.1"
echo "  Web UI   : http://10.42.0.1:5000"
echo "════════════════════════════════════════"
