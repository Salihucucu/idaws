#!/usr/bin/env bash
# ──────────────────────────────────────────────────────
# IDAWS Hotspot Durdurma Scripti
# ──────────────────────────────────────────────────────

set -euo pipefail

echo "[IDAWS] Hotspot kapatılıyor..."
nmcli connection down "idaws-hotspot" 2>/dev/null || true
nmcli connection delete "idaws-hotspot" 2>/dev/null || true

echo "[IDAWS] Hotspot kapatıldı."
