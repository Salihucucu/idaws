#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# IDAWS — GitHub yayin hazirligi
#
#   ./setup-github.sh <github-kullanici-adi> [repo-adi]
#
# Yaptiklari:
#   1. docker-compose.yml, Dockerfile ve DOCKER.md icindeki OWNER yer
#      tutucularini gercek kullanici adinla degistirir
#   2. git remote'u ekler (varsa gunceller)
#   3. Ne yapman gerektigini adim adim yazar
#
# Repoyu GitHub'da olusturmak ve push etmek senin kimliginle olmali;
# bu betik onun disindaki her seyi hazirlar.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

USER_NAME="${1:-}"
REPO_NAME="${2:-idaws}"

if [ -z "$USER_NAME" ]; then
    echo "Kullanim: $0 <github-kullanici-adi> [repo-adi]"
    echo "Ornek   : $0 salihucucu idaws"
    exit 1
fi

IMAGE="ghcr.io/${USER_NAME,,}/${REPO_NAME,,}"   # ghcr.io kucuk harf ister

echo "Kullanici : $USER_NAME"
echo "Repo      : $REPO_NAME"
echo "Imaj      : $IMAGE:latest"
echo

# ─── 1. Yer tutucularin degistirilmesi ───
changed=0
for f in docker/docker-compose.yml docker/Dockerfile DOCKER.md README.md; do
    [ -f "$f" ] || continue
    if grep -q "OWNER" "$f" 2>/dev/null; then
        sed -i "s|ghcr.io/OWNER/idaws|$IMAGE|g; s|github.com/OWNER/idaws|github.com/$USER_NAME/$REPO_NAME|g" "$f"
        echo "  guncellendi: $f"
        changed=$((changed + 1))
    fi
done
[ "$changed" -eq 0 ] && echo "  (yer tutucu bulunamadi — muhtemelen zaten guncellenmis)"

# ─── 2. Remote ───
REMOTE_URL="https://github.com/${USER_NAME}/${REPO_NAME}.git"
if git remote get-url origin >/dev/null 2>&1; then
    git remote set-url origin "$REMOTE_URL"
    echo "  remote guncellendi: $REMOTE_URL"
else
    git remote add origin "$REMOTE_URL"
    echo "  remote eklendi: $REMOTE_URL"
fi

# ─── 3. Kalan adimlar ───
cat <<EOM

────────────────────────────────────────────────────────────
Sirada senin yapman gerekenler:

1) GitHub'da bos bir repo olustur (README/gitignore EKLEME):
     https://github.com/new   →  isim: $REPO_NAME

2) Degisiklikleri commit'le ve push et:
     git add -A
     git commit -m "GitHub yayin ayarlari"
     git branch -M main
     git push -u origin main

   Kimlik dogrulama sorarsa: sifre yerine Personal Access Token kullan
   (https://github.com/settings/tokens — 'repo' ve 'write:packages' yetkisi)

3) Imaj yayini icin Actions iznini ac:
     Settings → Actions → General → Workflow permissions
        → "Read and write permissions" → Save

   Push'tan sonra Actions sekmesinde imaj derlenmeye baslar (~40 dk).
   Bittiginde arkadaslarinin yapmasi gereken tek sey:

     git clone $REMOTE_URL && cd $REPO_NAME && ./run-docker.sh

4) YOLO agirliklarini (*.pt) ayrica paylas — .gitignore disinda tutuluyor.
   Arkadaslarin dosyayi indirip params.yaml'daki model_path'i ayarlamali.
────────────────────────────────────────────────────────────
EOM
