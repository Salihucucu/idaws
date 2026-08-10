# IDAWS — Docker ile Kurulum

TEKNOFEST 2026 İnsansız Deniz Aracı otonomi yığınının tamamı tek bir imajda:
ROS 2 Humble, Gazebo Harmonic, ArduPilot SITL (Rover), dalga simülasyonu ve
IDAWS otonomi node'ları. Hiçbir şeyi elle kurmanıza gerek yok.

---

## Gereksinimler

| Gereksinim | Neden |
|---|---|
| Linux (Ubuntu 22.04+ önerilir) | Gazebo penceresi host'un X sunucusunu kullanıyor |
| Docker Engine 24+ ve `docker compose` | — |
| ~8 GB boş disk | İmaj ~4.5 GB |
| GPU (opsiyonel ama önerilir) | Kamera sensörü render'ı; yoksa FPS çok düşer |

Docker kurulu değilse:

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"   # sonra oturumu kapatıp aç
```

---

## Çalıştırma

```bash
git clone <repo-adresi> idaws
cd idaws
./run-docker.sh
```

Bu kadar. Betik X11 iznini verir, GPU'yu tespit eder, imajı çeker ve başlatır.
İlk çalıştırmada imaj indirilirken birkaç dakika beklemeniz gerekir.

Açılacak olanlar:

- **Gazebo penceresi** — Ezel İDA'sı, dalgalı deniz, spawn çevresinde 4 test dubası
- **Web kontrol paneli** — <http://localhost:5000>

### Diğer komutlar

```bash
./run-docker.sh stop     # durdur
./run-docker.sh logs     # log'ları izle
./run-docker.sh shell    # çalışan konteynerde kabuk aç
./run-docker.sh build    # hazır imaj yerine yerel kaynaktan derle (30-45 dk)
```

---

## Web panelinde neler var

| Panel | İçerik |
|---|---|
| **Kamera** | Gazebo kamerasının canlı MJPEG akışı. YOLO açıksa çizimli görüntü. |
| **LiDAR + Kümeleme** | Kuş bakışı tarama, renklendirilmiş engel kümeleri, hıza bağlı güvenlik/tehlike çemberleri |
| **Lokal Cost Map** | Araç merkezli 24×24 m şişirilmiş engel haritası |
| **Manuel Kumanda** | İleri/geri/sağ/sol + gaz. Klavye: `W/A/S/D`, boşluk = acil dur |
| **Otopilot** | ARM / DISARM, MANUAL / GUIDED / HOLD mod seçimi |
| **Parkur Seçimi** | Otopilottaki `SCR_USER1` parametresini yazar (sahadaki gerçek akışın aynısı) |

### Hızlı test turu

1. **MANUAL** moda al, **ARM** et
2. Manuel kumanda ile dubaların arasına sür — LiDAR panelinde kümeler renklenir
3. **Parkur 1** butonuna bas — otonomi devralır, hız governor'ı ~3.6 m/s'de tutar
4. **0 · IDLE** ile otonomiyi durdur

---

## Parkur sistemi nasıl çalışıyor

Görev seçimi **otopilot üzerindeki parametrelerle** yapılır; web arayüzü de
bu parametreyi yazar, yani test yolu sahadaki gerçek yolun birebir aynısıdır.

| `SCR_USER1` | Anlamı |
|---|---|
| 0 | IDLE — motorlar durur |
| 5 | GERÇEK görev — Parkur 1'den başlayıp otomatik ilerler |
| 10 | Test: Parkur 1'e zorla |
| 15 | Test: Parkur 2'ye zorla |
| 20 | Parkur 3 (Kamikaze) |

`SCR_USER2` Parkur 3 için hedef renk kodunu taşır.

Jetson tarafında `pymavlink_controller_node` bu parametreleri saniyede bir okur,
değiştiklerinde `mission_manager_node` faz geçişini uygular.

---

## Şartname teslimleri nerede

Kayıtlar host'taki `recordings/` klasörüne yazılır (konteyner içinde `/idaws/logs`):

| Dosya | Şartname karşılığı |
|---|---|
| `detect_*.mp4` | Dosya 1 — işlenmiş kamera verisi, ≥1 Hz, zaman etiketli, tespit çerçeveli |
| `lidar_*.mp4` | Diğer otonomi sensörü veri seti — kümeleme görünür |
| `costmap_*.mp4` | Dosya 3 — lokal engel/cost haritası, ≥1 Hz |
| `mission_*.csv` | Dosya 2 — telemetri, ≥1 Hz, başlık satırlı |

> **Not:** Konteyneri her zaman `./run-docker.sh stop` veya `Ctrl-C` ile kapatın.
> mp4 dosyalarının indeksi (moov atom) ancak temiz kapanışta yazılır; `docker kill`
> ile öldürülen konteynerde kayıtlar oynatılamaz hâle gelir.

---

## Sorun giderme

**Gazebo penceresi açılmıyor**

```bash
xhost +local:docker
echo $DISPLAY          # boşsa GUI mümkün değil, headless çalışır
```

**Kamera görüntüsü çok yavaş / siyah**

GPU geçişi çalışmıyordur. `ls /dev/dri` çıktısı boşsa host'ta GPU sürücüsü yok
demektir. NVIDIA kartlarda ayrıca container toolkit gerekir:

```bash
sudo apt install nvidia-container-toolkit && sudo systemctl restart docker
```

**LiDAR paneli boş**

Araç dubalardan uzaklaşmıştır. LiDAR menzili 12 m; manuel kumanda ile dubalara
yaklaşın veya konteyneri yeniden başlatın (araç başlangıç noktasına döner).

**Web paneli açılmıyor**

```bash
docker ps                        # idaws-sim ayakta mı
./run-docker.sh logs             # hangi bileşen patlamış
```

**"port 5000 already in use"**

Host'ta başka bir şey 5000'i kullanıyordur. `docker/docker-compose.yml` içinde
`"5000:5000"` yerine `"5001:5000"` yazıp `localhost:5001`'e bakın.

---

## Mimari

Her şey tek konteynerde, sırayla ve hazır olduğu doğrulanarak başlar:

```
1. Gazebo          deniz.sdf dünyası, ArduPilotPlugin udp:9002'yi dinler
2. ArduPilot SITL  ardurover, JSON modeliyle Gazebo'ya bağlanır
3. MAVProxy        tcp:5760 → udp:14550
4. ROS 2           gz köprüleri (/lidar→/scan, /camera/image_raw) + 7 IDAWS node'u
```

Tek konteyner tercih edildi çünkü gz-transport ve DDS keşfi konteynerler arasında
multicast gerektiriyor ve bu, farklı Docker ağ kurulumlarında kırılgan.

### Bağımlılıklar (hepsi commit'e sabitli)

| Kaynak | Commit | Ne sağlıyor |
|---|---|---|
| [ArduPilot](https://github.com/ArduPilot/ardupilot) | `0577bad42e` | `ardurover` SITL ikilisi |
| [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) | `685e3e3` | `ArduPilotPlugin` |
| [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim) | `ca8629d` | Dalga + hidrodinamik, `waves` ve `wam-v` modelleri |

Ezel İDA modeli ve `deniz.sdf` dünyası **bu repoda** (`src/idaws_bringup/sim/`)
versiyonludur — takımın kendi eseri, hiçbir upstream depoda yok.

---

## Geliştirme

Kaynak değiştirip denemek için:

```bash
./run-docker.sh build            # yerel Dockerfile'dan derler
```

Bu profilde `src/` konteynere canlı bağlanır. Kod değiştirdikten sonra:

```bash
./run-docker.sh shell
colcon build --symlink-install && exit
./run-docker.sh stop && ./run-docker.sh build
```

### Yeni imaj yayınlama

```bash
docker build -f docker/Dockerfile -t ghcr.io/<kullanici>/idaws:latest .
echo "$GITHUB_TOKEN" | docker login ghcr.io -u <kullanici> --password-stdin
docker push ghcr.io/<kullanici>/idaws:latest
```

`docker/docker-compose.yml` içindeki `ghcr.io/salihucucu/idaws:latest` satırını kendi
adresinizle değiştirmeyi unutmayın.
