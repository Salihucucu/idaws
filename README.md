# IDAWS — İnsansız Deniz Aracı Otonomi Yazılımı

TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması için ROS 2 tabanlı otonomi yığını.
Gazebo + ArduPilot SITL ile tam simülasyon, gerçek araçta Pixhawk + Jetson üzerinde
aynı kodla çalışır.

## Hızlı başlangıç

```bash
./run-docker.sh
```

Gazebo penceresi açılır, web paneli <http://localhost:5000> adresinde çalışır.
Ayrıntılar ve sorun giderme: **[DOCKER.md](DOCKER.md)**

---

## Sistem mimarisi

```
                    ┌──────────────────────┐
   Pixhawk ◄────────┤ pymavlink_controller │  telemetri, SCR_USER parametreleri,
   (MAVLink)        └──────────┬───────────┘  RC override, arm/mod komutları
                               │
   Kamera ──► yolo_vision ─────┤ vision/buoys, vision/image_annotated
                               │
   LiDAR  ──► lidar_logger ────┤ perception/lidar_clusters, perception/costmap
                               │
                    ┌──────────┴───────────┐
                    │ collision_avoidance  │  VFH + hıza bağlı güvenlik yarıçapı
                    └──────────┬───────────┘
                               │ cmd/velocity
                    ┌──────────┴───────────┐
                    │  mission_manager     │  faz yönetimi (Parkur 1/2/3)
                    └──────────────────────┘
                               │
                    ┌──────────┴───────────┐
                    │  web_ui / logger     │  kontrol paneli + şartname kayıtları
                    └──────────────────────┘
```

### Node'lar

| Node | Görevi |
|---|---|
| `pymavlink_controller_node` | Otopilot bağlantısı: telemetri yayını, `SCR_USER1/2` okuma-yazma, arm/mod/hız komutları |
| `yolo_vision_node` | YOLOv8 ile duba tespiti, çizimli görüntü yayını, 1 Hz mp4 kaydı |
| `lidar_logger_node` | LaserScan kümeleme, lokal cost map üretimi, 1 Hz mp4 kaydı |
| `collision_avoidance_node` | VFH engelden kaçınma, hıza bağlı güvenlik yarıçapı, hız governor'ı |
| `mission_manager_node` | Parkur 1 → 2 → 3 faz yönetimi |
| `mission_logger_node` | Telemetri CSV kaydı (≥1 Hz) |
| `web_ui_node` | Kontrol paneli: kamera, LiDAR, cost map, manuel kumanda, parkur seçimi |

---

## Güvenlik mantığı

**Otonomi yalnız görev fazındayken motor komutu üretir.** IDLE'da `cmd/velocity`
topic'i tamamen sessizdir; böylece araç ARM edildiğinde kendiliğinden hareket
etmez ve manuel kumanda otonomiyle çakışmaz.

**Güvenlik yarıçapı hıza bağlıdır:**

```
durma_mesafesi = v · t_tepki + v² / (2 · a_yavaşlama)
güvenlik       = taban + durma_mesafesi     (max_safety_radius ile sınırlı)
```

Sabit yarıçap 5 m/s'de engeli ancak 0.4 sn kala fark ettiriyordu; bu formülle
tepki payı ~2 sn'de sabit kalır.

**Hız sensör menziliyle sınırlıdır.** 12 m'lik LiDAR ile güvenle durulabilecek
azami hız 3.62 m/s'dir; `collision_avoidance_node` gazı kapalı çevrimle bu
değerde tutar. Daha hızlı gitmek için önce menzil artmalı.

---

## Parkur seçimi

Görev, otopilot üzerindeki `SCR_USER1` parametresiyle seçilir. Web arayüzündeki
butonlar da bu parametreyi yazar — test yolu sahadaki gerçek yolun aynısıdır.

| `SCR_USER1` | Faz |
|---|---|
| 0 | IDLE |
| 5 | GERÇEK görev (Parkur 1'den otomatik ilerler) |
| 10 / 15 / 20 | Test: Parkur 1 / 2 / 3 |

`SCR_USER2` Parkur 3 hedef renk kodunu taşır.

---

## Şartname teslimleri

| Dosya | İçerik | Üreten |
|---|---|---|
| `detect_*.mp4` | Dosya 1 — işlenmiş kamera, ≥1 Hz, zaman etiketli, tespit çerçeveli | `yolo_vision_node` |
| `lidar_*.mp4` | Diğer otonomi sensörü — kümeleme görünür | `lidar_logger_node` |
| `costmap_*.mp4` | Dosya 3 — lokal engel/cost haritası | `lidar_logger_node` |
| `mission_*.csv` | Dosya 2 — telemetri, başlık satırlı | `mission_logger_node` |

Docker'da bu dosyalar host'taki `recordings/` klasörüne yazılır.

### Bilinen eksikler

- `mission_*.csv` şartnamenin istediği **hız set point** ve **yön set point**
  kolonlarını içermiyor; şu an RC seviyesindeki `cmd_linear_x` / `cmd_angular_z`
  yazılıyor.
- Yarışma turunda yer istasyonuna görüntü aktarımı **yasak** (şartname 4.1).
  Web panelindeki kamera akışı geliştirme ve SITL testi içindir; yarışta
  kapatılması gerekir.

---

## Gerçek araçta çalıştırma

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch idaws_bringup idaws_launch.py
```

Parametreler `src/idaws_bringup/config/params.yaml` içinde: seri portlar, kamera
kaynağı, LiDAR ve güvenlik yarıçapı ayarları.

Simülasyon parametreleri ayrı: `src/idaws_bringup/sim/config/sim_params.yaml`

## Docker'sız simülasyon

Gazebo, ArduPilot ve `asv_wave_sim` host'ta kuruluysa:

```bash
bash src/idaws_bringup/sim/scripts/start_sim.sh
```

> `~/.bashrc` içindeki `GZ_PARTITION` **sabit bir değer** olmalıdır. Zaman
> damgalı üretilirse her terminal ayrı partition'a düşer ve `ros_gz_bridge`
> Gazebo'nun topic'lerini göremez. Ayrıca gz-transport keşfi multicast'e
> dayandığından loopback'te MULTICAST bayrağı açık olmalıdır
> (`sudo ip link set lo multicast on`).
