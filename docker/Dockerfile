# ─────────────────────────────────────────────────────────────────────────────
# IDAWS — TEKNOFEST 2026 İnsansız Deniz Aracı otonomi yığını
#
# Tek imajda: ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL (Rover) +
# ardupilot_gazebo eklentisi + asv_wave_sim dalga modeli + IDAWS node'ları.
#
# Bağımlılıklar commit'e sabitlenmiştir; upstream değişse bile aynı imaj çıkar.
# ─────────────────────────────────────────────────────────────────────────────

# ═════════════════════════ 1. AŞAMA: derleme ═════════════════════════
FROM ros:humble-ros-base-jammy AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Bağımlılık sürümleri — hepsi commit'e sabitli
ARG ARDUPILOT_REF=0577bad42e
ARG ARDUPILOT_GAZEBO_REF=685e3e3
ARG ASV_WAVE_SIM_REF=ca8629d

# ─────────── ArduPilot SITL (sadece Rover) ───────────
# EN BAŞTA derleniyor: en uzun süren adım bu (~15 dk) ve bağımlılıkları
# Gazebo tarafından tamamen ayrık. Böylece gz/gstreamer gibi paketler
# değiştiğinde ArduPilot katmanı önbellekte kalıyor.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake pkg-config rsync gawk ccache \
        python3-pip python3-dev python3-setuptools python3-wheel \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        "empy==3.3.4" pexpect future lxml "pymavlink>=2.4.37"

# Tam ağaç ~2 GB; sadece ardurover ikilisi ve varsayılan parametreler taşınacak.
WORKDIR /build
RUN git clone https://github.com/ArduPilot/ardupilot.git ardupilot \
    && cd ardupilot \
    && git checkout ${ARDUPILOT_REF} \
    && git submodule update --init --recursive --depth 1

WORKDIR /build/ardupilot
RUN ./waf configure --board sitl && ./waf rover

# ─── Gazebo Harmonic deposu (Ubuntu 22.04 için) ───
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ─── Gazebo tarafı derleme bağımlılıkları ───
#   rapidjson / opencv / gstreamer : ardupilot_gazebo CMakeLists'i üçünü de
#                                    koşulsuz arıyor (GstCameraPlugin kullanmasak da)
#   cgal / eigen                   : asv_wave_sim dalga çözücüsü
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libgz-cmake3-dev libgz-sim8-dev libgz-common5-dev libgz-rendering8-dev \
        libgz-math7-eigen3-dev libgz-msgs10-dev libgz-transport13-dev \
        libgz-plugin2-dev libsdformat13-dev \
        libcgal-dev libgmp-dev libmpfr-dev libfftw3-dev libopencv-dev libeigen3-dev \
        rapidjson-dev libboost-all-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# asv_wave_sim CMakeLists'i bu değişkene göre Garden/Harmonic kütüphanelerini
# seçiyor. Ayarlanmazsa Garden'ı (gz-msgs9/transport12/sim7) arar ve derleme
# "package not found" ile patlar.
ENV GZ_VERSION=harmonic

# ─────────── ardupilot_gazebo (ArduPilotPlugin) ───────────
WORKDIR /build
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && git checkout ${ARDUPILOT_GAZEBO_REF} \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j"$(nproc)"

# ─────────── asv_wave_sim (dalga + hidrodinamik, waves/wam-v modelleri) ───────────
# Ezel modelinin gövde dokusu wam-v'den, dünyanın su yüzeyi waves modelinden gelir.
WORKDIR /build/gz_ws/src
RUN git clone https://github.com/srmainwaring/asv_wave_sim.git \
    && cd asv_wave_sim && git checkout ${ASV_WAVE_SIM_REF}

WORKDIR /build/gz_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ═════════════════════════ 2. AŞAMA: çalışma imajı ═════════════════════════
FROM ros:humble-ros-base-jammy AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.title="IDAWS USV Autonomy Stack" \
      org.opencontainers.image.description="ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL, TEKNOFEST 2026 İDA" \
      org.opencontainers.image.source="https://github.com/Salihucucu/idaws"

# ─── Gazebo Harmonic deposu ───
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ─── Çalışma zamanı paketleri ───
#   gz-harmonic          : simülatör + GUI
#   ros-gzharmonic-*     : /lidar → /scan ve kamera köprüleri
#   iproute2             : entrypoint'te lo multicast ayarı (gz keşfi)
#   mesa-utils / libgl   : X11 üzerinden donanım hızlandırmalı render
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic-bridge \
        ros-humble-ros-gzharmonic-image \
        ros-humble-ros-gzharmonic-interfaces \
        ros-humble-ros2launch \
        ros-humble-ros2cli-common-extensions \
        python3-pip python3-flask python3-opencv python3-numpy python3-serial \
        python3-colcon-common-extensions \
        libfftw3-double3 \
        iproute2 procps net-tools iputils-ping less nano tini util-linux \
        libgl1-mesa-dri libgl1-mesa-glx mesa-utils libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir "pymavlink>=2.4.37" MAVProxy future

# ─── Derleme aşamasından çıktıları al ───
# ArduPilot: sadece Rover SITL ikilisi + varsayılan parametreler (~2 GB yerine ~30 MB)
COPY --from=builder /build/ardupilot/build/sitl/bin/ardurover /opt/ardupilot/bin/ardurover
COPY --from=builder /build/ardupilot/Tools/autotest/default_params/ /opt/ardupilot/default_params/

# ardupilot_gazebo eklentileri
COPY --from=builder /build/ardupilot_gazebo/build/*.so /opt/ardupilot_gazebo/
COPY --from=builder /build/ardupilot_gazebo/models/ /opt/ardupilot_gazebo/models/
COPY --from=builder /build/ardupilot_gazebo/worlds/ /opt/ardupilot_gazebo/worlds/

# asv_wave_sim: eklenti kütüphaneleri + waves/wam-v modelleri
COPY --from=builder /build/gz_ws/install/ /opt/gz_ws/install/
COPY --from=builder /build/gz_ws/src/asv_wave_sim/gz-waves-models/ /opt/gz_ws/gz-waves-models/

# ─── IDAWS çalışma alanı ───
WORKDIR /idaws
COPY src/ /idaws/src/
# --symlink-install KULLANILMIYOR: o modda install/ altında build/ dizinine
# işaret eden ~117 symlink oluşuyor (local_setup.bash dahil). build/ silinince
# çalışma alanı source edilemez hâle geliyordu. İmajda gerçek kopya isteriz.
RUN source /opt/ros/humble/setup.bash \
    && colcon build \
    && rm -rf /idaws/build /idaws/log

# ─── Ortam ───
# GZ_PARTITION sabit olmalı: her kabukta farklı değer üretilirse ros_gz_bridge
# Gazebo'nun topic'lerini göremez (host tarafında yaşanan hata).
ENV GZ_PARTITION=idaws_sim \
    GZ_VERSION=harmonic \
    GZ_SIM_RESOURCE_PATH=/idaws/install/idaws_bringup/share/idaws_bringup/sim/models:/idaws/install/idaws_bringup/share/idaws_bringup/sim/worlds:/opt/gz_ws/gz-waves-models/models:/opt/gz_ws/gz-waves-models/world_models:/opt/gz_ws/gz-waves-models/worlds:/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds:/usr/share/gz/gz-sim8 \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/gz_ws/install/lib:/opt/ardupilot_gazebo \
    LD_LIBRARY_PATH=/opt/gz_ws/install/lib:/usr/lib/x86_64-linux-gnu/OGRE-2.3 \
    GZ_RENDERING_PLUGIN_PATH=/opt/gz_ws/install/lib \
    PATH=/opt/ardupilot/bin:$PATH \
    IDAWS_WS=/idaws \
    IDAWS_LOG_DIR=/idaws/logs \
    IDAWS_RUN_DIR=/idaws/run

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/start-sim.sh  /usr/local/bin/start-sim.sh
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/start-sim.sh \
    && mkdir -p /idaws/run /idaws/logs /idaws/home

EXPOSE 5000

# tini: PID 1 olarak sinyalleri düzgün ilet, zombi süreç bırakma.
# Kayıt mp4'lerinin bozulmaması SIGTERM'in node'lara ulaşmasına bağlı.
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/entrypoint.sh"]
CMD ["start-sim.sh"]
