# PA3-Robotic (Duckietown Assignment 3)

## Proje Özeti

Bu proje, Duckietown Assignment 3'ün bir Duckiebot üzerinde gerçekleştirilmesini amaçlamaktadır.

Projede kullanılan temel bileşenler:

* `N0` noktasından `N15` noktasına en kısa yolu bulmak için **A*** algoritması
* Hedef düğümü algılamak için **AprilTag (ARTag) tabanlı konumlama**
* Robotu planlanan yol boyunca ilerletmek için **ROS tabanlı hareket kontrolü**

Navigator düğümü, yolu önce hesaplar ve ardından gerçek zamanlı AprilTag algılamaları ile robotu düğümden düğüme yönlendirerek hedefe ulaştırır.

---

## Dosya Yapısı

```text
PA3_ROBOTIC/
├── README.md
├── view_aruco.sh
└── assignment3/
    ├── Dockerfile
    ├── dt-project.yaml
    ├── configurations.yaml
    ├── dependencies-apt.txt
    ├── dependencies-py3.txt
    ├── dependencies-py3.dt.txt
    ├── launchers/
    │   └── default.sh
    └── packages/
        └── assignment3/
            ├── CMakeLists.txt
            ├── package.xml
            ├── launch/
            │   ├── assignment3.launch
            │   └── aruco_viewer.launch
            └── src/
                ├── astar.py
                ├── aruco_viewer.py
                └── navigator_node.py
```

---

## Ana Dosyalar

* **astar.py**
  A* algoritmasını uygular ve hesaplanan yolu ile toplam maliyeti ekrana yazdırır.

* **navigator_node.py**
  Ana ROS düğümüdür. Yolu yükler, AprilTag verilerini dinler ve hareket komutlarını yayınlar.
  Davranış `SEARCH → ALIGN → APPROACH → REACHED` state machine'i ile yürütülür.

* **aruco_viewer.py**
  Bilgisayarınızda açılan, robotun kamera yayınını canlı olarak gösteren ve ArUco
  etiketlerini kutu/ID/uzaklık/yön olarak üstüne çizen viewer uygulamasıdır.

* **assignment3.launch**
  Navigator düğümünü başlatır.

* **aruco_viewer.launch**
  Viewer'ı `rosrun` / `roslaunch` ile başlatmak için launch dosyası.

* **view_aruco.sh** (repo kökü)
  Viewer'ı doğrudan bilgisayarınızdan başlatmak için kabuk betiği.

* **default.sh**
  Duckietown çalıştırma scriptidir (`dts devel run` ile kullanılır).

---

## Robot Adı Nasıl Değiştirilir

Robot adınız `autobot01` değilse aşağıdaki yerleri güncelleyin:

### 1. default.sh

```bash
VEH="${VEHICLE_NAME:-autobot01}"
```

### 2. assignment3.launch

```xml
<arg name="veh" default="$(optenv VEHICLE_NAME autobot01)"/>
```

### 3. navigator_node.py

```python
ROBOT_NAME_DEFAULT = "autobot01"
```

---

## Build

```bash
dts devel build -f --arch arm32v7 -H ROBOTNAME.local
```

`ROBOTNAME` yerine kendi Duckiebot hostname’inizi yazın.

---

## Run

```bash
dts devel run -H ROBOTNAME.local
```

---

## ArUco Viewer (bilgisayarınızda canlı izleme)

ArUco algılamalarını kendi bilgisayarınızda canlı olarak görmek için:

```bash
# ROS master'a erişim (gerekirse)
export ROS_MASTER_URI=http://bear.local:11311
export ROS_HOSTNAME=$(hostname).local

./view_aruco.sh            # varsayılan robot "bear"
./view_aruco.sh autobot01  # farklı robot adı
DEBUG=1 ./view_aruco.sh    # navigator'ın debug_image topic'ini izle
```

Viewer, robotun kamera yayınını bir OpenCV penceresinde gösterir; görülen
her ArUco için kare çerçeve, ID, tahmini mesafe ve bearing (derece) çizer.
Pencereyi kapatmak için `q` veya `ESC`.

Gereksinimler (bilgisayarınızda):
* ROS (noetic / melodic) sourced
* Python 3 + `opencv-contrib-python`, `numpy`
* `duckietown_msgs` / `sensor_msgs`

`roslaunch` ile de başlatılabilir:

```bash
roslaunch assignment3 aruco_viewer.launch veh:=bear
```

---

## Beklenen Çıktı

```text
Path sequence: N0 → N1 → N2 → N6 → N7 → N11 → N15
Total cost: 7.5
Goal Reached
```

---

## Sorun Giderme

### AprilTag Algılanmıyorsa

Varsayılan topic:

```text
/<robot_name>/apriltag_detector_node/detections
```

Kontrol etmek için:

```bash
rostopic list | grep -i april
```

Farklıysa şu parametreyi güncelle:

```python
~apriltag_detections_topic
```

Yanlış topic kullanılırsa robot hedefi bulamaz ve kendi etrafında dönebilir.

---

## Notlar

* ROS ortamının doğru kurulu olduğundan emin olun
* Duckiebot bağlantısını kontrol edin
* Kamera ve AprilTag pipeline düzgün çalışmalı
