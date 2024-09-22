
# Aircombat-Simulation

Bu proje, hava muharebesinde bulanık mantık kullanarak karar verme ve hava aracı takip algoritmalarını uygulamayı hedeflemektedir. Sistem, dinamik olarak en tehlikeli uçağı belirler ve A* algoritmasını kullanarak hedefe en güvenli rotayı bulur. YOLOv8 entegre edilerek gerçek zamanlı uçak takibi yapılır, ardından başarılı bir it dalaşı (dogfight) simülasyonu gerçekleştirilir. Ayrıca, Deep Q-Network (DQN) tabanlı bir kontrolcü modeli geliştirilmiş ve uçak ROS tabanlı bir sistem aracılığıyla kontrol edilmiştir.

## Özellikler
- Dinamik tehdit değerlendirmesi için bulanık mantık.
- A* algoritması ile rota planlama ve çarpışma önleme.
- YOLOv8 tabanlı nesne algılama ile uçak takibi.
- Takviyeli öğrenme ve DQN ile uçak kontrolü.
- ROS entegrasyonu ile gerçek zamanlı kontrol ve simülasyon.

## Başlarken

### Gereksinimler

Bu yazılımı çalıştırmak için aşağıdaki yazılımların kurulu olduğundan emin olun:

- **Ubuntu 20.04**
- **ROS Noetic**
- **Hector Quadrotor paketi**

### Kurulum

1. Depoyu klonlayın:
   ```bash
   git clone https://github.com/Burakzdd/Aircombat-Simulation.git
   ```
2. Gerekli ROS paketlerini yükleyin:
   ```bash
   sudo apt install ros-noetic-hector-quadrotor
   ```
3. Çalışma alanını derleyin:
   ```bash
   cd Aircombat-Simulation
   catkin_make
   ```
4. Çalışma alanını kaynak dosyası yapın:
   ```bash
   source devel/setup.bash
   ```

### Simülasyonu Çalıştırma

1. Simülasyon ortamını başlatın:
   ```bash
   roslaunch aircombat_simulation aircombat.launch
   ```
2. Kontrol algoritmasını çalıştırın:
   ```bash
   rosrun aircombat_simulation control.py
   ```

## Gereksinimler

- **Ubuntu 20.04**
- **ROS Noetic**
- **Hector Quadrotor paketi**
