# EKF SLAM con QR Code su Robot LIMO (ROS 2)

Questo repository contiene il codice sviluppato per il progetto di Sistemi e Sensori per la Robotica.
Il sistema implementa un algoritmo **Extended Kalman Filter (EKF)** per la localizzazione e mappatura simultanea (SLAM) utilizzando un robot **AgileX LIMO** e QR Code come landmark visivi.

## Caratteristiche
* **Stima della posa:** Fusione sensoriale tra Odometria e Visione.
* **Landmark:** Rilevamento e stima della posizione di QR Code (OpenCV).
* **Visualizzazione:** Integrazione completa con RViz (Mappa, Traiettoria, Marker 3D).
---

## Istruzioni per l'uso

Il workflow è diviso in due fasi: **Registrazione** (sul robot) ed **Esecuzione** (sul PC).

### 1. Registrazione Dati (Sul Robot)
Collegarsi via SSH al LIMO e avviare i driver e la registrazione:

```bash
# 1. Avviare i driver base e la camera
ros2 launch limo_bringup limo_start.launch.py
ros2 launch orbbec_camera dabai.launch.py

# 2. Registrare la Rosbag (Dataset)
ros2 bag record -o ROSbag_Con_TF /camera/color/image_raw /camera/color/camera_info /odom /tf /tf_static

```

### 2. Esecuzione SLAM (Sul PC)

Dopo aver copiato la rosbag sul PC, avviare la simulazione:

```bash
# 1. Avviare RViz (Visualizzazione)
rviz2

# 2. Avviare il Playback dei dati (IMPORTANTE: usare --clock)
ros2 bag play ROSbag_Con_TF --clock -l

# 3. Avviare il Nodo EKF SLAM
python3 src/simulatore_real.py --ros-args -p use_sim_time:=true

```

---

## Visualizzazione (Topic ROS)

Su RViz puoi sottoscriverti ai seguenti topic per vedere i risultati:

* `/slam_pose` → Posizione stimata del robot.
* `/slam_landmarks` → Posizione stimata dei QR Code (Cubetti verdi).
* `/slam_path` → Scia del percorso effettuato.
* `/camera/color/image_raw` → Stream video con i QR rilevati.

## Autore

**Francesco Datres**

```

```
