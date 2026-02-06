import matplotlib
matplotlib.use('TkAgg') 
import numpy as np
import matplotlib.pyplot as plt

# Frequenze simulate
"""
Genero due frequenze diverse per IMU e Camera per simulare un EKF asincrono. In particolare
l'IMU gira a 100Hz e la camera a 10Hz.
"""

DT_PHYSICS = 0.01    # Passo di integrazione fisica e IMU (100Hz)
CAM_DOWNSAMPLE = 10  # La camera scatta solo ogni 10 passi di fisica (10Hz)

# Parametri QR Code
REAL_QR_SIDE = 0.10  # Il lato del QR stampato è 10 cm 

# Mappa e Robot
N_LANDMARKS = 5 
AREA_LOWER_BOUND = -10.0
AREA_UPPER_BOUND = 10.0

# ==========================================
# UTILITIES MATEMATICHE
# ==========================================
def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi 

# ==========================================
# EKF ASINCRONO
# ==========================================
class AsynchronousEKF:
    def __init__(self):
        self.mu = np.zeros(4)                    # [x, y, v, theta] inizializzato a zero
        self.P = np.diag([0.1, 0.1, 0.01, 0.1])  # matrice di covarianza iniziale
        self.landmark_id_to_idx = {}             # Mappa ID QR -> Indice nel vettore stato
        
        # Rumori
        self.Q_base = np.diag([0.01, 0.01, 0.01, 0.01]) # Rumore IMU
        self.R_meas = np.diag([0.3, 0.05])             # Rumore Camera: [distanza, angolo]

    def predict(self, imu_accel, imu_omega, dt): #passo dt come argomento
        x, y, v, theta = self.mu[:4]

        # 1. Modello Fisico (Predizione Stato)
        new_x = x + v * np.cos(theta) * dt
        new_y = y + v * np.sin(theta) * dt
        new_v = v + imu_accel * dt
        new_theta = wrap_angle(theta + imu_omega * dt)

        self.mu[0:4] = [new_x, new_y, new_v, new_theta] #aggiorna stato

        # 2. Aggiornamento Covarianza P
        # Jacobiano F rispetto allo stato
        N = len(self.mu)
        F = np.eye(N)
        F[0, 2] = np.cos(theta) * dt
        F[0, 3] = -v * np.sin(theta) * dt
        F[1, 2] = np.sin(theta) * dt
        F[1, 3] = v * np.cos(theta) * dt

        # Scaliamo il rumore Q in base al tempo passato (dt)
        Q = np.zeros((N, N))
        Q[0:4, 0:4] = self.Q_base * dt 
        
        self.P = F @ self.P @ F.T + Q

    def update(self, qr_id, r_meas, b_meas):
        # 1. dato un QR code osservato, controlliamo se è nuovo o noto
        if qr_id not in self.landmark_id_to_idx:
            self.initialize_landmark(qr_id, r_meas, b_meas)
        else:
            self.correct_pose(qr_id, r_meas, b_meas)

    def initialize_landmark(self, qr_id, r, b):
        rx, ry, _, rth = self.mu[:4]
        
        # Calcolo posizione globale landmark
        lx = rx + r * np.cos(rth + b)
        ly = ry + r * np.sin(rth + b)
        
        # Estendo il vettore di stato
        self.mu = np.concatenate((self.mu, [lx, ly]))
        self.landmark_id_to_idx[qr_id] = len(self.mu) - 2
        
        # Estendo covarianza
        old_dim = len(self.P)
        new_P = np.eye(old_dim + 2) * 100.0 # Alta incertezza iniziale
        new_P[:old_dim, :old_dim] = self.P   
        self.P = new_P
        print(f"Nuovo Landmark QR #{qr_id} inizializzato!")

    def correct_pose(self, qr_id, r_meas, b_meas):
        idx = self.landmark_id_to_idx[qr_id]
        rx, ry, _, rth = self.mu[:4]
        lx, ly = self.mu[idx : idx+2]

        # Predizione misura
        dx = lx - rx
        dy = ly - ry
        q = dx**2 + dy**2
        r_pred = np.sqrt(q)
        b_pred = wrap_angle(np.arctan2(dy, dx) - rth)
        
        y_res = np.array([r_meas - r_pred, wrap_angle(b_meas - b_pred)])

        # Jacobiano H
        H = np.zeros((2, len(self.mu)))
        H[0, 0] = -dx / r_pred; H[0, 1] = -dy / r_pred; H[0, 3] = 0
        H[1, 0] = dy / q;       H[1, 1] = -dx / q;      H[1, 3] = -1
        H[0, idx] = dx / r_pred; H[0, idx+1] = dy / r_pred
        H[1, idx] = -dy / q;     H[1, idx+1] = dx / q

        # Update Kalman Standard
        S = H @ self.P @ H.T + self.R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ y_res
        self.mu[3] = wrap_angle(self.mu[3])
        self.P = (np.eye(len(self.mu)) - K @ H) @ self.P

# ==========================================
# CAMERA QR CODE (Simulata)
# ==========================================
class QRCamera:
    def __init__(self):
        self.img_width = 640  
        self.fov = np.deg2rad(60)
        self.focal_length = self.img_width / (2 * np.tan(self.fov / 2))
        
        # Parametri QR
        self.QR_SIDE = REAL_QR_SIDE 
        self.max_dist = 5.0

    def detect_and_measure(self, landmarks, robot_state):
        """
        Simula:
        1. Rilevamento QR code (solo se nel campo visivo)
        2. Calcolo Pixel (bounding box width)
        3. Stima Distanza (Pinhole inversa)
        """
        rx, ry, _, rth = robot_state
        observations = [] # Lista di (QR_ID, Distanza, Angolo)

        for i, lm in enumerate(landmarks):
            # i è l'indice nella lista dei landmark veri.
            # Facciamo finta che ogni landmark abbia un QR code con ID = i
            qr_id = i 

            dx = lm[0] - rx
            dy = lm[1] - ry
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > self.max_dist or dist < 0.2: continue 

            global_angle = np.arctan2(dy, dx)
            rel_angle = wrap_angle(global_angle - rth)
            if abs(rel_angle) > self.fov / 2: continue

            # --- SIMULAZIONE MISURA ---
            # 1. Proiezione nel piano immagine (Pixel)
            # Width in pixel = (Focale * LatoReale) / Distanza
            w_pixel = (self.focal_length * self.QR_SIDE) / dist
            
            # Centro X in pixel
            u_c = self.img_width / 2 
            x_center = u_c - np.tan(rel_angle) * self.focal_length

            # Aggiungiamo rumore ai pixel (Simulazione imperfezione sensore)
            w_pixel += np.random.normal(0, 1) 
            x_center += np.random.normal(0, 2)

            # --- STIMA INVERSA  ---
            # Da pixel a metri
            if w_pixel < 1: continue
            
            # Ricavo distanza sapendo la dimensione del QR
            r_est = (self.focal_length * self.QR_SIDE) / w_pixel
            b_est = np.arctan((u_c - x_center) / self.focal_length)

            observations.append((qr_id, r_est, b_est))
            
        return observations

# ==========================================
# CLASSI SUPPORTO 
# ==========================================
class Unicycle:
    def __init__(self, dt):
        self.state = np.array([0.0, 0.0, 0.0, 0.0]) 
        self.dt = dt
        self.v_cmd = 0.0
        self.theta_cmd = 0.0
        self.u = np.array([0.0, 0.0])

    def step(self):
        v, theta = self.state[2], self.state[3]
        error_v = self.v_cmd - v
        a = 2.0 * error_v      
        omega = self.theta_cmd 
        self.u = np.array([a, omega]) # Questa è l'accelerazione vera

        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dv = a
        dtheta = omega

        self.state += np.array([dx, dy, dv, dtheta]) * self.dt
        self.state[3] = wrap_angle(self.state[3])

class IMU:
    def __init__(self):
        self.rng = np.random.default_rng(0)
    
    def read(self, u):
        # u[0] è accelerazione lineare, u[1] è velocità angolare (omega)
        accel_noise = self.rng.normal(0, 0.02)
        gyro_noise = self.rng.normal(0, 0.005) 
        
        # L'IMU restituisce accelerazione e velocità angolare
        accel_meas = u[0] + accel_noise
        omega_meas = u[1] + gyro_noise
        return accel_meas, omega_meas

class KeyboardController:
    def __init__(self, uni):
        self.uni = uni
    def on_key(self, event):
        if event.key == "up": self.uni.v_cmd += 2.0   
        elif event.key == "down": self.uni.v_cmd -= 2.0
        elif event.key == "left": self.uni.theta_cmd += 1.0 
        elif event.key == "right": self.uni.theta_cmd -= 1.0 
        elif event.key == " ": 
            self.uni.v_cmd = 0.0
            self.uni.theta_cmd = 0.0

# ==========================================
# MAIN SIMULATION LOOP (Asincrono)
# ==========================================
class SimulationAsync:
    def __init__(self):
        print("Avvio SLAM ASINCRONO con QR CODES...")
        
        # Componenti
        self.uni = Unicycle(dt=DT_PHYSICS)
        self.imu = IMU()
        self.camera = QRCamera()
        self.ctrl = KeyboardController(self.uni)
        
        # EKF (Notare: non passiamo dt qui)
        self.ekf = AsynchronousEKF()
        
        # Generazione Mondo (Landmark veri)
        rng = np.random.default_rng(42)
        coords = rng.uniform(AREA_LOWER_BOUND, AREA_UPPER_BOUND, size=(N_LANDMARKS, 2))
        self.landmarks = [np.array(p) for p in coords]

        # Grafica
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect("equal")
        self.ax.set_xlim(AREA_LOWER_BOUND, AREA_UPPER_BOUND)
        self.ax.set_ylim(AREA_LOWER_BOUND, AREA_UPPER_BOUND)
        self.ax.grid(True)
        
        self.lm_plot = self.ax.scatter(*zip(*self.landmarks), c='black', marker='s', label='QR Veri') # Quadrati neri per QR
        self.robot_body, = self.ax.plot([0], [0], "bo", ms=10, zorder=5, label='Robot')
        self.robot_head, = self.ax.plot([0, 0.5], [0, 0], "r-", lw=2, zorder=5)
        self.slam_lm_plot, = self.ax.plot([], [], 'mo', ms=6, label='SLAM Map')
        self.ax.legend(loc='upper right')
        
        # Contatore cicli per simulare frequenze diverse
        self.tick_counter = 0

    def run(self):
        self.fig.canvas.mpl_connect("key_press_event", self.ctrl.on_key)
        plt.ion() 
        plt.show()

        while True:
            if not plt.fignum_exists(self.fig.number): break
            
            self.uni.step() # Il robot si muove fisicamente
            self.tick_counter += 1
            
            # =========================================================
            # 2. IMU PREDICTION (Alta Frequenza - Ogni tick)
            # =========================================================
            # Simuliamo la lettura dell'IMU
            accel, omega = self.imu.read(self.uni.u)
            
            # Eseguiamo la PREDICT ad ogni passo fisico
            self.ekf.predict(accel, omega, DT_PHYSICS)

            # =========================================================
            # 3. CAMERA UPDATE (Bassa Frequenza - Ogni N tick)
            # =========================================================
            # Simuliamo che la camera sia lenta (es. 10Hz vs 100Hz IMU)
            colors = np.full(len(self.landmarks), 'black')
            
            if self.tick_counter % CAM_DOWNSAMPLE == 0:
                # 3a. La camera scatta e processa
                observations = self.camera.detect_and_measure(self.landmarks, self.uni.state)
                
                # 3b. Per ogni QR visto, facciamo UPDATE dell'EKF
                for obs in observations:
                    qr_id, r_meas, b_meas = obs
                    
                    # Logica Debug colori
                    if qr_id < len(colors): colors[qr_id] = 'green'
                    
                    # UPDATE EKF (Usiamo l'ID diretto!)
                    self.ekf.update(qr_id, r_meas, b_meas)

            # =========================================================
            # 4. GRAFICA 
            # =========================================================
            if self.tick_counter % 5 == 0: # 20 FPS video circa
                self.robot_body.set_data([self.uni.state[0]], [self.uni.state[1]])
                hx = self.uni.state[0] + 1.0 * np.cos(self.uni.state[3])
                hy = self.uni.state[1] + 1.0 * np.sin(self.uni.state[3])
                self.robot_head.set_data([self.uni.state[0], hx], [self.uni.state[1], hy])
                
                # Disegno Mappa SLAM
                if len(self.ekf.mu) > 4:
                    map_x = self.ekf.mu[4::2]
                    map_y = self.ekf.mu[5::2]
                    self.slam_lm_plot.set_data(map_x, map_y)
                
                self.lm_plot.set_color(colors)
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
            
            # Pausa piccola per sincronizzare il loop al tempo reale approssimativo
            plt.pause(0.001)

if __name__ == "__main__":
    sim = SimulationAsync()
    sim.run()