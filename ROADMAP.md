# Plague-Bot VR — Roadmap y seguimiento

Robot de telepresencia (IBERO México) para detección de plagas en invernadero.
Rama de despliegue en la Pi: **`raspberry-deploy`**. Repo: `github.com/orlandocamc/plague-bot-vr` (`main` = sim/Alienware, intacta).

**Meta del demo:** navegación **autónoma** (Nav2 con mapa pre-hecho) + cámara/IA hacia un dashboard.
**Fecha objetivo:** viernes 2026-07-03.
**Fallback si la autonomía no converge:** teleop + waypoints desde el dashboard.

Última actualización: 2026-07-01.

---

## Fases

| # | Fase | Estado |
|---|------|--------|
| 0 | Fundación: consolidar workspace, bridge ESP32, EKF, mapeo motores/encoders | ✅ Hecho |
| 1 | Validación de encoders (test del metro) + manejo recto | 🔄 En curso |
| 2 | SLAM del espacio real (jueves) + guardar mapa | ⬜ Pendiente |
| 3 | Nav2 con el mapa real (click-to-goal) | ⬜ Pendiente |
| 4 | Dashboard: streams a 4 paneles + nodo adaptador (detecciones→DB) | ⬜ Pendiente |
| 5 | Brazo: gesto pre-programado al detectar | ⬜ Pendiente |

---

## Tareas pendientes

### Fase 1 — Encoders y manejo
- [ ] Test del metro: manejar 1.0 m físico, leer `/odom` (encoder puro, NO `/odometry/filtered`).
- [ ] Recalcular si hace falta: `tpr_nuevo = 4480 × (x_odom / 1.0)` y actualizar `ticks_per_rev` en `esp32_bridge.py`.
- [ ] Verificar que va recto; si se desvía, activar PID por lado (`use_pid:=True`) y tunear `pid_kp/ki/kp_ff`.
- [ ] Confirmar sentido de giro real vs. mapeo (`verify` en `calibrate_base.py`).

### Fase 2 — SLAM (jueves, con acceso al espacio)
- [ ] Mapear el espacio real con slam_toolbox (`slam_params.yaml`, loop-closure OFF por filas auto-similares).
- [ ] `map_saver` → guardar `.pgm`/`.yaml` del espacio real (el `greenhouse.pgm` del sim NO sirve).
- [ ] Crear/ajustar launch de SLAM.

### Fase 3 — Nav2
- [ ] Launch de Nav2 con el mapa real + AMCL (`nav2_params.yaml`, `use_sim_time:=false`).
- [ ] Probar click→`/goal_pose` y path-planning autónomo.
- [ ] Decidir techo de autonomía (Nav2 vs. fallback teleop+waypoints).

### Fase 4 — Dashboard (host = laptop/Alienware, la Pi es cliente)
- [ ] Streams: carrito-cam=`pi:8080/stream`, brazo-cam=`/stream2`.
- [ ] Panel LIDAR: roslibjs `/scan` en canvas.
- [ ] Panel Mapa: roslibjs `/map`+pose, con click→`/goal_pose`; teleop buttons→`/cmd_vel`.
- [ ] Nodo adaptador en la Pi: `Detection2DArray` → `POST /robot/datos` (header `x-api-key`).
- [ ] Levantar `rosbridge_server` (:9090) en la Pi.

### Fase 5 — Brazo
- [ ] Gesto pre-programado con `A,` (ángulos fijos) al dispararse una detección.

---

## Avances

- **2026-07-01** — Fundación completa (commits en `raspberry-deploy`):
  - Workspace consolidado en `plaguebot_bringup` (bridge, EKF, yaml nav2/slam/ekf, URDF, meshes).
  - `esp32_bridge.py`: dueño único de `/dev/ttyUSB1`; publica `/odom` + `/imu/data`; recibe `/cmd_vel`→`D,` y `/arm/joint_angles`→`A,`. Auto-cal de bias de giroscopio al arranque; watchdog frena si no hay `/cmd_vel` por 0.5 s.
  - Mapeo de motores calibrado: `motor_signs=[1,1,1,-1]` (M4 trasero-derecho invertido).
  - `ticks_per_rev=4480` medido a mano (E1≈4476, E2≈4484) — falta validar end-to-end.
  - `robot_base.launch.py`: RSP + joint_state_publisher + bridge + EKF + RPLIDAR C1.
  - `calibrate_base.py`: modos watch/pulse/verify/revs/brake.

---

## Notas técnicas a tener en cuenta

### Protocolo ESP32 "Maestro" (Adán), verificado 2026-07-01
- Puerto **`/dev/ttyUSB1`** (CP2102), 115200 8N1, líneas terminadas en `\n`. (`/dev/ttyUSB0` = RPLIDAR C1, 460800.)
- **Telemetría OUT @50Hz:** `E1,E2,E3,E4,AccX,AccY,AccZ,GyrX,GyrY,GyrZ\n`. E1/E2 = encoders delanteros, E3/E4 traseros (pulsos crudos, ints). Acc m/s², Gyro rad/s.
- **Drive IN:** `D,M1_R,M1_L,M2_R,M2_L,M3_R,M3_L,M4_R,M4_L\n` — 8 PWM 0-255. M1=frontal-izq. Freno = todos 0.
- **Brazo IN:** `A,J1,J2,J3,J4,J5,J6\n` — 6 servos 0-180, home=90. El ESP32 interpola a 45°/s.

### SEGURIDAD ⚠️
- **No hay failsafe en firmware:** un PWM latcheado solo se detiene desunudando la batería de motores (NO con reset del ESP32, NO con `D,0`). Watchdog de firmware = TODO de Adán. El watchdog vive por ahora en el nodo del bridge (frena a los 0.5 s sin `/cmd_vel`).
- **Un solo dueño del puerto serie:** solo un proceso puede tener `/dev/ttyUSB1`. Escrituras concurrentes ya latchearon un comando de motor malo. Parar el bridge antes de correr `calibrate_base.py`.
- En pruebas: velocidad baja (~0.1 m/s), ruedas en el aire cuando se calibren motores individuales.

### Calidad de datos IMU (levantar con Adán)
- AccZ≈11.8 en vez de 9.81 (escala ~1.2× → quitar gravedad no es confiable; apoyarse en giroscopio para heading).
- Bias de GyrX≈-0.12 rad/s en reposo (el bridge lo auto-calibra al arranque). GyrZ≈0 en reposo sugiere que Z es yaw.
- EKF fusiona **solo vx** del odom + **vyaw** del IMU (sin orientación absoluta).

### Geometría del robot
- Skid-steer de 4 ruedas (modelo diff-drive). `wheel_radius=0.0762 m`, `track=0.34 m`, `wheel_separation_multiplier=1.65` (comp. de arrastre).
- ADR-0002: el sim intercambia L/R a propósito — verificar contra el cableado real antes de confiar en autonomía.

---

## Riesgos / cosas a vigilar
- **Térmico:** RealSense + Hailo + Nav2 + SLAM + rosbridge en una sola Pi 5.
- **Mapa:** SLAM está en la ruta crítica (jueves, una sola oportunidad con acceso al espacio).
- **Rama sin subir:** confirmar que `raspberry-deploy` esté en el remoto (respaldo).
