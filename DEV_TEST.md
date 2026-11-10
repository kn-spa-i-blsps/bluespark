# Test na laptopie: kamera z laptopa → YOLO → ApproachGate → RC override

Cel: sprawdzić, że łańcuch **kamera → `vision_node` → blackboard → `ApproachGate` → `rc_override_node`**
próbuje ruszyć silnikami (czyli że na `/mavros/rc/override` pojawiają się PWM-y ≠ 1500).
Bez Pixhawka i bez mavrosa, więc nic się fizycznie nie kręci.

Dwa kontenery z `docker-compose.dev.yml`: `front` (kamera + YOLO) i `governer` (shell: sterowanie + misja).

> Ten dokument jest napisany bez uruchamiania. Przejdź go ręcznie i zanotuj, co nie zadziałało (sekcja 6).

## 0. Wymagania przed startem
- Model YOLO w `src/bluespark_vision/ml_models/best.pt`. Plik jest w `.gitignore`, więc trzeba go dodać ręcznie
  **przed buildem** (`glob('ml_models/*')` w `setup.py` bierze pliki przy budowaniu).
- Kamera USB / wbudowana widoczna jako `/dev/video0`: `ls /dev/video*`.
- Model musi znać klasę, której szukamy. `ApproachGate` domyślnie szuka `person`. Sprawdź klasy:
  `python3 -c "from ultralytics import YOLO; print(YOLO('src/bluespark_vision/ml_models/best.pt').names)"`
  i jeśli nie ma `person`, podaj inną etykietę przez `-p approach_label:=<klasa>` (krok 4).
- Okno z podglądem nie jest potrzebne, więc `xhost` pomijamy.

## 1. Build (raz, albo po zmianach w `setup.py`/nowych plikach)
```bash
docker compose -f docker-compose.dev.yml build
docker compose -f docker-compose.dev.yml run --rm bluespark_dev
# w środku poczeka na colcon build, wyląduje w shellu → wyjdź: exit
```
Zmiany w istniejących plikach `.py` nie wymagają rebuildu (`--symlink-install`).

## 2. Terminal 1: front (kamera + YOLO)
```bash
docker compose -f docker-compose.dev.yml up front
```
Oczekiwane w logach: `Publishing camera feed (mode=usb, 640x480@30.0fps, ...)`, brak błędu ładowania `best.pt`.

## 3. Terminal 2: governer (sterowanie, bez mavrosa)
```bash
docker compose -f docker-compose.dev.yml run --rm governer
```
W środku shella:
```bash
ros2 run bluespark_control rc_override_node &
sleep 2
ros2 service list | grep control/set_      # ma być 6 serwisów
ros2 topic list | grep front               # /front/camera/image_raw, /front/detected_objects
```
Jeśli w Terminalu 2 nie widać topiców z frontu, oba kontenery nie widzą się w DDS. Sprawdź `ROS_DOMAIN_ID=42`
(`network_mode: host` jest ustawiony w obu).

## 4. Misja (w tym samym shellu governera)
```bash
ros2 run bluespark_autonomy benchTest --ros-args -p mission:=approach -p skip_fcu:=true -p approach_timeout:=60.0
# inna klasa: -p approach_label:=<klasa>
```
`skip_fcu:=true` pomija ustawianie trybu i arming, które bez Pixhawka kończą się FAILURE i przerywają sekwencję.

## 5. Co sprawdzać (drugi shell: `docker exec -it bluespark_governer bash`, potem `source /workspace/install/setup.bash`)
| Co | Komenda | Oczekiwanie |
|---|---|---|
| Wizja widzi cel | `ros2 topic echo /front/detected_objects` | obiekt z `label` ustawioną na szukaną klasę, gdy ją pokażesz |
| PWM-y lecą | `ros2 topic echo /mavros/rc/override` | brak celu: `channels[3]` (yaw) = 1550 (obrót w poszukiwaniu) |
| Centrowanie | przesuwaj cel w kadrze | yaw/heave odchylają się od 1500 proporcjonalnie do kąta |
| Podejście | cel wyśrodkowany, mały/duży w kadrze | `channels[4]` (surge) > 1500, gdy odległość > `TARGET_DISTANCE` |
| Koniec | po `approach_timeout` lub Ctrl+C | wszystkie osie wracają do 1500 |

Kolejność kanałów: `[0]` pitch, `[1]` roll, `[2]` heave, `[3]` yaw, `[4]` surge, `[5]` sway. Kanały `[6..17]` = 0.
`/mavros/rc/override` jest publikowany także bez mavrosa. Nikt go po prostu nie odbiera.

## 6. Znane niewiadome (do sprawdzenia przy pierwszym przejściu)
- `best.pt` może nie mieć klasy `person` ani w ogóle klas z kalibracji. Wtedy `object_config.json` w
  `calibration_files/` może nie mieć wymiarów obiektu, `calculate_pose` zwróci `None` i obiekt zostanie pominięty.
  Zobacz, czy `label` występuje w `object_config.json`.
- `vision_node` używa `imgsz=224` i progu 0.5, słaba kamera laptopa może niczego nie wykryć.
- Kamera: `camera_mode:=usb` czyta `/dev/video0`. Jeśli laptop ma więcej urządzeń, indeks 0 jest na sztywno w `camera.py`.
- Pin `setuptools==65.7.0` jest tylko w `docker-compose.prod.yml`, a dev go nie używa. Jeśli `colcon build` ostrzega o
  `setup.py develop`, to ten sam problem.
- Gdy to zadziała, kolejne kroki to mavros w SITL (ArduSub) i test z uzbrojeniem.

## 7. Sprzątanie
```bash
docker compose -f docker-compose.dev.yml down
```
