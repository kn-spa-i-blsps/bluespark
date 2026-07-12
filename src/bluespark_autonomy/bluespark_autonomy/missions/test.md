# Uruchamianie testów na sucho (bench tests)

Cel: odpalić stack na governerze **ręcznie**, uzbroić drona i pchnąć pojedynczą
oś tak, żeby śruby się pokręciły — na sucho, kontrolując każdy krok.

> **Bezpieczeństwo:** to uzbraja drona i kręci śrubami naprawdę. Rób z zdjętymi /
> zabezpieczonymi śrubami, palce z daleka. Killswitch pod ręką. Ctrl+C w każdym
> terminalu z nodem wysyła STOP (1500) na wszystkie kanały.

---

## Kluczowa zasada: NIE odpalaj governera przez `up` do ręcznych testów

W prodzie `WHICH_RPI=governer` uruchamia `run_governer.sh`, który **sam** odpala
`governer.launch.py` z autonomią i pilnuje jej w pętli (restart + wskrzeszanie).
Jeśli wejdziesz do takiego kontenera i odpalisz własny test, będą **dwa drzewa
walczące o te same PWM**, a supervisor będzie wskrzeszał autonomię, którą chcesz
zatrzymać.

Do ręcznych testów odpalaj kontener **z powłoką zamiast supervisora** (patrz
niżej). Front zostaw normalnie na `up` — tam nie ma supervisora.

---

## 0. Front (opcjonalnie — tylko jeśli testujesz coś z wizją)

Na froncie (`.env` ma `WHICH_RPI=front`):
```bash
docker compose -f docker-compose.prod.yml up -d
```
Do testów samego ruchu (wiggle / surge / depth) front NIE jest potrzebny.

---

## 1. Governer: kontener z powłoką (bez supervisora)

Na governer Pi, w katalogu repo:
```bash
docker compose -f docker-compose.prod.yml run --rm --entrypoint bash bluespark
```
`run` + `--entrypoint bash` = kontener wstaje z shellem i **nie** odpala
`run_governer.sh`. Masz środowisko proda, ale sam decydujesz co uruchomić.

W środku kontenera przygotuj środowisko (raz):
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install     # jeśli install/ jeszcze nie zbudowane
source install/setup.bash
```

---

## 2. Odpal stack ręcznie (mavros + control), bez autonomii

Robimy to komponentami, żeby mieć kontrolę i NIE odpalać automatycznie żadnej
misji. W tym samym shellu:

```bash
# mavros -> Pixhawk
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 &
sleep 5

# RC override (wystawia control/set_* — bez tego nic nie pchnie PWM)
ros2 run bluespark_control rc_override_node &
sleep 2

# vehicle_manager (wystawia /manager/set_arming, /manager/set_mode)
ros2 run bluespark_control vehicle_manager_node &
sleep 3
```

Sprawdź, że wszystko wstało i jak NAPRAWDĘ nazywają się serwisy RC (namespace!):
```bash
ros2 node list
ros2 service list | grep -E 'set_(surge|yaw|heave)|set_arming|set_mode'
```
Zanotuj dokładną nazwę np. `control/set_surge` vs `/control/set_surge` — użyjesz
jej w kroku 4.

---

## 3. Uzbrój (najważniejszy krok — tu zwykle leży problem)

```bash
ros2 service call /manager/set_arming std_srvs/srv/SetBool "{data: true}"
```

Patrz na odpowiedź i na logi mavrosa. Dwa scenariusze:

- **Uzbroił się** (mavros pokaże `armed`): świetnie, idź do kroku 4.
- **Odrzucił / w logach `MODE: Unsupported FCU` albo pre-arm fail**: patrz sekcja
  TROUBLESHOOTING niżej — bez uzbrojenia śruby NIE ruszą, choćbyś słał PWM.

---

## 4. Pchnij jedną oś — "klik i kręci"

Użyj dokładnej nazwy serwisu z kroku 2. Przykład dla surge do przodu:
```bash
# lekko do przodu
ros2 service call /control/set_surge bluespark_interfaces/srv/SetRCOverride "{pwm_value: 1600}"

# ...i STOP po chwili (WAŻNE — zrób to zawsze):
ros2 service call /control/set_surge bluespark_interfaces/srv/SetRCOverride "{pwm_value: 1500}"
```
Śruba surge powinna się zakręcić między tymi dwiema komendami. 1500 = stop.

> Jeśli PWM leci (rc_override to loguje), ale śruba stoi -> dron nie jest
> uzbrojony albo tryb blokuje ruch. Wróć do kroku 3 / TROUBLESHOOTING.

---

## 5. To samo, ale przez benchTest (opakowane, z auto-disarm)

Gdy ręczny arm + RC działa, benchTest robi to samo ładniej. W drugim `exec`/oknie
do TEGO SAMEGO kontenera (mavros+control muszą już żyć z kroku 2):

```bash
docker exec -it <nazwa_kontenera> bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# najprostszy ruch:
ros2 run bluespark_autonomy benchTest --ros-args -p mission:=wiggle

# pojedyncza oś z własnym PWM/czasem:
ros2 run bluespark_autonomy benchTest --ros-args -p mission:=surge -p pwm:=1600 -p duration:=2.0
```
benchTest: ALT_HOLD -> arm -> ruch -> disarm, sam się kończy i rozbraja.

Nazwę kontenera znajdziesz przez `docker ps` (będzie typu `bluespark_governer`
lub losowa, jeśli odpalony przez `run`).

---

## 6. Sprzątanie

Ctrl+C na nodach (wysyła STOP), potem:
```bash
ros2 service call /manager/set_arming std_srvs/srv/SetBool "{data: false}"
```
Wyjście z kontenera zamyka wszystko (bo odpalony przez `run --rm`).

---

## TROUBLESHOOTING

### `MODE: Unsupported FCU` (powtarza się w kółko)
mavros nie potrafi obsłużyć trybów tego FCU. Zwykle: mavros nie wie, że to
ArduSub, albo handshake wersji się nie domknął (`VER: broadcast request timeout`).
- Potwierdź w QGroundControl, że firmware to **ArduSub**.
- Do samego kręcenia śrub tryb NIE jest konieczny — liczy się uzbrojenie.
  Spróbuj pominąć `set_mode` i uzbroić od razu (krok 3), potem pchnij RC (krok 4).

### Uzbrojenie odrzucone (na sucho)
ArduSub ma **pre-arm checks** — na biurku (brak ciśnienia/GPS/czujników) potrafi
odmawiać uzbrojenia. To najczęstszy powód, że "na sucho się nie uzbraja".
- W QGroundControl wyłącz odpowiednie **arming checks** na czas testów na sucho.
- Po testach na sucho PRZYWRÓĆ checks przed wodą.

### `Service for surge is unavailable` (benchTest) / brak serwisu (ręcznie)
`rc_override_node` nie działa albo jest w innym namespace niż wołasz.
- Sprawdź `ros2 service list | grep set_surge` i użyj DOKŁADNEJ nazwy.

### Śruby nie ruszają mimo lecącego PWM
Prawie zawsze: dron rozbrojony. Patrz krok 3. RC override bez uzbrojenia jest
przez Pixhawka ignorowane.

### py_trees `TypeError` przy starcie benchTest (mission approach/depth)
Sygnatura dekoratora Timeout/FailureIsSuccess może różnić się wersją. Sprawdź:
```bash
python3 -c "import py_trees; print(py_trees.__version__)"
```
i zgłoś wersję — poprawka jest jednolinijkowa. Misje ruchu (wiggle/surge/square/
allaxes) nie używają Timeout, więc działają niezależnie.