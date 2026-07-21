# Berdanka-test

Прошивка и вспомогательные TCP GUI для проверки переносимого устройства на
`Arduino UNO R4 WiFi`: IMU `ICM-20948`, TFT `ILI9488`, Wi-Fi AP и отображение
цели на экране.

Подробное техническое задание и текущие договоренности описаны в
[`task.md`](task.md).

## Текущий сценарий

1. Arduino поднимает Wi-Fi AP, TCP-сервер и UDP-приемник на порту `3333`.
2. TFT показывает черный экран с прицелом, маркером в центре и коротким zero-индикатором в углу.
3. Прошивка сразу вычитывает DMP FIFO и берет быстрый локальный zero по короткому стабильному окну.
4. После zero-калибровки прошивка заново центрирует маркер.
5. Клиент отправляет целеуказание: угол места и азимут цели.
6. Прошивка читает ориентацию устройства из IMU.
7. TFT показывает маркер цели относительно текущего yaw и roll-derived elevation устройства.
8. Jetson control panel отправляет цель и настройки по Wi-Fi без обратной телеметрии.
9. Цель не сохраняется в EEPROM: после перезагрузки Arduino стартует без сохраненной цели.

IMU работает через DMP `GAME_ROTATION_VECTOR` / Quat6, без магнитометра.
На старте прошивка постоянно вычитывает FIFO и берет zero сразу после короткого
непрерывного стабильного окна IMU. Если устройство движется прямо в момент
zero, на экране остается `KEEP STILL`, а прошивка ждет новое стабильное окно.
Стартовая zero-калибровка и `CMD:CENTER` задают локальный базис; абсолютный
север не используется.

Монтаж IMU в Arduino-версии:

```text
Y - прямо сквозь дисплей
X - вправо по дисплею
Z - вверх по дисплею
```

Курс/yaw берется как поворот вокруг `Z`. Вертикальное смещение маркера на
экране берется из roll-компонента устройства вокруг `X`, поэтому roll отвечает
за движение маркера вверх/вниз.
После zero-калибровки Quat6 yaw продолжает жить без магнитного ориентира, поэтому
прошивка включает runtime auto-zero: когда устройство неподвижно, `zeroYaw`
медленно подтягивается к raw yaw и убирает уход маркера. В IMU-логе для
диагностики есть `zero_ready`, `zero_stable_ms`, `yaw_auto_zero`,
`yaw_still_ms` и `yaw_auto_zero_deg`.

UWB, DeepStream и TensorRT engine в текущей версии не используются.
Предполагаемая дальность цели для контрольной визуализации по умолчанию: `20 м`.

## Математика экрана

Расчет согласован с `berdanka-vizualize`:

```text
yaw_offset_deg = target_azimuth_deg - device_yaw_deg
elevation_offset_deg = target_elevation_deg - device_roll_as_elevation_deg

px_per_deg_x = SCREEN_W / FOV_X
px_per_deg_y = SCREEN_H / FOV_Y

screen_x = CX + yaw_offset_deg * px_per_deg_x
screen_y = CY - elevation_offset_deg * px_per_deg_y
```

Нельзя считать `360° / width`. Экран показывает настраиваемое поле зрения.
Для камеры 8 мм, 1/2", 4:3 по умолчанию используется `FOV_X=43.60°`,
`FOV_Y=33.40°`.

Связь камеры `640x480` и TFT `480x320` сделана через нормализацию полного
кадра камеры на полный экран:

```text
display_x = camera_x * 480 / 640
display_y = camera_y * 320 / 480
```

Поэтому дефолтный `FOV_X/FOV_Y` дисплея равен дефолтному `HFOV/VFOV` камеры.

## Структура

- `src/main.cpp` - `setup()`, `loop()` и порядок работы модулей.
- `src/config.h` - пины, размеры экрана, таймауты и настройки по умолчанию.
- `src/app_state.*` - общие объекты Arduino и состояние приложения.
- `src/angle_utils.h` - угловая математика и clipping прямоугольников.
- `src/imu_zero.*` - IMU DMP, чтение FIFO, прогрев и zero-калибровка.
- `src/ui.*` - прицел, статусы и countdown.
- `src/marker.*` - отрисовка и очистка маркера цели.
- `src/net.*` - Wi-Fi AP, TCP/UDP и парсинг команд.
- `platformio.ini` - сборка PlatformIO для `uno_r4_wifi`.
- `lib/ILI9488/` - локальная библиотека дисплея.
- `tools/windows_tcp_gui.py` - TCP GUI для Windows.
- `tools/jetson_tcp_gui.py` - TCP GUI для Linux/Jetson с камерой, YOLO и ручной целью.
- `tools/jetson_wifi_visualize.py` - легкая Jetson Linux control panel без графиков: цель, настройки и круговая проверка движения маркера.
- `tools/yolo11n.pt` - опциональная локальная YOLO-модель для GUI.
- `tools/quadron_1280.onnx` - опциональная ONNX-модель для GUI.
- `requirements-jetson-viz.txt` - зависимости для Jetson control panel.
- `task.md` - ТЗ, протокол, ограничения и будущие этапы.

## Wi-Fi протокол

Основной пакет:

```text
MSG:<text>;X:<elevation_offset_deg>;Y:<azimuth_offset_deg>\n
```

Пример:

```text
MSG:TARGET;X:5.0;Y:12.0
```

`X=0;Y=0` означает активный базис. При старте он создается после быстрой
zero-калибровки IMU, а `CMD:CENTER` переносит его в текущую ориентацию устройства.
Повторная отправка одного и того же `MSG` не сдвигает цель дальше.
Положительный `Y` смещает маркер вправо, положительный `X` смещает маркер
вверх. Дальность в Arduino сейчас не используется и не влияет на положение
квадрата без UWB/позиции устройства.

Служебные команды:

```text
PING
CMD:CENTER
CFG:FOV_X:<deg>
CFG:FOV_Y:<deg>
CFG:TCP_POLL_MS:<ms>
CFG:BOX_SIZE:<px>
CFG:BOX_REFRESH_MS:<ms>
CFG:BOX_RENDER:<0|1>
CFG:BOX_DELTA_RENDER:<0|1>
MSGONLY:<text>
```

`CMD:CENTER` переносит активный базис цели в текущую ориентацию устройства и
центрирует маркер. После этого обычные `MSG` снова трактуются как смещения от
нового базиса.

Старый `CFG:DEG_PER_PX:<value>` оставлен только для совместимости и внутри
прошивки пересчитывается в `FOV_X/FOV_Y`.

Jetson control panel отправляет тихий `PING` примерно раз в секунду, чтобы Arduino
не закрывала TCP по heartbeat timeout и чтобы UDP-статус оставался `UDP OK`.
Для частой передачи цели предпочтителен UDP; TCP оставлен как fallback.

## Сборка

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

Если `pio` не доступен в терминале, используйте PlatformIO из VS Code.

## GUI

Windows:

```bash
python tools/windows_tcp_gui.py
```

Linux/Jetson:

```bash
cd ~/ivank/Berdanka-test
sudo nvpmodel -m 8
sudo jetson_clocks
python3 tools/jetson_tcp_gui.py
```

На Jetson GUI по умолчанию выбирает `Quadron 1280 TensorRT FP16 (.engine)`,
загружает `tools/quadron_1280_fp16.engine` и включает YOLO detection. Engine
собирается для конкретного Jetson/TensorRT и поэтому игнорируется Git через
`*.engine`; его нужно хранить или копировать отдельно от репозитория.

USB-камеры обнаруживаются автоматически каждые две секунды. Поле `Camera`
показывает доступные `/dev/video*`, а поле `Mode` заполняется поддерживаемыми
разрешениями, FPS и форматами из `v4l2-ctl`. `Auto (camera default)` оставляет
выбор драйверу камеры. При первом обнаружении GUI выбирает доступный режим,
ближайший к входу Quadron `1280x1280`. После выбора нажмите `Start`; TensorRT inference идет в
отдельном worker-потоке, а устаревшие кадры отбрасываются.

Jetson Wi-Fi control panel:

```bash
pip install -r requirements-jetson-viz.txt
python3 tools/jetson_wifi_visualize.py
```

Откройте `http://127.0.0.1:8050` на Jetson или `http://<jetson-ip>:8050` с другого устройства.
Для проверки отрисовки включите `Move target in circle`: Jetson будет отправлять
цель по кругу через `MSG:<text>;X:<elevation>;Y:<azimuth>`.

TCP GUI умеет подключаться к AP Arduino, отправлять ручные углы, менять FOV
экрана и опционально использовать камеру для автопередачи смещения цели через
YOLO, отдельный контрастный трекер или ORB-трекер OpenCV. Windows-версия
полностью синхронизирована с Jetson TCP GUI и при запуске из PowerShell
использует `netsh` для подключения к Wi-Fi, а Linux/Jetson использует `nmcli`.

В TCP GUI камера находится в изменяемой панели: потяните разделитель между
вкладками и изображением, чтобы менять размер preview. Во вкладке `Target`
можно задать `X`/`Y`, размер квадрата на камере, отправить цель вручную,
включить автопосылку или кликнуть по изображению камеры, чтобы выставить цель
по текущим `HFOV/VFOV`, `trim` и `invert` настройкам.
Там же есть `Circle test around 0/0`: он отправляет цель по окружности вокруг
активного базиса, с настраиваемыми радиусами по yaw/pitch, периодом и частотой
отправки.
Во вкладках `Contrast` и `ORB` можно включить соответствующий OpenCV-захват,
настроить фильтры и стабилизацию точки, посмотреть статус трекера в GUI и
отдельно разрешить отправку выбранной точки на Arduino. На изображении камеры
для этих режимов рисуется только одна выбранная точка без отладочного текста.

## IMU лог

Прошивка печатает строки `IMU,...` примерно 10 раз в секунду. Для записи CSV и живых
графиков с Windows COM4:

```bash
pip install pyserial matplotlib
python tools/plot_imu_com4.py --port COM4
```
