# Berdanka-test

Прошивка и вспомогательные TCP GUI для проверки переносимого устройства на
`Arduino UNO R4 WiFi`: IMU `ICM-20948`, TFT `ILI9488`, Wi-Fi AP и отображение
цели на экране.

Подробное техническое задание и текущие договоренности описаны в
[`task.md`](task.md).

## Текущий сценарий

1. Arduino поднимает Wi-Fi AP и TCP-сервер на порту `3333`.
2. Клиент отправляет целеуказание: угол места и азимут цели.
3. Прошивка читает ориентацию устройства из IMU.
4. TFT показывает маркер цели относительно текущего yaw/pitch устройства.
5. Jetson-визуализатор может включить телеметрию и получать ориентацию по Wi-Fi.

UWB, DeepStream и TensorRT engine в текущей версии не используются.
Предполагаемая дальность цели для контрольной визуализации по умолчанию: `300 м`.

## Математика экрана

Расчет согласован с `berdanka-vizualize`:

```text
yaw_offset_deg = device_yaw_deg - target_azimuth_deg
elevation_offset_deg = target_elevation_deg - device_pitch_deg

px_per_deg_x = SCREEN_W / FOV_X
px_per_deg_y = SCREEN_H / FOV_Y

screen_x = CX + yaw_offset_deg * px_per_deg_x
screen_y = CY - elevation_offset_deg * px_per_deg_y
```

Нельзя считать `360° / width`. Экран показывает настраиваемое поле зрения,
по умолчанию `FOV_X=60°`, `FOV_Y=80°`.

## Структура

- `src/main.cpp` - прошивка Arduino: IMU, Wi-Fi/TCP, парсинг команд, TFT HUD.
- `platformio.ini` - сборка PlatformIO для `uno_r4_wifi`.
- `lib/ILI9488/` - локальная библиотека дисплея.
- `tools/windows_tcp_gui.py` - TCP GUI для Windows.
- `tools/jetson_tcp_gui.py` - TCP GUI для Linux/Jetson без DeepStream.
- `tools/jetson_wifi_visualize.py` - контрольная Jetson Linux 3D-визуализация с Wi-Fi телеметрией Arduino.
- `tools/yolo11n.pt` - опциональная локальная YOLO-модель для GUI.
- `tools/quadron_1280.onnx` - опциональная ONNX-модель для GUI.
- `requirements-jetson-viz.txt` - зависимости для 3D-визуализатора.
- `task.md` - ТЗ, протокол, ограничения и будущие этапы.

## TCP протокол

Основной пакет:

```text
MSG:<text>;X:<elevation_deg>;Y:<azimuth_deg>\n
```

Пример:

```text
MSG:TARGET;X:5.0;Y:-12.0
```

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
CFG:TELEMETRY:<0|1>
```

Старый `CFG:DEG_PER_PX:<value>` оставлен только для совместимости и внутри
прошивки пересчитывается в `FOV_X/FOV_Y`.

Если включена `CFG:TELEMETRY:1`, Arduino отправляет строки:

```text
TEL;ROLL:<deg>;PITCH:<deg>;YAW:<deg>;TARGET_PITCH:<deg>;TARGET_YAW:<deg>;PITCH_REL:<deg>;YAW_REL:<deg>;FOV_X:<deg>;FOV_Y:<deg>;ON_TARGET:<0|1>
```

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
python3 tools/jetson_tcp_gui.py
```

Jetson 3D Wi-Fi визуализатор:

```bash
pip install -r requirements-jetson-viz.txt
python3 tools/jetson_wifi_visualize.py
```

Откройте `http://127.0.0.1:8050` на Jetson или `http://<jetson-ip>:8050` с другого устройства.

GUI умеют подключаться к AP Arduino, отправлять ручные углы, менять FOV экрана
и опционально использовать камеру/YOLO для автопередачи смещения цели.
