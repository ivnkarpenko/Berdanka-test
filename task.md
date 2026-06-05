# Berdanka-test: техническое задание и текущее состояние

## 1. Назначение

`Berdanka-test` - аппаратный тестовый проект для переносимого устройства с
дисплеем. Он должен показать на TFT, куда относительно текущей ориентации
устройства находится цель, полученная от внешнего источника целеуказания.

Проект является практической проверкой части математики из
`berdanka-vizualize`, но без UWB-позиционирования на текущем этапе.

## 2. Аппаратная платформа

- MCU: `Arduino UNO R4 WiFi`.
- IMU: `ICM-20948`, подключение по I2C.
- Дисплей: `ILI9488`, SPI, рабочая ориентация landscape `480x320`.
- Wi-Fi: встроенный модуль UNO R4 WiFi.

Пины дисплея в прошивке:

```text
TFT_CS  = 10
TFT_DC  = 9
TFT_RST = 8
TFT_LED = A0
```

## 3. Что сейчас реализовано

- Arduino поднимает Wi-Fi AP `cisco` / `cisco1234`.
- TCP-сервер слушает порт `3333`.
- IMU читается через DMP `GAME_ROTATION_VECTOR`.
- При старте выполняется 10-секундная нулевая калибровка.
- На экране есть HUD: `Roll`, `Pitch`, `Yaw`, `Msg`, `IP`, статус IMU и TCP.
- Маркер цели рисуется поверх прицела.
- Маркер зеленый, если устройство наведено в допуске, иначе красный.
- При `TCP WAIT` маркер полый, при `TCP OK` маркер залитый.
- Есть throttling TCP, HUD и рендера маркера для снижения лагов.
- Настройки экрана и рендера можно менять TCP-командами.
- По команде `CFG:TELEMETRY:1` Arduino отдает текущие углы и состояние цели по Wi-Fi.
- Добавлена контрольная Jetson Linux 3D-программа с целью, координатами цели и приемом Wi-Fi телеметрии.

## 4. Что сознательно не используется

### UWB

UWB пока не используется в `Berdanka-test`.

Сейчас прошивка не принимает позицию устройства, расстояния до якорей и не
делает компенсацию смещения переносимого устройства. Это соответствует режиму
`no_uwb` из `berdanka-vizualize`: устройство получает направление на цель и
отображает его относительно своей IMU-ориентации.

UWB можно добавить позже отдельным этапом:

- принять позицию устройства в мировой системе координат;
- рассчитать вектор `target - device_position`;
- использовать ту же проекцию на экран;
- для реальной 2D-схемы желательно 3 якоря, для 3D - 4 якоря.

### DeepStream и TensorRT engine

DeepStream-конфиги, custom parser и TensorRT engine-сценарий удалены из проекта.
Для Jetson остается обычный Python GUI. Если нужна детекция, она должна идти
через локальную YOLO/ONNX модель в GUI без DeepStream pipeline.

## 5. Согласование с berdanka-vizualize

Визуализатор задает мировую систему:

```text
X - восток / вправо
Y - север / вперед
Z - вверх
```

Цель задается из статического источника как:

```text
azimuth_deg
elevation_deg
range_m
```

В текущей прошивке `range_m` не используется, потому что без UWB и без знания
позиции переносимого устройства дальность не меняет направление на экране.
В контрольной Jetson-визуализации предполагаемая дальность цели по умолчанию
задана `300 м`.

Главное правило, перенесенное из `berdanka-vizualize`: экран показывает не
`360°`, а настраиваемое поле зрения.

```text
deg_per_px_x = FOV_X / SCREEN_W
deg_per_px_y = FOV_Y / SCREEN_H
```

В прошивке используется обратная форма:

```text
px_per_deg_x = SCREEN_W / FOV_X
px_per_deg_y = SCREEN_H / FOV_Y
```

По умолчанию:

```text
SCREEN_W = 480
SCREEN_H = 320
FOV_X = 60°
FOV_Y = 80°
```

## 6. Формулы прошивки

Вход:

```text
target_azimuth_deg
target_elevation_deg
device_yaw_deg
device_pitch_deg
```

Смещения:

```text
yaw_offset_deg = wrap180(device_yaw_deg - target_azimuth_deg)
elevation_offset_deg = target_elevation_deg - device_pitch_deg
```

Экран:

```text
screen_x = CX + yaw_offset_deg * px_per_deg_x
screen_y = CY - elevation_offset_deg * px_per_deg_y
```

Знак yaw оставлен в старой логике прошивки: `device_yaw - target_yaw`.

## 7. TCP-протокол

### Основной пакет цели

Формат оставлен совместимым со старым GUI:

```text
MSG:<message>;X:<elevation_deg>;Y:<azimuth_deg>\n
```

Где:

- `X` - угол места цели, градусы;
- `Y` - азимут цели, градусы;
- `message` - короткая подпись для HUD.

Пример:

```text
MSG:TARGET;X:4.5;Y:-18.0
```

### Служебные команды

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

`CMD:CENTER` записывает текущие `pitch/yaw` как целевые углы и центрирует маркер.

`CFG:DEG_PER_PX:<value>` оставлен только как legacy-команда. Прошивка переводит
ее в:

```text
FOV_X = value * SCREEN_W
FOV_Y = value * SCREEN_H
```

### Телеметрия от Arduino

По умолчанию телеметрия выключена, чтобы старые GUI не заполняли лог. Новая
Jetson 3D-программа включает ее после подключения:

```text
CFG:TELEMETRY:1
```

Формат строки:

```text
TEL;ROLL:<deg>;PITCH:<deg>;YAW:<deg>;TARGET_PITCH:<deg>;TARGET_YAW:<deg>;PITCH_REL:<deg>;YAW_REL:<deg>;FOV_X:<deg>;FOV_Y:<deg>;ON_TARGET:<0|1>
```

Эта строка нужна для контрольной визуализации: Jetson получает ориентацию
устройства по Wi-Fi и отображает ее в 3D-сцене.

## 8. Репозиторий после чистки

Оставлено:

- `src/main.cpp` - основная прошивка;
- `platformio.ini` - PlatformIO-профиль;
- `lib/ILI9488/` - локальная библиотека дисплея;
- `tools/windows_tcp_gui.py` - Windows GUI;
- `tools/jetson_tcp_gui.py` - Linux/Jetson GUI;
- `tools/jetson_wifi_visualize.py` - Jetson Linux 3D-визуализация с Wi-Fi телеметрией Arduino;
- `tools/yolo11n.pt` и `tools/quadron_1280.onnx` - опциональные модели для GUI;
- `requirements-jetson-viz.txt` - зависимости Jetson 3D-визуализатора;
- `README.md` и `task.md`.

Удалено:

- `tools/deepstream_quadron/`;
- DeepStream custom parser и конфиги;
- TensorRT engine-конфигурация;
- `trilateration_gui.py`, потому что UWB пока не используется;
- старый Processing serial animation sketch.

Игнорируется:

- `.pio/`;
- `.venv/`;
- `__pycache__/`;
- `runs/`;
- локальные `*.engine`, `*.plan`, `*.so`;
- локальные веса `yolo_quadro_weights/`.

## 9. Сборка и запуск

Сборка:

```bash
pio run
```

Прошивка:

```bash
pio run -t upload
```

Монитор:

```bash
pio device monitor -b 115200
```

Windows GUI:

```bash
python tools/windows_tcp_gui.py
```

Linux/Jetson GUI:

```bash
python3 tools/jetson_tcp_gui.py
```

Jetson 3D Wi-Fi визуализатор:

```bash
pip install -r requirements-jetson-viz.txt
python3 tools/jetson_wifi_visualize.py
```

Открыть:

```text
http://127.0.0.1:8050
```

Возможности:

- подключение к Arduino TCP `192.168.4.1:3333`;
- включение Wi-Fi телеметрии `CFG:TELEMETRY:1`;
- задание цели углами `azimuth/elevation/range`;
- задание цели координатами `X/Y/Z`;
- перемещение цели с живым обновлением 3D-сцены;
- отправка текущей цели на Arduino в формате `MSG:JETSON;X:<elevation>;Y:<azimuth>`;
- отображение статического источника, устройства, цели, луча на цель и направления устройства;
- отображение виртуального экрана устройства с той же yaw-конвенцией, что в прошивке.

## 10. Требования к поведению

- После старта устройство должно дождаться IMU и выполнить нулевую калибровку.
- Без TCP-клиента устройство должно показывать `TCP WAIT`.
- При подключении TCP-клиента статус должен стать `TCP OK`.
- Пакет `MSG` должен обновлять цель и сохранять ее в EEPROM.
- `CMD:CENTER` должен центрировать цель на текущем направлении устройства.
- Yaw-смещение должно считаться по старой конвенции `device_yaw - target_yaw`.
- Цель выше текущего pitch должна двигать маркер вверх.
- Изменение `FOV_X/FOV_Y` должно менять масштаб маркера без перепрошивки.
- Маркер должен оставаться частично видимым у края экрана.
- Jetson-визуализатор должен иметь предполагаемую дальность цели `300 м` по умолчанию.

## 11. Будущие задачи

- Перейти с `String` в TCP parser на фиксированный `char[]` буфер.
- Вынести Wi-Fi credentials и тайминги в отдельный профиль конфигурации.
- Добавить watchdog/recover для Wi-Fi.
- Добавить sequence number и диагностический ACK-режим.
- После появления UWB-части перенести из `berdanka-vizualize` расчет
  `target - device_position` и сравнение ошибок `no_uwb` / `uwb`.
