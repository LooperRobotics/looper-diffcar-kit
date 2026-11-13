# Looper Differential-Drive Car Kit

English | [中文](README.zh-CN.md)

Build a differential-drive car that maps and navigates on its own: one Looper camera plus an ESP32, a motor driver board, two motors and a few 3D-printed parts.

- Mapping, localization, obstacle avoidance and the web UI all run inside the Looper. **You need a Looper with the differential-drive car system pre-installed**; mention this when ordering.
- This repository covers the car side: bill of materials, wiring and the ESP32 firmware.

| Path | Contents |
|---|---|
| `firmware/esp32/` | ESP32 firmware: drives the motors, reads the wheel encoders, measures battery voltage, runs the status LED, and acts as the car's WiFi adapter |
| `README.md` | This guide: parts, wiring, flashing, first use (Chinese version: `README.zh-CN.md`) |

Models for the 3D-printed parts and assembly photos are still being prepared and will be added under `3d/`.

## 1. Bill of materials

Prices are reference prices from October 2026 (CNY). The links are Taobao listings for reference only; any part with the same specification works.

| Part | Model / specification | Qty | Price (CNY) | Link | Purpose |
|---|---|---|---|---|---|
| Looper camera | With the car system pre-installed | 1 | — | — | Stereo depth camera; the navigation software and web UI run inside it |
| ESP32 dev board | ESP32-S3 N16R8 core board, USB-C, headers soldered | 1 | 25 | [Taobao](https://e.tb.cn/h.j0uvFJCZZ0cFmPI) | Motors, encoders, voltage, LED, WiFi adapter |
| Motor driver board | TB6612 dual-channel driver module (with 5 V regulator) | 1 | 39 | [Taobao](https://e.tb.cn/h.j14kMSvXX27PHJH) | Drives both motors, powers the ESP32 with 5 V, exposes a battery-voltage pin |
| Gear motor set | MG513 Hall-encoder motor, 1:30 gearbox, with encoder cable, 37 mm heavy bracket, hex coupling, 65 mm rubber wheel | 2 sets | 61.5 / set | [Taobao](https://e.tb.cn/h.j0vXTpOY4hPzYhh) | Left and right drive wheels |
| Caster | 1-inch plate caster (with screws) | 1 | 1.4 | [Taobao](https://e.tb.cn/h.j0F3FJUKRffRql1) | Front of the car |
| Li-ion battery | 12 V 5000 mAh, 20 × 60 × 100 mm, with charger and connector cable | 1 | 66.3 | [Taobao](https://e.tb.cn/h.jbEBJNWnJouurii) | Powers the whole car |
| USB cable | USB-C to USB-C, 20 cm | 1 | 9.9 | [Taobao](https://e.tb.cn/h.j0vm15Pdk3tCAU7) | Connects the Looper and the ESP32; the ESP32 is the Looper's network adapter over this cable |
| GH1.25 8-pin cable | | 1 | | | Looper's 8-pin port: power and serial |
| Jumper wires | Female to female | about 17 | | | ESP32 ↔ driver board, ESP32 ↔ Looper serial |
| Screws and nuts | M3 socket-head cap screws, M3 nuts | as needed | | | Chassis counterbores are sized for M3 cap screws (Ø6.5 × 3.2 mm) |

3D-printed parts (PETG):

| Part | Size |
|---|---|
| Chassis | Ø235 × 6 mm disc |
| Camera mount | Base 110 × 44 × 4 mm + upright 110 × 71 × 4 mm |
| Caster spacer | 38 × 32 × about 15 mm |
| Battery case | |
| Driver board case | |

## 2. Wiring

Orientation: the caster is at the front. "Left" and "right" are as seen from behind the car, looking forward.

### 2.1 Power

The 12 V battery feeds two places in parallel:

- the Looper's GH1.25 8-pin port: pin ① 12 V in, pin ② GND;
- the driver board's power input.

The driver board's regulator supplies 5 V to the ESP32 (the 5V and GND wires in 2.3).

### 2.2 Looper ↔ ESP32

**USB**: connect the Looper's USB-C port to the ESP32's **native USB port** (the one wired straight to the chip, not through the serial chip) with the C-to-C cable. The ESP32 shows up on the Looper as a network adapter, and all of the car's WiFi traffic goes over this cable.

**Serial**: pins of the GH1.25 8-pin port (viewed from below, left to right):

| Pin | Signal | Connect to |
|---|---|---|
| ① | 12 V in | Battery + |
| ② | GND | Battery − |
| ③ | External trigger | Leave unconnected |
| ④ | UART_RX (Looper receives) | ESP32 **GPIO41** (ESP32 transmits) |
| ⑤ | UART_TX (Looper transmits) | ESP32 **GPIO40** (ESP32 receives) |
| ⑥ | GND | ESP32 GND (recommended, reduces motor noise) |
| ⑦ | External trigger | Leave unconnected |
| ⑧ | PPS input | Leave unconnected |

The serial lines cross over: the Looper's RX goes to the ESP32's TX, and the Looper's TX to the ESP32's RX.

### 2.3 ESP32 ↔ driver board

Left header row of the ESP32 board, counting down from the first hole below RST:

| Left row # | ESP32 pin | Driver board | Note |
|---|---|---|---|
| 1 | GPIO4 | E2B | Left encoder B |
| 2 | GPIO5 | E2A | Left encoder A |
| 3 | GPIO6 | E1B | Right encoder B |
| 4 | GPIO7 | E1A | Right encoder A |
| 5 | GPIO15 | PWMB | Left wheel |
| 6 | GPIO16 | BIN2 | Left wheel |
| 7 | GPIO17 | BIN1 | Left wheel |
| 8–10 | GPIO18, GPIO8, GPIO3 | — | Leave unconnected |
| 11 | GPIO46 | STBY | Driver enable |
| 12 | GPIO9 | — | Leave unconnected |
| 13 | GPIO10 | ADC | Battery voltage |
| 14 | GPIO11 | AIN1 | Right wheel |
| 15 | GPIO12 | AIN2 | Right wheel |
| 16 | GPIO13 | PWMA | Right wheel |
| Second to last | 5V | 5V | ESP32 is powered by the driver board |
| Last | GND | GND | Grounds must be shared |

### 2.4 Motors

- Left motor to driver channel **B**; its encoder is E2A / E2B.
- Right motor to driver channel **A**; its encoder is E1A / E1B.

If left and right are swapped, the car spins in place or drives backwards.

## 3. Flashing the ESP32

Flashing before mounting the board is easiest.

1. Install [PlatformIO](https://platformio.org/install) (the VS Code extension or the command-line version).
2. Connect the ESP32's **serial USB-C port** (the one behind the CH343 serial chip, usually labelled COM or UART) to your computer.
3. Build and flash:

   ```bash
   cd firmware/esp32
   pio run -t upload
   ```

   If several serial devices are plugged in, pick the port with `--upload-port`, for example `--upload-port /dev/ttyACM0` on Linux or `--upload-port COM5` on Windows.

Firmware updates work the same way. Calibration is stored on the ESP32 and survives updates.

## 4. First power-on: calibrate the encoder direction

A freshly flashed ESP32 does not know the sign of each encoder, and until it does, the car will not follow speed commands. This is a one-time step.

1. **Lift the car so both drive wheels are off the ground.** The wheels turn during calibration.
2. Connect the battery, and connect the ESP32's serial USB-C port to your computer.
3. Open a serial terminal: `pio device monitor` (115200 baud).
4. Type `j` and press Enter. Each wheel turns forward for 1 second and the LED turns yellow. When the firmware prints that calibration is done and the closed loop is unlocked (`标定结束，闭环已解锁`), type `w` and press Enter to save.

The firmware's serial messages are in Chinese. If it reports that a wheel barely moved (`几乎没转`), check that motor's power and encoder wires. `?` + Enter prints the current status, and `P` + Enter prints the wiring table.

## 5. Connecting to the car

### 5.1 Join the car's hotspot

A few seconds after power-on the car opens its own WiFi hotspot (the web UI needs the Looper to finish booting, about 1 minute):

- Name: `Looper-XXXX` (XXXX is a 4-character code unique to each car)
- Default password: `looper1234`

Join it from a phone or computer and open **http://192.168.4.1:8000** in a browser to reach the car's web UI.

Every car ships with the same default password, so change it on the **Device** page (**Change name / password**).

### 5.2 Put the car on your WiFi (optional)

To have the car and your computer on the same network:

1. On the **Device** page, under the network settings, click **Add network** and enter your WiFi name and password. You can save several; they are tried in order.
2. Switch **Connection mode** from **Car hotspot** to **Wi-Fi router**. The page first tries the connection and tells you the car's address on the new network (`http://<address>:8000`).
3. Put your phone or computer back on that WiFi and open that address. On the same network, computers can usually also use **http://looper.local:8000** (Android phones do not support this kind of name).

If you cannot find the car:

- If nobody opens the web UI from the new network within 5 minutes of switching, the car returns to its own hotspot.
- In router mode, if the car cannot join any saved network for 90 seconds, it opens its hotspot too.
- If that still fails: power on and cut power within 10 seconds, twice in a row. On the third power-on the car is forced back into hotspot mode.

## 6. Status LED

The RGB LED on the ESP32 board:

| LED | Meaning |
|---|---|
| One blue blink | Just powered on |
| Solid green | Normal |
| Yellow | Calibrating, self-test, or firmware update in progress |
| Slow red blink | No command for 2 seconds while driving; the car has stopped itself |
| Fast red blink | Wheel stalled; motors stopped |
| Purple blink | Battery low (below 9.5 V), please charge |
| Slow orange blink | No message from the Looper for 5 seconds: the Looper is still booting, the serial wires are loose, or the software on the Looper is not running |
| Slow cyan blink | A web page that was showing the car lost its connection (network dropped, or the phone screen locked). It goes off when that device opens the page again |
| White | The firmware never shows this colour; the LED's data line has picked up motor noise |

## 7. Notes

- A full battery is about 12.6 V; around 11 V is normal. Below 9.5 V the car stops and blinks purple.
- **Unplug the charger before driving.** Charging while driving can disturb the serial link and make the ESP32 reboot repeatedly.
- On a low battery, hard acceleration or sharp turns can briefly cut power to the whole car. Charge in time.

## License

[Apache License 2.0](LICENSE). The files under `firmware/esp32/src/tinyusb_net/` come from [TinyUSB](https://github.com/hathach/tinyusb) and keep its MIT license; the copyright notices are at the top of each file.
