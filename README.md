# RCWL-0516 Motion Detector for Heltec V3

Detects people moving nearby with an **RCWL-0516** microwave radar sensor and a **Heltec V3** (WiFi Kit 32 V3 or WiFi LoRa 32 V3). The OLED shows **MOTION** or **Clear** with live event stats. The white LED lights a few milliseconds after the sensor picks up movement.

## Wiring

| RCWL-0516 | Heltec V3 |
|---|---|
| VIN | **5V** (the sensor needs 4–28 V, so 3V3 is too low) |
| GND | GND |
| OUT | **GPIO 7** |
| 3V3, CDS | not connected |

OUT is a 3.3 V signal, so it's safe to connect straight to the ESP32. For the best range, solder a **100 µF capacitor between VIN and GND** on the sensor.

## Screen

```
MOTION          up 12m04s     <- highlighted bar while there's motion
Events:7          10m:3       <- total events / events in the last 10 min
Now: moving 4.2s              <- or "Idle: 1m05s" since the last movement
Last:3.1s, 2m10s ago          <- length of the last event, and when it ended
Max:12s          Sum:48s      <- longest event / total time with motion
Sig:75%          Noise:2      <- live signal strength / rejected spikes
▌ ▌▌   ▌▌▌  ▌                 <- timeline, 1 column per second (~2 min)
```

| PRG button | Action |
|---|---|
| Tap | Reset all stats |
| Hold | Turn the screen off or back on |

## How detection works

The sensor's OUT pin isn't a clean on/off signal. Every detection starts with a burst of flickering, and movement far away gives short, choppy pulses (1–900 ms). A filter that waits for a clean HIGH throws those away, which cuts the range to a hand's length.

Instead, a 1 kHz timer interrupt samples OUT and counts how many of the last 32 ms were HIGH. When the count reaches `CONFIRM_SAMPLES`, that's motion, and the interrupt turns the LED on right away, even while the screen is redrawing. The sensor's own power-up pulse is ignored, and WiFi and Bluetooth are switched off because their power bursts cause false alarms.

## Tuning (top of the sketch)

| Constant | Default | What it does |
|---|---|---|
| `CONFIRM_SAMPLES` | 4 | HIGH ms in the last 32 ms to count as motion. Lower means more range, higher means fewer false alarms |
| `HOLD_MS` | 1000 | How long MOTION stays on after activity stops |
| `WARMUP_MS` | 5000 | Time the sensor gets to settle after power-up |
| `PIN_RADAR` | 7 | GPIO connected to OUT |

## Placement tips

- Point the side of the sensor with the components toward the area you want to watch.
- Keep it 10 cm or more from the Heltec board and its USB cable, with no metal behind it.
- Range is about 5–7 m, and it sees through thin wood, plastic and glass.
- It senses **movement**. Someone standing perfectly still reads as Clear.
- Leave the sensor's **R-GN** pad empty, because a resistor there reduces range.

## Serial

At 115200 baud it prints `motion:<0|1> signal:<0-100>` every 200 ms for **Tools → Serial Plotter**, plus a line when motion starts and ends.

## Build

- Board: **Heltec WiFi LoRa 32(V3)** (esp32 core 3.x)
- Libraries: **Adafruit SSD1306**, **Adafruit GFX**
