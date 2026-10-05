# TechsPassion Pad Starter

Drive your own ESP32 robot with the **Controller** mode of the TechsPassion app, then program what it does.
The phone is just a game pad, and your code decides what every move and button means.

## How it works

While you touch the controller, the app sends the whole pad state about 10 times a second:

```
GET /pad?x=40&y=-75&b=3
```

| Value | Meaning |
|---|---|
| `x` | Joystick left (-100) to right (100) |
| `y` | Joystick down (-100) to up (100) |
| `b` | Buttons held: A = 1, B = 2, C = 4, D = 8 (added together) |

Whatever text the robot sends back is shown in the app, so your robot can talk back ("Obstacle!", "Battery 70%").

In the sketch you don't need any of that: just read `padX`, `padY` and `button('A')` in `loop()`, and call `say("...")` to show a message.

## Quick start

1. Install the **Arduino IDE** and **esp32 by Espressif Systems** version 3.x (see the main [README](../../README.md#1-arduino-ide-and-the-esp32-core)).
2. Open `TechsPassion_Pad_Starter.ino`.
3. Change `AP_PASSWORD` (8 characters or more).
4. Check the motor pins. They match the TechsPassion Lite Robot. On another board or driver, change them, and set `FLIP_LEFT` / `FLIP_RIGHT` if a side drives backwards.
   On a classic ESP32 DevKit, pins 38-48 don't exist: use for example 25, 26, 27 (left) and 32, 33, 14 (right).
5. Upload, then join the **TP-PadBot** Wi-Fi on your phone.
6. In the TechsPassion app: **Robot** tab, connect to `192.168.4.1`, then tap **Controller**.

## Make it yours

Everything between `Your code starts here` and `Your code ends here` is yours:

```cpp
if (button('C')) digitalWrite(LED_PIN, HIGH);   // light while C is held
if (padX > 50) say("Turning right");
```

## Safety

The robot stops by itself if no update arrives for 500 ms (the phone left, Wi-Fi dropped, the app closed).
Drive in a clear area and keep the robot in sight.
