# Rocketry

I like rockets! And you?
If yes, I glad to share my projects with you. If you still not interested, you have to watch my video:

[![Drake 5M rocket](https://img.youtube.com/vi/orwE_tsarW4/0.jpg)](https://www.youtube.com/watch?v=orwE_tsarW4)

Also, you may read articles about my hobby (in russian):
[Drake 5M](https://habr.com/ru/post/571968/) and [Rocket flight theory for experimental Congreve rockets](https://habr.com/ru/post/590281/).

To make it happens, I developed some software for rockets. So, full list is bellow.

---

This folder contains sketches and schemas for candy (rcandy) rocket stuff.
What is implemented:
- Flight controller based on MPU6050 + BMP180 + GPS + LoRa + SD card (Nano)
- Simple flight controller with various sensors (KY-017, SW-420, LM393, BMP180)
- RC Triad (LoRa-controlled smart rc control, ignitor with fuse check, rocket flight controller with telemetry)
- Tranceiver\Receiver with ignitor based on nrF24L01 (Nano) (alternatively can used as base RF433 or LoRa)
- Scale (2 versions) for thrust measurement based on HX711 + SD shield (UNO)
- LoRa receiver + GPS shield to track Flight Controller
