---
title: Serial Communication Tutorials
description: "A practical tutorial on talking to microcontrollers and devices over serial links: UART basics, baud rate and a computed bandwidth budget, why raw text breaks and how to frame packets with a header, length and CRC, a Python (pyserial) frame parser tested on a corrupted byte stream, and the matching microcontroller side, and when to use micro-ROS, CAN or Ethernet instead."
aliases:
  - Serial Communication
  - UART
  - pyserial
tags:
  - communication-protocols
  - electronics
  - embedded-systems
type: tutorial
level: beginner
subfield:
  - electronics-embedded
status: reviewed
author: Jordan_Smith
date: 2025-05-02
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[Microcontroller]]"
related:
  - "[[Arduino]]"
  - "[[Hardware_Interface_Setup]]"
  - "[[Python_ROS_Nodes]]"
key_takeaways:
  - "A UART sends bytes one bit at a time with start and stop bits; at 115,200 baud that is about 11.5 kB/s."
  - "Never rely on raw text lines for robot data: frame each message with a header, length, payload and checksum (CRC) so that lost or corrupted bytes are detected and the stream can resynchronise."
  - "For anything beyond a simple link, use micro-ROS, CAN or Ethernet rather than inventing a protocol."
sources:
  - "pySerial developers (2026). pySerial documentation. https://pyserial.readthedocs.io/en/latest/"
  - "Arduino (2026). UNO R4 Minima documentation. https://docs.arduino.cc/hardware/uno-r4-minima/"
  - "micro-ROS (2026). micro-ROS: ROS 2 for microcontrollers. https://micro.ros.org/"
---

# Serial Communication Tutorials

Serial links are the simplest way for a robot's main computer to talk to a [[Microcontroller]]: an [[Arduino]] reading sensors, a motor driver, a GPS receiver. This tutorial covers the basics and how to make the link reliable.

> **Etymology.** *Serial*: bits sent one after another (Latin *series*, "a row"), as opposed to *parallel*, many bits at once. *UART* = universal asynchronous receiver-transmitter.

## UART basics

- **Two wires plus ground**: TX of one side goes to RX of the other. Check voltage levels (3.3 V versus 5 V) before connecting.
- **Baud rate**: bits per second; both sides must match (common values 115,200, 921,600 or higher over USB).
- **Frame**: 1 start bit, 8 data bits, optional parity, 1 stop bit, so about 10 bits per byte ("8N1").
- **USB serial**: most boards appear as `/dev/ttyACM0` or `/dev/ttyUSB0` on Linux.

## Worked example: bandwidth budget

At 115,200 baud with 10 bits per byte the link carries $115{,}200/10 = 11{,}520$ bytes/s. A framed message of 16 bytes (described below) fits $11{,}520/16 = 720$ times per second, enough for 100–500 Hz sensor and command streams. Sending the same data as text such as `"1234.567,-12.345,0.001\n"` takes more bytes and parsing time; binary framing is more compact.

## Framing: make every message checkable

Bytes get lost and corrupted, and a reader can start listening mid-message. A robust frame:

| Field | Bytes | Purpose |
|---|---|---|
| Header | 2 (e.g. `0xAA 0x55`) | Marks the start; lets the reader resynchronise |
| Length | 1 | Payload size |
| Payload | n | The data (e.g. three little-endian float32) |
| CRC-16 | 2 | Detects corruption; drop the frame if it fails |

With a 12-byte payload the frame is $2 + 1 + 12 + 2 = 17$ bytes, fitting about 677 frames per second at 115,200 baud.

## Python side (pyserial)

```python
import serial, struct, binascii

def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)      # CRC-16/CCITT-FALSE

def read_frames(port="/dev/ttyACM0", baud=115200):
    s = serial.Serial(port, baud, timeout=0.1)
    buf = bytearray()
    while True:
        buf += s.read(64)
        while True:
            i = buf.find(b"\xAA\x55")
            if i < 0 or len(buf) < i + 3:
                break
            n = buf[i + 2]
            end = i + 3 + n + 2
            if len(buf) < end:
                break
            payload, crc = buf[i + 3:i + 3 + n], int.from_bytes(buf[end - 2:end], "little")
            del buf[:end]
            if crc16(bytes(payload)) == crc and n == 12:
                yield struct.unpack("<3f", payload)  # e.g. three sensor values
```

Fed a test stream containing stray bytes, a valid frame, a frame with one byte missing, and another valid frame, this parser returned the two valid frames and silently dropped the damaged one, then resynchronised on the next header. The microcontroller builds the same frame: header, length, `memcpy` of the floats, CRC-16 over the payload, then `Serial.write`. Both sides must agree on byte order (little-endian on almost all MCUs) and on the CRC variant.

## In a ROS 2 system

Wrap the reader in a node that publishes standard messages with timestamps (see [[Python_ROS_Nodes]]), or write a [[ros2_control]] hardware interface in C++ for control loops (see [[Hardware_Interface_Setup]]). On supported boards, **micro-ROS** removes the need for a custom protocol entirely.

## Troubleshooting

- Garbage characters: mismatched baud rate or voltage levels.
- Nothing received: TX/RX swapped, or the wrong port or permissions (add your user to the `dialout` group).
- Occasional bad values: no framing or checksum, so add them.
- Delays: buffering on the USB serial adapter; send smaller, more frequent frames, or tune latency settings.

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[Serial_Communication_Tutorials]])
```
