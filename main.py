# ESP32 MicroPython main.py
# PC에서 A:90,90,90,90,90,90 형식으로 명령 수신

from machine import I2C, Pin
from pca9685 import PCA9685
from utime import sleep_ms
import config
import sys

try:
    import select
except ImportError:
    import uselect as select


# ─────────────────────────────────────────
# PCA9685 초기화
# ─────────────────────────────────────────

i2c = I2C(
    0,
    scl=Pin(config.I2C_SCL),
    sda=Pin(config.I2C_SDA),
    freq=400_000
)

devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])

if config.PCA_ADDR not in devices:
    print("ERROR: PCA9685 not found")
    print("Expected address:", hex(config.PCA_ADDR))

    while True:
        sleep_ms(1000)

pca = PCA9685(i2c, config.PCA_ADDR)

print("PCA9685 connected:", hex(config.PCA_ADDR))


# PC에서 보내는 각도 순서
# base, shoulder, elbow, wrist_r, wrist_p, gripper
SERVO_ORDER = [
    "base",
    "shoulder",
    "elbow",
    "wrist_r",
    "wrist_p",
    "gripper"
]


NAME_TO_INDEX = {
    servo["name"]: index
    for index, servo in enumerate(config.SERVO)
}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def write_servo(name, degree):
    """
    config.py의 채널, 방향, 오프셋, 펄스폭을 적용해 서보를 움직입니다.
    """

    if name not in NAME_TO_INDEX:
        print("Unknown servo:", name)
        return

    index = NAME_TO_INDEX[name]
    servo_config = config.SERVO[index]

    # config.py에 기록된 안전 각도 범위
    degree = clamp(
        int(degree),
        servo_config["min_deg"],
        servo_config["max_deg"]
    )

    # 방향 및 중앙 오프셋 적용
    absolute_degree = (
        90
        + servo_config["dir"] * (degree - 90)
        + servo_config["offset"]
    )

    absolute_degree = clamp(absolute_degree, 0, 180)

    # 각도를 서보 펄스폭으로 변환
    pulse_us = int(
        servo_config["min_us"]
        + (
            servo_config["max_us"]
            - servo_config["min_us"]
        )
        * absolute_degree
        / 180
    )

    channel = config.SERVO_CH[index]
    pca.set_us(channel, pulse_us)


def move_all(angles):
    if len(angles) != 6:
        return

    for name, degree in zip(SERVO_ORDER, angles):
        write_servo(name, degree)


def parse_command(line):
    """
    A:90,90,90,90,90,90 형식을 숫자 목록으로 변환
    """

    line = line.strip()

    if not line.startswith("A:"):
        return None

    values = line[2:].split(",")

    if len(values) != 6:
        print("Invalid value count:", line)
        return None

    try:
        return [int(value) for value in values]

    except ValueError:
        print("Invalid command:", line)
        return None


# ─────────────────────────────────────────
# 시작할 때 전체 서보를 90도로 이동
# ─────────────────────────────────────────

print("Moving all servos to neutral position")

for servo_name in SERVO_ORDER:
    write_servo(servo_name, 90)
    sleep_ms(100)

print("READY")
print("Waiting for A:90,90,90,90,90,90")


# ─────────────────────────────────────────
# USB 시리얼 명령 수신
# ─────────────────────────────────────────

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)


while True:
    try:
        events = poller.poll(50)

        if events:
            received_line = sys.stdin.readline()
            angles = parse_command(received_line)

            if angles is not None:
                move_all(angles)
                print("RX:", angles)

    except KeyboardInterrupt:
        print("Stopped")
        break

    except Exception as error:
        print("ERROR:", error)
        sleep_ms(100)
