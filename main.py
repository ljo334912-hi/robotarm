# ESP32 MicroPython main.py
# PCA9685 로봇팔: 각도 명령 + 좌표 명령 지원

from machine import I2C, Pin
from pca9685 import PCA9685
from utime import sleep_ms
import config
import sys
import math

try:
    import ujson as json
except ImportError:
    import json

try:
    import select
except ImportError:
    import uselect as select


# PC에서 사용하는 서보 순서
SERVO_ORDER = [
    "base",
    "shoulder",
    "elbow",
    "wrist_r",
    "wrist_p",
    "grip"
]

NAME_TO_INDEX = {
    servo["name"]: index
    for index, servo in enumerate(config.SERVO)
}

current_angles = [90, 90, 90, 90, 90, 90]


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
print("I2C:", [hex(device) for device in devices])

if config.PCA_ADDR not in devices:
    print("ERROR: PCA9685 not found")

    while True:
        sleep_ms(1000)

pca = PCA9685(i2c, config.PCA_ADDR)


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def write_servo(name, degree):
    index = NAME_TO_INDEX[name]
    servo = config.SERVO[index]

    degree = clamp(
        float(degree),
        servo["min_deg"],
        servo["max_deg"]
    )

    # 방향 및 중앙 오프셋 적용
    absolute_degree = (
        90
        + servo["dir"] * (degree - 90)
        + servo["offset"]
    )

    absolute_degree = clamp(absolute_degree, 0, 180)

    pulse_us = int(
        servo["min_us"]
        + (
            servo["max_us"]
            - servo["min_us"]
        )
        * absolute_degree
        / 180
    )

    channel = config.SERVO_CH[index]
    pca.set_us(channel, pulse_us)


def move_direct(angles):
    """손 추적처럼 연속으로 들어오는 각도를 즉시 적용합니다."""

    if len(angles) != 6:
        return False

    for index, name in enumerate(SERVO_ORDER):
        servo = config.SERVO[NAME_TO_INDEX[name]]

        safe_angle = clamp(
            float(angles[index]),
            servo["min_deg"],
            servo["max_deg"]
        )

        write_servo(name, safe_angle)
        current_angles[index] = safe_angle

    return True


def move_smooth(target_angles, delay=15):
    """현재 위치에서 목표 위치까지 부드럽게 이동합니다."""

    safe_targets = []

    for index, name in enumerate(SERVO_ORDER):
        servo = config.SERVO[NAME_TO_INDEX[name]]

        safe_targets.append(
            clamp(
                float(target_angles[index]),
                servo["min_deg"],
                servo["max_deg"]
            )
        )

    maximum_change = max(
        abs(safe_targets[index] - current_angles[index])
        for index in range(6)
    )

    # 약 2도씩 이동
    steps = max(1, int(maximum_change / 2))

    start_angles = current_angles[:]

    for step in range(1, steps + 1):
        ratio = step / steps

        for index, name in enumerate(SERVO_ORDER):
            angle = (
                start_angles[index]
                + (
                    safe_targets[index]
                    - start_angles[index]
                )
                * ratio
            )

            write_servo(name, angle)

        sleep_ms(delay)

    for index in range(6):
        current_angles[index] = safe_targets[index]


def inverse_kinematics(x, y, z):
    """
    로봇 좌표(cm)를 서보 각도로 변환합니다.

    x: 로봇 앞쪽
    y: 로봇 좌우
    z: 바닥으로부터 높이
    """

    d1 = config.LINK["d1"]
    a2 = config.LINK["a2"]
    a3 = config.LINK["a3"]
    a5 = config.LINK["a5"]

    x = float(x)
    y = float(y)
    z = float(z)

    # 베이스 좌우 회전
    base_angle = 90 + math.degrees(math.atan2(y, x))

    # 그리퍼 길이를 제외한 손목 중심 위치
    radial = math.sqrt(x * x + y * y) - a5
    vertical = z - d1

    if radial < 0:
        raise ValueError("target is too close")

    distance = math.sqrt(
        radial * radial + vertical * vertical
    )

    minimum_reach = abs(a2 - a3)
    maximum_reach = a2 + a3

    if distance < minimum_reach or distance > maximum_reach:
        raise ValueError(
            "unreachable: distance={:.1f}cm, range={:.1f}~{:.1f}cm".format(
                distance,
                minimum_reach,
                maximum_reach
            )
        )

    elbow_cos = (
        radial * radial
        + vertical * vertical
        - a2 * a2
        - a3 * a3
    ) / (2 * a2 * a3)

    elbow_cos = clamp(elbow_cos, -1.0, 1.0)
    elbow_rad = math.acos(elbow_cos)

    shoulder_rad = (
        math.atan2(vertical, radial)
        - math.atan2(
            a3 * math.sin(elbow_rad),
            a2 + a3 * math.cos(elbow_rad)
        )
    )

    shoulder_angle = 90 - math.degrees(shoulder_rad)
    elbow_angle = math.degrees(elbow_rad)

    target_angles = [
        base_angle,
        shoulder_angle,
        elbow_angle,
        90,                 # 손목 회전
        90,                 # 손목 상하
        current_angles[5]   # 현재 그리퍼 각도 유지
    ]

    return target_angles


def send_response(response):
    print(json.dumps(response))


def handle_json(command):
    command_name = command.get("cmd", "")

    if command_name == "ping":
        send_response({
            "ok": True,
            "message": "ESP32 ready"
        })
        return

    if command_name == "home":
        move_smooth([90, 90, 90, 90, 90, 90])

        send_response({
            "ok": True,
            "angles": current_angles
        })
        return

    if command_name == "move_to":
        try:
            target = inverse_kinematics(
                command["x"],
                command["y"],
                command["z"]
            )

            move_smooth(target)

            send_response({
                "ok": True,
                "angles": current_angles
            })

        except Exception as error:
            send_response({
                "ok": False,
                "error": str(error)
            })

        return

    if command_name == "grip":
        value = float(command.get("angle", 0))
        target = current_angles[:]

        # serial_comm.py:
        # 음수 = 열기, 양수 = 닫기
        if value < 0:
            target[5] = config.SERVO[5]["min_deg"]
        else:
            target[5] = config.SERVO[5]["max_deg"]

        move_smooth(target)

        send_response({
            "ok": True,
            "angles": current_angles
        })
        return

    send_response({
        "ok": False,
        "error": "unknown command"
    })


def handle_line(line):
    line = line.strip()

    if not line:
        return

    # 기존 손 추적 각도 명령
    if line.startswith("A:"):
        try:
            values = [
                int(value)
                for value in line[2:].split(",")
            ]

            move_direct(values)

        except Exception:
            pass

        return

    # main_vision.py JSON 명령
    if line.startswith("{"):
        try:
            command = json.loads(line)
            handle_json(command)

        except Exception as error:
            send_response({
                "ok": False,
                "error": str(error)
            })


# ─────────────────────────────────────────
# 시작 위치
# ─────────────────────────────────────────

for servo_index, servo_name in enumerate(SERVO_ORDER):
    write_servo(servo_name, 90)
    sleep_ms(100)

print("READY")


# ─────────────────────────────────────────
# USB 시리얼 수신
# ─────────────────────────────────────────

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

while True:
    try:
        events = poller.poll(50)

        if events:
            received = sys.stdin.readline()
            handle_line(received)

    except KeyboardInterrupt:
        break

    except Exception:
        sleep_ms(100)
