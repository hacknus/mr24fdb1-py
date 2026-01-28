from mr24fdb1 import MR24FDB1Radar
import time

radar = MR24FDB1Radar(port="/dev/ttyAMA10", baudrate=9600)

print("Ready")
time.sleep(1.5)

while True:
    radar.read_frame()
    msg = radar.get_message()

    if msg:
        result = radar.parse_bodysign(msg, move_min=1, move_max=15)
        if result:
            print(result)