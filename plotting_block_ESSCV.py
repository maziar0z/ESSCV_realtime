"""ESSCV display: target speed (mph) as a number inside a circle whose size grows with the target speed.
Based on plotting_block_May25.py (same port 10890, same 1 s update loop).

Color compares the target with the current speed:
  red    target ~0 (stop at the red light)
  green  target >= current - SLOW_MILD_MPH          (keep going)
  gold   target is SLOW_MILD..SLOW_STRONG mph below  (ease off)
  red    target is more than SLOW_STRONG mph below   (slow down)
"""
import math
import socket
import datetime
import pickle as pkl
from matplotlib import pyplot as plt

SLOW_MILD_MPH = 2.0
SLOW_STRONG_MPH = 8.0
MIN_SIZE, MAX_SIZE = 120, 330      # marker size range (points)
SIZE_FULL_MPH = 60.0               # target speed that gets MAX_SIZE

plt.ion()  # Turn on interactive mode

fig = plt.figure(figsize=(6, 6))
ax = plt.subplot(111)
plt.axis('off')
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.tight_layout()

HOST_REC = ""
PORT_REC = 10890

s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec.bind((HOST_REC, PORT_REC))
s_rec.setblocking(False)

current_time = datetime.datetime.now()
last_time = current_time


def unpack(message):
    """Display message from warning_block_ESSCV.py (dict); a bare number is treated as the target speed."""
    if isinstance(message, dict):
        return float(message["target_mph"]), message.get("speed_mph")
    return float(message), None


while True:
    current_time = datetime.datetime.now()

    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData = None

        keepReceiving = True
        while keepReceiving:
            try:
                data, fromAddr = s_rec.recvfrom(10048)
                if data:
                    newestData = data
            except socket.error as why:
                keepReceiving = False
                break

        try:
            target_mph, speed_mph = unpack(pkl.loads(newestData)) if newestData else (None, None)
        except Exception as e:   # bad packet: keep the last frame
            print('bad display message:', e)
            target_mph, speed_mph = None, None
        if target_mph is not None and not math.isfinite(target_mph):
            target_mph = None
        if speed_mph is not None and not math.isfinite(speed_mph):
            speed_mph = None

        if target_mph is not None:
            ax.clear()
            ax.axis('off')
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)

            print('target speed (mph)', round(target_mph, 1), ' current speed (mph)',
                  None if speed_mph is None else round(speed_mph, 1))

            below = 0.0 if speed_mph is None else speed_mph - target_mph
            if target_mph < 1.0:   # stop
                color = 'red'
            elif below <= SLOW_MILD_MPH:
                color = 'green'
            elif below <= SLOW_STRONG_MPH:
                color = 'gold'
            else:
                color = 'red'
            size = MIN_SIZE + (MAX_SIZE - MIN_SIZE) * min(max(target_mph, 0.0), SIZE_FULL_MPH) / SIZE_FULL_MPH

            plt.plot(0, 0, color, alpha=0.5, marker='o', markersize=size)[0]
            plt.text(0, 0.05, f"{target_mph:.0f}", ha='center', va='center', fontsize=size / 3, fontweight='bold')
            plt.text(0, -0.28 * size / MAX_SIZE, "mph", ha='center', va='center', fontsize=size / 9)
            plt.draw()
            plt.pause(0.01)
        else:
            print("Waiting for complete data...")
            plt.pause(0.01)
