"""Quick test of the omni robot inside rSim (only needs `robosim`).

Sends pure movements (forward, sideways, turn) and compares what the robot
did against what it should have done. Run it every time rSim's C++ changes.
"""
import math

import robosim

ANGLES = [60, 180, 300]        # must match OmniWheel*Angle in vssconfig.h
L = 0.0375 + 0.0025 / 2        # distance center -> wheel
R = 0.026                      # wheel radius
DT = 0.025                     # 25 ms per step


def wheels(vx, vy, w):
    """(vx, vy) in m/s in the robot's frame, w in rad/s -> rad/s for each wheel."""
    return [(L * w - vx * math.sin(math.radians(a)) + vy * math.cos(math.radians(a))) / R
            for a in ANGLES]


def check(name, vx, vy, w, theta0=0.0, steps=40, ramp=10):
    sim = robosim.VSS(0, 1, 0, int(DT * 1000), [0.6, 0.6, 0, 0], [[0.0, 0.0, theta0]], [],
                      omni_blue=True)
    th0 = sim.get_state()[7]
    for k in range(steps):
        # Why do we increase the speed gradually (a "ramp") instead of all at once?
        f = min(1.0, (k + 1) / ramp)
        sim.step([wheels(vx * f, vy * f, w * f)])
    x, y, th = sim.get_state()[5:8]
    turn = (th - th0 + 180) % 360 - 180
    # Expected: speed x time (discounting the ramp), in the field's frame
    time = sum(min(1.0, (k + 1) / ramp) for k in range(steps)) * DT
    t = math.radians(theta0)
    ex = (vx * math.cos(t) - vy * math.sin(t)) * time
    ey = (vx * math.sin(t) + vy * math.cos(t)) * time
    print(f"{name:24s} did: dx={x:+.2f} dy={y:+.2f} turn={turn:+6.1f}°   "
          f"expected: dx={ex:+.2f} dy={ey:+.2f} turn={math.degrees(w * time):+6.1f}°")


check("forward", 0.5, 0.0, 0.0)
check("sideways (left)", 0.0, 0.5, 0.0)
check("turn in place", 0.0, 0.0, 2.0, steps=20)
check("forward, facing +y", 0.5, 0.0, 0.0, theta0=90)
print("\nDifferences of 1-2 cm or a few degrees are normal (motors take time to reach speed).")
