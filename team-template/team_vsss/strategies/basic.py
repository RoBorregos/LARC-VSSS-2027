"""Example strategy: striker, defender and goalkeeper.

Everything is from "my perspective": I always attack towards +x,
my own goal is at x = -length/2.
"""
import math

from ..world import Command, Thing, World

V_MAX = 0.8      # m/s
W_MAX = 8.0      # rad/s
# What does a proportional controller do, and what happens if KP is too big?
KP = 4.0         # position gain: (m/s) for each meter of error
KP_ANG = 6.0     # angle gain: (rad/s) for each radian of error


def angle_error(desired_degrees, current_degrees):
    """Difference between two angles, in radians, in the range [-pi, pi]."""
    # Why do we add 180, take % 360 and then subtract 180?
    d = (desired_degrees - current_degrees + 180) % 360 - 180
    return math.radians(d)


def go_to(robot: Thing, x, y, omni: bool, look_degrees=None) -> Command:
    """Proportional controller: speed proportional to the distance to the target."""
    ex, ey = x - robot.x, y - robot.y
    dist = math.hypot(ex, ey)
    v = min(V_MAX, KP * dist)
    vx, vy = (ex / dist * v, ey / dist * v) if dist > 1e-3 else (0.0, 0.0)

    if omni:
        # An omni robot can move in one direction while looking in another.
        target = 0.0 if look_degrees is None else look_degrees
        w = KP_ANG * angle_error(target, robot.theta)
    else:
        # A differential robot must point towards where it goes. It can drive
        # backwards, so we choose the direction that needs less turning.
        # Why does a differential robot need to turn before moving?
        heading = math.degrees(math.atan2(ey, ex))
        err = angle_error(heading, robot.theta)
        if abs(err) > math.pi / 2:
            err = angle_error(heading + 180, robot.theta)
        w = KP_ANG * err
        # If it isn't aligned yet, it moves slower (avoids spiral paths).
        brake = max(0.0, math.cos(err))
        vx, vy = vx * brake, vy * brake

    w = max(-W_MAX, min(W_MAX, w))
    return Command(robot.id, vx, vy, w)


class Basic:
    def decide(self, m: World) -> list[Command]:
        commands = []
        rival_goal = (m.length / 2, 0.0)
        my_goal = (-m.length / 2, 0.0)
        b = m.ball

        # Aim at the side of the goal that the rival goalkeeper leaves free
        # (the rival closest to its own goal).
        # What does min(..., key=lambda ...) do?
        keeper = min(m.rivals, key=lambda r: abs(r.x - rival_goal[0]), default=None)
        y_shot = 0.12 if (keeper is None or keeper.y < 0) else -0.12

        for r in m.mine:
            if r.id == 0:      # --- striker: gets behind the ball and pushes ---
                dx, dy = rival_goal[0] - b.x, y_shot - b.y
                d = math.hypot(dx, dy)
                ux, uy = dx / d, dy / d                     # direction ball -> rival goal
                # What is the 'behind' point, and why 8 cm?
                behind = (b.x - 0.08 * ux, b.y - 0.08 * uy)
                if math.hypot(r.x - behind[0], r.y - behind[1]) < 0.05:
                    target = (b.x + 0.10 * ux, b.y + 0.10 * uy)   # aligned: push!
                else:
                    target = behind
                heading = math.degrees(math.atan2(uy, ux))
                commands.append(go_to(r, *target, m.omni, look_degrees=heading))

            elif r.id == 1:    # --- defender: between the ball and my goal, not past midfield ---
                x = my_goal[0] + 0.4 * (b.x - my_goal[0])
                y = my_goal[1] + 0.4 * (b.y - my_goal[1])
                commands.append(go_to(r, min(x, -0.1), y, m.omni))

            else:              # --- goalkeeper: follows the ball's y in front of the goal ---
                y = max(-m.goal_width / 2, min(m.goal_width / 2, b.y))
                commands.append(go_to(r, my_goal[0] + 0.07, y, m.omni, look_degrees=90))

        return commands
