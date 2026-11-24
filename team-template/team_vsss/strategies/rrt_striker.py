"""Striker that plans its way to the ball with RRT. The other robots use Basic.

Everything is in "my perspective" (I attack towards +x), like any strategy.
"""
import math
import random

from ..planning.rrt import distance, plan_rrt, segment_is_free, shortcut
from ..world import Command, World
from .basic import V_MAX, Basic, go_to

ROBOT_R = 0.045        # a 7.5 cm square robot, approximated as a circle (m)
BALL_R = 0.0215        # m
MARGIN = 0.01          # extra safety distance (m)
REPLAN_EVERY = 10      # replan at least every 10 steps (0.25 s)


class RRTStriker:
    def __init__(self):
        self.basic = Basic()
        self.rng = random.Random(0)    # fixed seed: same match, same result
        self.steps_since_plan = REPLAN_EVERY
        # What the last plan produced (the viewer draws these)
        self.path = None               # smoothed path, list of (x, y)
        self.tree_edges = []
        self.obstacles = []

    def decide(self, m: World) -> list[Command]:
        commands = [c for c in self.basic.decide(m) if c.id != 0]
        me = next((r for r in m.mine if r.id == 0), None)
        if me is None:
            return commands

        # Target: 8 cm behind the ball, on the line ball -> center of the rival goal
        b = m.ball
        dx, dy = m.length / 2 - b.x, -b.y
        d = math.hypot(dx, dy)
        ux, uy = dx / d, dy / d
        behind = (b.x - 0.08 * ux, b.y - 0.08 * uy)
        heading = math.degrees(math.atan2(uy, ux))
        here = (me.x, me.y)

        # Already behind the ball: push it, no planning needed
        if distance(here, behind) < 0.04:
            self.path = None
            commands.append(go_to(me, b.x + 0.10 * ux, b.y + 0.10 * uy, m.omni, look_degrees=heading))
            return commands

        obstacles = self._obstacles(m, here, behind)
        self.steps_since_plan += 1
        if (self.path is None
                or self.steps_since_plan >= REPLAN_EVERY
                or distance(self.path[-1], behind) > 0.05          # the target moved
                or not self._still_free(here, obstacles)):         # something got in the way
            self._plan(here, behind, obstacles, m)

        commands.append(self._follow(me, m.omni, heading))
        return commands

    # ------------------------------------------------------------------
    def _obstacles(self, m: World, start, goal):
        """Circles the robot's CENTER must not enter (already inflated)."""
        circles = [(r.x, r.y, 2 * ROBOT_R + MARGIN) for r in m.rivals]
        circles += [(r.x, r.y, 2 * ROBOT_R + MARGIN) for r in m.mine if r.id != 0]
        circles.append((m.ball.x, m.ball.y, ROBOT_R + BALL_R + MARGIN))
        # An obstacle that already contains the start or the goal would make
        # planning impossible: ignore it (we are already touching it).
        return [c for c in circles
                if distance(start, c[:2]) > c[2] and distance(goal, c[:2]) > c[2]]

    def _plan(self, start, goal, obstacles, m: World):
        bounds = (-m.length / 2 + ROBOT_R, m.length / 2 - ROBOT_R,
                  -m.width / 2 + ROBOT_R, m.width / 2 - ROBOT_R)
        result = plan_rrt(start, goal, obstacles, bounds, rng=self.rng)
        self.tree_edges = result.edges()
        self.obstacles = obstacles
        self.path = shortcut(result.path, obstacles) if result.path else [start, goal]
        self.steps_since_plan = 0

    def _still_free(self, here, obstacles):
        points = [here] + self.path[1:]
        return all(segment_is_free(a, b, obstacles) for a, b in zip(points, points[1:]))

    def _follow(self, me, omni, heading):
        """Goes to the next waypoint; drops the waypoints already reached."""
        while len(self.path) > 2 and distance((me.x, me.y), self.path[1]) < 0.04:
            self.path.pop(0)
        target = self.path[1] if len(self.path) > 1 else self.path[0]
        cmd = go_to(me, target[0], target[1], omni, look_degrees=heading)
        is_last = len(self.path) <= 2
        if omni and not is_last:
            # Intermediate waypoint: don't slow down near it, keep full speed
            ex, ey = target[0] - me.x, target[1] - me.y
            d = math.hypot(ex, ey)
            if d > 1e-6:
                cmd.vx, cmd.vy = ex / d * V_MAX, ey / d * V_MAX
        return cmd