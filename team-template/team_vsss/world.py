"""
What a strategy 'sees' and what it 'responds'.

Key detail: each team sees the field AS IF it attacked towards +x.
This lets us write a single strategy that works for both teams.
For the yellow team we rotate the field 180°: (x, y) -> (-x, -y) and
theta -> theta + 180.
(A rotation, not a mirror: a mirror would invert the robots' turning direction.)
"""

# What do these libraries do?
from dataclasses import dataclass
from typing import List


@dataclass
class Thing:
    """
    A robot or the ball, seen from the team's perspective.
    """
    x: float             # m
    y: float             # m
    theta: float = 0.0   # degrees (the ball doesn't have one)
    vx: float = 0.0      # m/s
    vy: float = 0.0      # m/s
    w: float = 0.0       # degrees/s  (careful: Command.w is in rad/s)
    id: int = 0


@dataclass
class World:
    ball: Thing
    mine: List[Thing]
    rivals: List[Thing]
    length: float        # field length (m)
    width: float         # field width (m)
    goal_width: float    # m
    omni: bool           # Are my robots omnidirectional?
    time: float = 0.0    # seconds since the match began


@dataclass
class Command:
    """
    Order for one robot, in the FIELD's frame, from my team's perspective.
    vx, vy: m/s towards where I want it to go
    w: rad/s (positive = counterclockwise)
    A differential robot can't move sideways: it only uses the part of (vx, vy)
    that points to its front. That's why its strategy must turn it using w.
    """
    id: int
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0


# What does the : after the parameter do?
def _rotate(thing: Thing) -> Thing:
    return Thing(-thing.x, -thing.y, (thing.theta + 180) % 360,
                 -thing.vx, -thing.vy, thing.w, thing.id)


# What is the -> for?
def build_world(frame, field, im_yellow: bool, omni: bool, time: float) -> World:
    """
    Converts an rSoccer Frame into the World that a team sees.
    """
    # What does this helper function do?
    def robot_thing(r):
        return Thing(r.x, r.y, r.theta, r.v_x, r.v_y, r.v_theta, r.id)

    ball = Thing(frame.ball.x, frame.ball.y, 0, frame.ball.v_x, frame.ball.v_y)
    blues = [robot_thing(r) for r in frame.robots_blue.values()]
    yellows = [robot_thing(r) for r in frame.robots_yellow.values()]

    if im_yellow:
        ball = _rotate(ball)
        mine = [_rotate(r) for r in yellows]
        rivals = [_rotate(r) for r in blues]
    else:
        mine, rivals = blues, yellows
    return World(ball, mine, rivals, field.length, field.width, field.goal_width,
                 omni, time)


def command_to_field(c: Command, im_yellow: bool) -> Command:
    """Undoes the rotation: converts a command from my team's perspective
    to the real field."""
    if im_yellow:
        return Command(c.id, -c.vx, -c.vy, c.w)
    return c
