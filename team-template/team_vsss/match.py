"""
A VSSS match, ready to watch on screen.

Inherits from VSSBaseEnv (rSoccer): from there we get the simulator, the field
drawing and the reset()/step() structure from Gymnasium.
We only decide how orders are translated, when a goal happens, and the initial
position of each robot.
"""

import math

import pygame
from rsoccer_gym.Entities import Ball, Frame, Robot
from rsoccer_gym.vss.vss_gym_base import VSSBaseEnv

from . import robot as rb
from .simulator import RSimVSSMixed
from .world import build_world, command_to_field


class MatchVSS(VSSBaseEnv):
    # rSoccer draws at 60 frames/s, but each step simulates 25 ms (= 40 steps/s).
    # With 40 the match is shown at real speed.
    # What is the ** doing inside the curly braces?
    metadata = {**VSSBaseEnv.metadata, "render_fps": 40}

    def __init__(self, n_blue=3, n_yellow=3, omni_blue=True, omni_yellow=False,
                 duration_s=300, max_accel=3.0, render_mode=None):
        super().__init__(field_type=0, n_robots_blue=n_blue, n_robots_yellow=n_yellow,
                         time_step=0.025, render_mode=render_mode)

        # VSSBaseEnv already created a "normal" simulator (differential robots only).
        # We throw it away and create ours, which supports omni robots.
        # Why can't we just use the simulator that VSSBaseEnv created?
        self.rsim.stop()
        self.rsim = RSimVSSMixed(field_type=0, n_robots_blue=n_blue,
                                 n_robots_yellow=n_yellow, time_step_ms=25,
                                 omni_blue=omni_blue, omni_yellow=omni_yellow)
        self.field = self.rsim.get_field_params()

        self.omni = {"blue": omni_blue, "yellow": omni_yellow}
        self.max_steps = int(duration_s / self.time_step)
        self.max_accel = max_accel      # m/s^2: imitates the robot's low-level control
        self.total_steps = 0
        self.goals = {"blue": 0, "yellow": 0}
        self.event = None               # 'goal_blue', 'goal_yellow', 'ball_stuck' or None
        # What is a dictionary, and why is its key a pair (team, id)?
        self._prev_vel = {}             # (team, id) -> (vx, vy) of the previous step
        self._ball_ref = None           # (x, y, step): last time the ball moved

    # ---------- used by the strategies ----------
    def world(self, team):
        """What the team ('blue' or 'yellow') sees, always attacking towards +x."""
        return build_world(self.frame, self.field, team == "yellow",
                           self.omni[team], self.total_steps * self.time_step)

    # ---------- the 4 methods that VSSBaseEnv asks us for ----------
    def _get_commands(self, actions):
        """actions = {'blue': [Command, ...], 'yellow': [Command, ...]}"""
        orders = []
        for team, commands in actions.items():
            yellow = team == "yellow"
            robots = self.frame.robots_yellow if yellow else self.frame.robots_blue
            for c in commands:
                c = command_to_field(c, yellow)                                 # 1. undo the rotation
                vx_g, vy_g = self._limit_acceleration(team, c.id, c.vx, c.vy)   # 2. smooth it
                # Why do we need the robot's theta to convert the command?
                theta = robots[c.id].theta
                vx, vy = rb.global_to_local(vx_g, vy_g, theta)                  # 3. to the robot's frame
                if self.omni[team]:
                    w0, w1, w2 = rb.omni_wheels(vx, vy, c.w)                    # 4. kinematics
                else:
                    w0, w1 = rb.differential_wheels(vx, c.w)   # ignores vy: it can't move sideways
                    w2 = 0.0
                orders.append(Robot(yellow=yellow, id=c.id,
                                    v_wheel0=w0, v_wheel1=w1, v_wheel2=w2))
        return orders

    def _frame_to_observations(self):
        return self.frame          # to watch matches, the raw Frame is enough

    def _calculate_reward_and_done(self):
        """Decides if the play is over. (The reward only matters for RL: here it is 0.)"""
        half = self.field.length / 2
        b = self.frame.ball
        self.event = None
        if b.x > half:
            self.goals["blue"] += 1
            self.event = "goal_blue"
        elif b.x < -half:
            self.goals["yellow"] += 1
            self.event = "goal_yellow"
        elif self._ball_stuck(b):
            self.event = "ball_stuck"
        return 0.0, self.event is not None

    def _ball_stuck(self, b, seconds=10.0, tolerance=0.03):
        """True if the ball has not moved more than 'tolerance' meters in 'seconds'."""
        if self._ball_ref is None or math.hypot(b.x - self._ball_ref[0],
                                                b.y - self._ball_ref[1]) > tolerance:
            self._ball_ref = (b.x, b.y, self.steps)
            return False
        return (self.steps - self._ball_ref[2]) * self.time_step > seconds

    def _get_initial_positions_frame(self):
        """Kick-off from the center: blues on the left, yellows on the right."""
        f = Frame()
        f.ball = Ball(x=0.0, y=0.0)
        # striker, defender, goalkeeper. The striker is shifted in y so that,
        # when the formation is mirrored, both strikers are not on the same line.
        formation = [(-0.20, 0.10), (-0.40, -0.25), (-0.65, 0.0)]
        for i in range(self.n_robots_blue):
            x, y = formation[i]
            f.robots_blue[i] = Robot(x=x, y=y, theta=0)
        for i in range(self.n_robots_yellow):
            x, y = formation[i]
            # Why do yellow robots start with theta=180?
            f.robots_yellow[i] = Robot(x=-x, y=-y, theta=180)
        return f

    # ---------- extras ----------
    def step(self, actions):
        self.total_steps += 1
        obs, reward, play_over, _, _ = super().step(actions)
        time_over = self.total_steps >= self.max_steps
        return obs, reward, play_over, time_over, {"event": self.event}

    def reset(self, *, seed=None, options=None):
        self._prev_vel = {}
        self._ball_ref = None
        return super().reset(seed=seed, options=options)

    def _limit_acceleration(self, team, rid, vx, vy):
        """Doesn't let the velocity change more than max_accel * dt in one step.

        Without this, a sudden start saturates the motors and the omni robot
        twists (see the guide, calibration section).
        """
        # What would happen to the robot if we removed this function?
        vx0, vy0 = self._prev_vel.get((team, rid), (0.0, 0.0))
        dvx, dvy = vx - vx0, vy - vy0
        change = math.hypot(dvx, dvy)
        maximum = self.max_accel * self.time_step
        if change > maximum:
            dvx, dvy = dvx * maximum / change, dvy * maximum / change
        new = (vx0 + dvx, vy0 + dvy)
        self._prev_vel[(team, rid)] = new
        return new

    def _render(self):
        super()._render()     # field, robots and ball (rSoccer)
        # rSoccer draws +y DOWNWARDS (in pygame, the screen's y grows downwards).
        # We flip the image so it matches the simulator: +y up, counterclockwise
        # turns look counterclockwise. The scoreboard is drawn afterwards so it
        # is not upside down.
        flipped = pygame.transform.flip(self.window_surface, False, True)
        self.window_surface.blit(flipped, (0, 0))
        if not pygame.font.get_init():
            pygame.font.init()
        font = pygame.font.Font(None, 32)
        t = int(self.total_steps * self.time_step)
        # What do // and % compute here?
        text = f"Blue {self.goals['blue']} - {self.goals['yellow']} Yellow    {t // 60}:{t % 60:02d}"
        self.window_surface.blit(font.render(text, True, (255, 255, 255)), (12, 10))
