"""
Bridge between rSoccer and our modified version of rSim
with omnidirectional robots.

What modifications does this add to the original rSim?
"""
import numpy as np
import robosim
from rsoccer_gym.Simulators.rsim import RSimVSS


class RSimVSSMixed(RSimVSS):
    """
    Subclass of RSimVSS (rSoccer), where each team can have
    either omnidirectional or differential robots.
    """

    def __init__(self, *args, omni_blue=False, omni_yellow=False, **kwargs):
        # Robots are differential by default.
        # These must be saved BEFORE super().__init__(), because that constructor
        # calls _init_simulator(), which needs them.
        self.omni_blue = omni_blue
        self.omni_yellow = omni_yellow
        super().__init__(*args, **kwargs)

    def _init_simulator(self, field_type, n_robots_blue, n_robots_yellow,
                        ball_pos, blue_robots_pos, yellow_robots_pos, time_step_ms):
        return robosim.VSS(
            field_type, n_robots_blue, n_robots_yellow, time_step_ms,
            ball_pos, blue_robots_pos, yellow_robots_pos,
            omni_blue=self.omni_blue, omni_yellow=self.omni_yellow,
        )

    def send_commands(self, commands):
        # One row per robot (blues first) and 3 columns: omni robots use all 3,
        # differential robots only the first 2 (rSim ignores the third one).
        cmds = np.zeros((self.n_robots_blue + self.n_robots_yellow, 3))
        for c in commands:
            # Why add a conditional here, and how does if/else work in a single
            # line?
            row = self.n_robots_blue + c.id if c.yellow else c.id
            cmds[row] = [c.v_wheel0, c.v_wheel1, c.v_wheel2]
        self.simulator.step(cmds)
