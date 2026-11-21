"""
Robot dimensions and kinematics.
Translates a desired movement into how fast each wheel must spin.

These constants MUST match the ones in rSim/src/robosim/vssconfig.h.
If you change them in the simulator (and recompile), change them here too.
"""

"""
TASKS:
- Make an official documentation of the kinematics behind the simulation
"""
import math

# --- Dimensions [m] ---
WHEEL_RADIUS = 0.026  # WheelRadius
# Why are the two radii different?
CENTRAL_DIST_WHEEL = 0.0375 + 0.0025 / 2  # Radius + WheelThickness/2
MAX_RPM = 440  # WheelMotorMaxRPM
MAX_W_WHEEL = MAX_RPM * 2 * math.pi / 60  # max wheel speed [rad/s] (~46)

# --- Omnidirectional robot ---
# Are these angles the same as in our real robot?
ANGLES_OMNI = (60, 180, 300)  # OmniWheel0/1/2Angle, degrees from the front


def omni_wheels(vx, vy, w):
    """
    Inverse omni kinematics.
    vx, vy: desired velocity [m/s], in the ROBOT's frame (x = robot's front)
    w: angular velocity [rad/s] (positive = counterclockwise)
    Returns [w0, w1, w2] in rad/s, in the order expected by rSim
    """
    wheels = []
    for ang in ANGLES_OMNI:
        a = math.radians(ang)
        # How does this work?
        v_wheel = CENTRAL_DIST_WHEEL * w - vx * math.sin(a) + vy * math.cos(a)
        wheels.append(v_wheel / WHEEL_RADIUS)
    return saturate(wheels)


# --- Classic differential robot ---
# Why do we define the two kinds of movement?
def differential_wheels(v, w):
    """
    v: forward speed [m/s]
    w: angular velocity [rad/s]
    Returns [left, right] in rad/s
    """
    left = (v - w * CENTRAL_DIST_WHEEL) / WHEEL_RADIUS
    right = (v + w * CENTRAL_DIST_WHEEL) / WHEEL_RADIUS
    return saturate([left, right])


def saturate(wheels):
    """
    If some wheel needs more speed than the motor can give, scales
    EVERY wheel by the same factor.

    Scaling all of them (instead of only cropping the one that is too big)
    keeps the direction of movement.

    As a result the robot goes slower, but it doesn't lose its direction.
    """
    # How does this single-line for work?
    maximum = max(abs(w) for w in wheels)

    if maximum > MAX_W_WHEEL:
        factor = MAX_W_WHEEL / maximum
        wheels = [w * factor for w in wheels]
    return wheels


def global_to_local(vx_g, vy_g, theta_degrees):
    """Rotates a velocity from the FIELD's frame to the ROBOT's frame."""
    t = math.radians(theta_degrees)
    vx = vx_g * math.cos(t) + vy_g * math.sin(t)
    vy = -vx_g * math.sin(t) + vy_g * math.cos(t)
    return vx, vy
