"""Watch a match on screen.

Examples:
    python scripts/view_match.py
    python scripts/view_match.py --blue basic --yellow still
    python scripts/view_match.py --yellow-omni --duration 60
    python scripts/view_match.py --no-window --duration 120   # fast, scoreboard only
"""
# What is argparse for?
import argparse

from team_vsss.match import MatchVSS
from team_vsss.strategies import STRATEGIES

p = argparse.ArgumentParser()
p.add_argument("--blue", default="basic", choices=STRATEGIES)
p.add_argument("--yellow", default="basic", choices=STRATEGIES)
p.add_argument("--blue-differential", action="store_true", help="blue robots with 2 wheels")
p.add_argument("--yellow-omni", action="store_true", help="yellow robots with 3 omni wheels")
p.add_argument("--duration", type=float, default=300, help="match seconds")
p.add_argument("--no-window", action="store_true", help="doesn't draw; runs as fast as possible")
args = p.parse_args()

env = MatchVSS(omni_blue=not args.blue_differential, omni_yellow=args.yellow_omni,
               duration_s=args.duration,
               render_mode=None if args.no_window else "human")
blue = STRATEGIES[args.blue]()
yellow = STRATEGIES[args.yellow]()

env.reset()
time_over = False
# What do try / except / finally do here?
try:
    while not time_over:
        actions = {
            "blue": blue.decide(env.world("blue")),
            "yellow": yellow.decide(env.world("yellow")),
        }
        _, _, play_over, time_over, info = env.step(actions)
        if play_over:
            print(f"{info['event']}  ->  {env.goals}")
            # Why do we call env.reset() after every goal?
            env.reset()        # back to kick-off (the score is kept)
except KeyboardInterrupt:
    pass
finally:
    print("Final:", env.goals)
    env.close()
