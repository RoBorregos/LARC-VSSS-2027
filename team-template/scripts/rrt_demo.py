"""Watch RRT grow, without the simulator.

    python scripts/rrt_demo.py            # SPACE: new random scenario, ESC: quit
    python scripts/rrt_demo.py --star     # RRT* instead of RRT
    python scripts/rrt_demo.py --save rrt.png --seed 3   # no window: saves an image
"""
import argparse
import random

import pygame

from team_vsss.planning.rrt import plan_rrt, plan_rrt_star, shortcut, distance

BOUNDS = (-0.70, 0.70, -0.60, 0.60)    # where the robot's center can be (m)
SCALE = 500                            # pixels per meter
W, H = 800, 700
COLORS = {"bg": (22, 90, 50), "obstacle": (200, 70, 50), "tree": (170, 220, 170),
          "path": (250, 200, 60), "short": (90, 170, 255), "start": (255, 255, 255),
          "goal": (255, 140, 0), "line": (235, 235, 235)}


def to_px(p):
    """Meters (y up) -> pixels (y down)."""
    return int(W / 2 + p[0] * SCALE), int(H / 2 - p[1] * SCALE)


def random_scenario(rng):
    start, goal = (-0.62, rng.uniform(-0.5, 0.5)), (0.62, rng.uniform(-0.5, 0.5))
    obstacles = []
    while len(obstacles) < 7:
        o = (rng.uniform(-0.45, 0.45), rng.uniform(-0.5, 0.5), rng.uniform(0.07, 0.13))
        if distance(start, o[:2]) > o[2] + 0.05 and distance(goal, o[:2]) > o[2] + 0.05:
            obstacles.append(o)
    return start, goal, obstacles


def draw(screen, start, goal, obstacles, result, n_edges, smooth):
    screen.fill(COLORS["bg"])
    xmin, xmax, ymin, ymax = BOUNDS
    x0, y0 = to_px((xmin, ymax))
    x1, y1 = to_px((xmax, ymin))
    pygame.draw.rect(screen, COLORS["line"], (x0, y0, x1 - x0, y1 - y0), 1)
    for ox, oy, r in obstacles:
        pygame.draw.circle(screen, COLORS["obstacle"], to_px((ox, oy)), int(r * SCALE))
    for a, b in result.edges()[:n_edges]:                       # the tree, growing
        pygame.draw.line(screen, COLORS["tree"], to_px(a), to_px(b), 1)
    if result.path and n_edges >= len(result.nodes) - 1:        # tree complete: show paths
        pygame.draw.lines(screen, COLORS["path"], False, [to_px(p) for p in result.path], 3)
        pygame.draw.lines(screen, COLORS["short"], False, [to_px(p) for p in smooth], 3)
    pygame.draw.circle(screen, COLORS["start"], to_px(start), 7)
    pygame.draw.circle(screen, COLORS["goal"], to_px(goal), 7)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--star", action="store_true", help="use RRT* (keeps improving the path)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--save", help="don't open a window: save the final image here")
    args = ap.parse_args()
    rng = random.Random(args.seed)

    def new_plan():
        start, goal, obstacles = random_scenario(rng)
        planner = plan_rrt_star if args.star else plan_rrt
        kwargs = {"max_iters": 800} if args.star else {}
        result = planner(start, goal, obstacles, BOUNDS, rng=rng, **kwargs)
        smooth = shortcut(result.path, obstacles) if result.path else None
        info = (f"{'RRT*' if args.star else 'RRT'}: {len(result.nodes)} nodes, "
                + (f"path {sum(distance(a, b) for a, b in zip(result.path, result.path[1:])):.2f} m"
                   f" -> shortcut {sum(distance(a, b) for a, b in zip(smooth, smooth[1:])):.2f} m"
                   if result.path else "no path"))
        return start, goal, obstacles, result, smooth, info

    pygame.init()
    if args.save:
        screen = pygame.Surface((W, H))
        start, goal, obstacles, result, smooth, info = new_plan()
        draw(screen, start, goal, obstacles, result, len(result.nodes), smooth)
        pygame.image.save(screen, args.save)
        print(info)
        return

    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()
    start, goal, obstacles, result, smooth, info = new_plan()
    pygame.display.set_caption(info)
    shown = 0
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                start, goal, obstacles, result, smooth, info = new_plan()
                pygame.display.set_caption(info)
                shown = 0
        shown += 3                                    # reveal 3 edges per frame
        draw(screen, start, goal, obstacles, result, shown, smooth)
        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()