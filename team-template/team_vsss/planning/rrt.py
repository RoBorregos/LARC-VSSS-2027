"""RRT (Rapidly-exploring Random Tree) on the field plane.

The robot is treated as a POINT; obstacles are circles already "inflated"
by the robot's size.
All positions are (x, y) tuples in meters.
"""
import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

Point = Tuple[float, float]
Circle = Tuple[float, float, float]          # (x, y, radius)


@dataclass
class RRTResult:
    path: Optional[List[Point]]              # None if no path was found
    nodes: List[Point] = field(default_factory=list)
    parents: List[int] = field(default_factory=list)
    iterations: int = 0

    def edges(self):
        """Tree edges as (point, point) pairs, for drawing."""
        return [(self.nodes[p], self.nodes[i])
                for i, p in enumerate(self.parents) if p >= 0]


# ---------------------------------------------------------------- geometry
def distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def point_segment_distance(c: Point, a: Point, b: Point) -> float:
    """Shortest distance from point c to the segment a-b."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    length2 = abx * abx + aby * aby
    if length2 == 0.0:
        return distance(c, a)
    # t = how far along the segment the closest point is (0 = at a, 1 = at b)
    t = ((c[0] - a[0]) * abx + (c[1] - a[1]) * aby) / length2
    t = max(0.0, min(1.0, t))
    closest = (a[0] + t * abx, a[1] + t * aby)
    return distance(c, closest)


def segment_is_free(a: Point, b: Point, obstacles: List[Circle]) -> bool:
    """True if the straight segment a-b doesn't touch any obstacle."""
    return all(point_segment_distance((ox, oy), a, b) > r for ox, oy, r in obstacles)


def inside(p: Point, bounds) -> bool:
    xmin, xmax, ymin, ymax = bounds
    return xmin <= p[0] <= xmax and ymin <= p[1] <= ymax


def steer(from_p: Point, to_p: Point, step: float) -> Point:
    """Moves from from_p towards to_p, at most 'step' meters."""
    d = distance(from_p, to_p)
    if d <= step:
        return to_p
    k = step / d
    return (from_p[0] + k * (to_p[0] - from_p[0]), from_p[1] + k * (to_p[1] - from_p[1]))

# ---------------------------------------------------------------- RRT
def plan_rrt(start: Point, goal: Point, obstacles: List[Circle], bounds,
             step=0.06, goal_bias=0.1, max_iters=1500, rng=random) -> RRTResult:
    """Grows a tree from start until it reaches the goal (or gives up)."""
    xmin, xmax, ymin, ymax = bounds
    nodes = [start]
    parents = [-1]    # the root has no parent
    pts = np.empty((max_iters + 2, 2))   # same nodes, as a numpy table
    pts[0] = start

    # Trivial case: nothing in between
    if segment_is_free(start, goal, obstacles):
        return RRTResult([start, goal], [start, goal], [-1, 0], 0)

    for it in range(1, max_iters + 1):
        # 1. SAMPLE: a random point (sometimes, the goal itself)
        if rng.random() < goal_bias:
            sample = goal
        else:
            sample = (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))

        # 2. NEAREST: the tree node closest to the sample
        n = len(nodes)
        d2 = ((pts[:n] - sample) ** 2).sum(axis=1)
        i_near = int(d2.argmin())
        near = nodes[i_near]

        # 3. STEER: a new point, at most 'step' meters from 'near'
        new = steer(near, sample, step)

        # 4. CHECK: inside the field and no collisions on the way
        if not inside(new, bounds) or not segment_is_free(near, new, obstacles):
            continue

        # 5. ADD the new node to the tree
        nodes.append(new)
        parents.append(i_near)
        pts[n] = new

        # 6. GOAL? if the goal can be reached in a straight line, finish
        if distance(new, goal) <= step and segment_is_free(new, goal, obstacles):
            nodes.append(goal)
            parents.append(n)
            return RRTResult(_extract_path(nodes, parents), nodes, parents, it)

    return RRTResult(None, nodes, parents, max_iters)

def _extract_path(nodes, parents) -> List[Point]:
    """Walks from the last node (the goal) back to the root using the parents."""
    path = []
    i = len(nodes) - 1
    while i != -1:
        path.append(nodes[i])
        i = parents[i]
    path.reverse()
    return path

# ---------------------------------------------------------------- smoothing
def shortcut(path: List[Point], obstacles: List[Circle]) -> List[Point]:
    """Removes unnecessary waypoints: from each point, jumps to the
    farthest point that can be reached in a straight line."""
    if path is None or len(path) <= 2:
        return path
    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not segment_is_free(path[i], path[j], obstacles):
            j -= 1
        result.append(path[j])
        i = j
    return result

def plan_rrt_star():
    return