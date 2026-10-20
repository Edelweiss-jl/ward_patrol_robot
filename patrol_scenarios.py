"""各控制器共享的无初始重叠随机场景；只改变初始条件，不改变运动规则。"""
import math
import numpy as np
import pybullet as p
from complex_ward_demo import RADIUS, MARGIN


def initialize_episode(sim, seed=None, beds=None, rng=None):
    rng = np.random.default_rng(seed) if rng is None else rng
    if beds is None:
        sim.tasks = [sim.tasks[i] for i in rng.permutation(len(sim.tasks))]
    placed = []
    for actor in sim.ward.actors:
        for _ in range(128):
            fraction = float(rng.uniform(.1,.9))
            xy = tuple((1-fraction)*a+fraction*b for a,b in zip(*actor.route[:2]))
            if (sim.free(xy,False,actor.radius+.05)
                    and all(math.dist(xy,r.xy)>RADIUS+MARGIN+actor.radius+.04 for r in sim.robots)
                    and all(math.dist(xy,a.xy)>a.radius+actor.radius+.08 for a in placed)):
                break
        else:
            raise ValueError("Unable to generate a collision-free initial scene")
        actor.xy = xy
        actor.speed *= float(rng.uniform(.85,1.15))
        p.resetBasePositionAndOrientation(actor.body,[*actor.xy,.45 if actor.cart else .77],[0,0,0,1])
        placed.append(actor)
    return dict(scenario=sim.ward.kind,beds=[t.bed_id for t in sim.tasks],
                robots=[r.xy for r in sim.robots],
                people=[dict(xy=a.xy,speed=a.speed,radius=a.radius) for a in sim.ward.actors])
