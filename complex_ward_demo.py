"""两个复杂病区共用的独立仿真代码，不导入或修改原始 demo。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import heapq
import json
import math
from pathlib import Path
import struct
import time
import zlib

import pybullet as p

# 长度和速度使用米、米/秒；规划时额外留出安全距离。
RADIUS = 0.28
MARGIN = 0.16
# 网格越小，路线越细，但寻路需要检查的点也越多。
GRID = 0.25
DT = 1 / 60


# 场景中的矩形障碍：x、y 是中心位置，sx、sy 是完整宽度。
@dataclass
class Box:
    x: float
    y: float
    sx: float
    sy: float
    height: float
    color: tuple
    name: str = ""
    body: int = -1


# 移动人员或推车沿 route 循环移动；body 保存对应的仿真物体编号。
@dataclass
class Actor:
    route: tuple
    speed: float
    radius: float = 0.26
    cart: bool = False
    xy: tuple = field(init=False)
    target: int = 1
    body: int = -1
    waiting: float = 0

    def __post_init__(self):
        self.xy = self.route[0]


# 病区布局只保存场景数据，具体建模和运动由 Simulation 负责。
class Ward:
    def __init__(self, kind):
        self.kind = kind
        self.hx, self.hy = (9, 7) if kind == "corridor" else (11, 8)
        # 障碍、装饰和巡检点分开保存；装饰不参与路径规划。
        self.boxes = []
        self.decor = []
        self.stops = []
        self.labels = []
        self.actors = []
        self.start = (-8, 0) if kind == "corridor" else (-9.5, 0)
        wall = (0.69, 0.77, 0.80, 1)
        for y in (-self.hy, self.hy):
            self.add(0, y, self.hx * 2, .16, 1.7, wall)
        for x in (-self.hx, self.hx):
            self.add(x, 0, .16, self.hy * 2, 1.7, wall)
        if kind == "corridor":
            self.corridor()
        else:
            self.ring()

    def add(self, x, y, sx, sy, h, color=(.65, .72, .76, 1), name=""):
        box = Box(x, y, sx, sy, h, color, name)
        self.boxes.append(box)
        return box

    def detail(self, x, y, z, sx, sy, sz, color):
        self.decor.append(((x, y, z), (sx, sy, sz), color))

    def bed(self, x, y, name, stop):
        self.add(x, y, 1.05, 2.1, .58, (.22, .43, .59, 1), name)
        self.detail(x, y, .66, 1, 1.95, .16, (.80, .93, .97, 1))
        self.detail(x, y + .65, .78, .80, .43, .12, (1, 1, .95, 1))
        for dy in (-1, 1):
            self.detail(x, y + dy, .84, 1.05, .07, .50, (.80, .84, .87, 1))
        self.stops.append((name, stop))
        self.labels.append((name, (x, y, 1.20)))

    # 将正面墙分成门口左右两段，为机器人留出通道。
    def front(self, left, right, y, door, width=1.9):
        for lo, hi in ((left, door - width / 2), (door + width / 2, right)):
            self.add((lo + hi) / 2, y, hi - lo, .16, 1.55)
        self.detail(door, y, .015, width, .25, .025, (.28, .73, .70, 1))

    def corridor(self):
        for sign in (1, -1):
            for col, x in enumerate((-6, 0, 6)):
                room = ("N" if sign > 0 else "S") + str(col + 1)
                self.front(x - 3, x + 3, sign * 2, x)
                self.labels.append(("ROOM " + room, (x, sign * 2.6, .1)))
                if col < 2:
                    self.add(x + 3, sign * 4.5, .16, 5, 1.55)
                for j, dx in enumerate((-1.65, 1.65)):
                    self.bed(x + dx, sign * 5.3, room + "-" + str(j + 1),
                             (x + dx, sign * 3.6))
                    self.add(x + dx + .85, sign * 5.9, .48, .55, .75,
                             (.73, .60, .43, 1), "cabinet")
                    self.add(x + dx + .85, sign * 4.8, .30, .30, .10,
                             (.55, .60, .65, 1), "IV base")
                    self.detail(x + dx + .85, sign * 4.8, 1.05, .045, .045, 1.9,
                                (.65, .70, .75, 1))
        self.add(0, 0, 3, .85, .9, (.22, .60, .59, 1), "nurse station")
        self.labels.append(("NURSE STATION", (0, 0, 1.15)))
        self.actors = [Actor(((-7.5, 1.15), (7.5, 1.15)), .48),
                       Actor(((7.5, -1.15), (-7.5, -1.15)), .42)]

    def ring(self):
        for sign in (1, -1):
            for col, x in enumerate((-8.25, -2.75, 2.75, 8.25)):
                name = ("ICU" if sign > 0 else "WARD") + str(col + 1)
                self.front(x - 2.75, x + 2.75, sign * 4, x + .65)
                if col < 3:
                    self.add(x + 2.75, sign * 6, .16, 4, 1.55)
                self.bed(x - .85, sign * 6.25, name, (x + .65, sign * 6))
                self.add(x - 1.85, sign * 6.7, .55, .6, .8, (.69, .57, .41, 1))
                self.add(x + 1.85, sign * 7.1, .65, .55, 1.1, (.55, .68, .74, 1))
                self.detail(x - 1.85, sign * 6.7, 1.15, .50, .12, .40,
                            (.08, .21, .26, 1))
        self.add(0, 0, 5.8, 3.3, 1, (.25, .59, .59, 1), "central station")
        self.detail(0, 0, 1.08, 5.9, 3.4, .16, (.88, .91, .88, 1))
        self.labels.append(("CENTRAL NURSE STATION", (0, 0, 1.35)))
        for x in (-6.2, 6.2):
            self.add(x, 0, 1.2, 2.2, .5, (.44, .57, .76, 1), "waiting seats")
            self.detail(x, .9, .85, 1.2, .15, .7, (.35, .46, .65, 1))
        self.actors = [Actor(((-9.4, 3), (9.4, 3)), .52),
                       Actor(((9.4, -3), (-9.4, -3)), .45),
                       Actor(((-9.4, -2.8), (-9.4, 2.8)), .35),
                       Actor(((9.4, 2.8), (9.4, -2.8)), .32, .42, True)]


def box_body(pos, size, color, collision=True):
    # PyBullet 接收半尺寸，因此这里把实际长、宽、高各除以二。
    half = [s / 2 for s in size]
    c = p.createCollisionShape(p.GEOM_BOX, halfExtents=half) if collision else -1
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    return p.createMultiBody(0, c, v, pos)


def cylinder(xy, radius, height, color, collision=True):
    c = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height) if collision else -1
    v = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
    return p.createMultiBody(0, c, v, [*xy, height / 2 + .02])


# 每次 tick 推进一个固定时间步：更新人员，再处理巡检和机器人移动。
class Simulation:
    def __init__(self, ward, gui=True):
        self.ward = ward
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        if self.client < 0:
            raise RuntimeError("Cannot connect to PyBullet")
        self.gui = gui
        self.xy = ward.start
        self.temp = []
        self.path = []
        self.lines = []
        # index 指向当前床位；timer 控制重规划间隔，dwell 记录到站停留时间。
        self.index = 0
        self.elapsed = 0
        self.timer = 1
        self.dwell = 0
        self.replans = 0
        self.distance = 0
        self.events = []
        self.paused = False
        self.done = False
        self.status_id = -1
        self.yaw = 0
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setTimeStep(DT)
        box_body([0, 0, -.06], [ward.hx * 2, ward.hy * 2, .12], (.90, .93, .94, 1))
        for b in ward.boxes:
            b.body = box_body([b.x, b.y, b.height / 2], [b.sx, b.sy, b.height], b.color)
        for pos, size, color in ward.decor:
            box_body(pos, size, color, False)
        for text, pos in ward.labels:
            p.addUserDebugText(text, pos, [.1, .2, .28], 1.1)
        for a in ward.actors:
            if a.cart:
                a.body = box_body([*a.xy, .45], [.58, .58, .9], (.96, .65, .17, 1))
            else:
                a.body = cylinder(a.xy, a.radius, 1.5, (.97, .52, .24, 1))
        self.robot = cylinder(self.xy, RADIUS, .45, (.90, .16, .18, 1))
        self.marker = cylinder((self.xy[0] + .22, self.xy[1]), .07, .55, (1, .9, .15, 1), False)
        self.targets = [cylinder(pos, .18, .035, (.15, .8, .35, 1), False)
                        for _, pos in ward.stops]
        # 固定家具不会移动，先缓存可通行网格，后续只补查动态和临时障碍。
        self.static_free = {(i, j) for i in range(int(2 * ward.hx / GRID) + 1)
                            for j in range(int(2 * ward.hy / GRID) + 1)
                            if self.free(self.world((i, j)), dynamic=False)}
        self.camera()

    def camera(self):
        p.resetDebugVisualizerCamera(26 if self.ward.kind == "ring" else 22, 0, -68, [0, 0, 0])

    # 网格编号从病区左下角开始；世界坐标以病区中心为原点。
    def world(self, node):
        return (node[0] * GRID - self.ward.hx, node[1] * GRID - self.ward.hy)

    def grid(self, xy):
        return (round((xy[0] + self.ward.hx) / GRID), round((xy[1] + self.ward.hy) / GRID))

    def free(self, xy, dynamic=True, radius=RADIUS + MARGIN):
        x, y = xy
        if abs(x) > self.ward.hx - radius or abs(y) > self.ward.hy - radius:
            return False
        # 把障碍向外扩展机器人半径和余量，再判断机器人中心能否通过。
        for b in self.ward.boxes + self.temp:
            if abs(x - b.x) <= b.sx / 2 + radius and abs(y - b.y) <= b.sy / 2 + radius:
                return False
        return not dynamic or all(math.dist(xy, a.xy) > radius + a.radius for a in self.ward.actors)

    def segment(self, a, b, dynamic=True):
        # 两端安全不代表整段安全：每隔不超过 6 厘米检查一次。
        count = max(1, math.ceil(math.dist(a, b) / .06))
        return all(self.free((a[0] + (b[0] - a[0]) * i / count,
                              a[1] + (b[1] - a[1]) * i / count), dynamic)
                   for i in range(count + 1))

    def plan(self, start, goal, dynamic=True):
        # 连到真实起点，避免取整后的首段穿墙；所有边均作连续采样检查。
        s, g = self.grid(start), self.grid(goal)
        cache = {}
        def valid(n):
            if n not in cache:
                cache[n] = n in self.static_free and self.free(self.world(n), dynamic)
            return cache[n]
        # 从起点附近选可安全连接的网格，避免坐标取整后落入障碍。
        seeds = [(s[0] + dx, s[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
        seeds = [n for n in seeds if valid(n) and self.segment(start, self.world(n), dynamic)]
        if not seeds or not valid(g):
            return []
        # A* 优先搜索“已走距离 + 到目标的直线估计距离”最小的点。
        cost = {n: math.dist(start, self.world(n)) / GRID for n in seeds}
        queue = [(cost[n] + math.dist(n, g), n) for n in seeds]
        heapq.heapify(queue)
        prev = {}
        closed = set()
        while queue:
            _, node = heapq.heappop(queue)
            if node in closed:
                continue
            if node == g:
                # 沿前驱记录从终点回溯，再反转为起点到终点的顺序。
                path = [goal, self.world(g)]
                while node in prev:
                    node = prev[node]
                    path.append(self.world(node))
                path.append(start)
                path.reverse()
                return path if self.segment(self.world(g), goal, dynamic) else []
            closed.add(node)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (node[0] + dx, node[1] + dy)
                if not valid(nxt):
                    continue
                # 斜向移动时，两侧也必须可通行，防止从障碍夹角中挤过去。
                if dx and dy and (not valid((node[0] + dx, node[1])) or not valid((node[0], node[1] + dy))):
                    continue
                new_cost = cost[node] + math.hypot(dx, dy)
                if new_cost < cost.get(nxt, float('inf')):
                    if not self.segment(self.world(node), self.world(nxt), dynamic):
                        continue
                    cost[nxt], prev[nxt] = new_cost, node
                    heapq.heappush(queue, (new_cost + math.dist(nxt, g), nxt))
        return []

    def replan(self):
        if self.done:
            return
        # 每次都从机器人当前位置出发；无路时保留空路径，等待后续重试。
        self.path = self.plan(self.xy, self.ward.stops[self.index][1])
        self.replans += 1
        self.timer = 0
        for line in self.lines:
            p.removeUserDebugItem(line)
        self.lines = [p.addUserDebugLine([*a, .08], [*b, .08], [0, .72, .3], 3)
                      for a, b in zip(self.path, self.path[1:])] if self.gui else []

    def update_actors(self):
        for a in self.ward.actors:
            target = a.route[a.target]
            d = math.dist(a.xy, target)
            if d < .015:
                a.target = (a.target + 1) % len(a.route)
                continue
            step = min(a.speed * DT, d)
            nxt = tuple(a.xy[k] + (target[k] - a.xy[k]) * step / d for k in (0, 1))
            # 人员/推车也礼让机器人，且不穿过新增障碍。
            blocked = (math.dist(nxt, self.xy) < RADIUS + MARGIN + a.radius + .04
                       or any(other is not a and math.dist(nxt, other.xy) < a.radius + other.radius + .08
                              for other in self.ward.actors)
                       or not self.free(nxt, dynamic=False, radius=a.radius + .05))
            if blocked:
                a.waiting += DT
                # 两点往返路线：等待后折返，避免人员与推车相互卡在门口。
                if a.waiting >= 1.0:
                    a.target = (a.target + 1) % len(a.route)
                    a.waiting = 0
                continue
            a.waiting = 0
            a.xy = nxt
            p.resetBasePositionAndOrientation(a.body, [*nxt, .45 if a.cart else .77], [0, 0, 0, 1])

    def tick(self):
        if self.paused or self.done:
            return
        self.elapsed += DT
        self.timer += DT
        self.update_actors()
        goal = self.ward.stops[self.index][1]
        # 到站后停留 0.6 秒，再记录床位并切换到下一个巡检点。
        if math.dist(self.xy, goal) < .07:
            self.dwell += DT
            if self.dwell >= .6:
                name = self.ward.stops[self.index][0]
                self.events.append(dict(event="bed_arrival", bed=name, time=round(self.elapsed, 3)))
                print(f"[ARRIVAL] {name} at {self.elapsed:.1f}s")
                p.changeVisualShape(self.targets[self.index], -1, rgbaColor=[.2, .4, .9, 1])
                self.index += 1
                self.dwell = 0
                self.done = self.index == len(self.ward.stops)
                self.path = []
                self.timer = 1
            return
        # 没有路径时定期重试，动态障碍离开后自动恢复。
        blocked = not self.path or any(not self.free(pt) for pt in self.path[:6])
        if blocked and self.timer >= .65:
            self.replan()
        while self.path and math.dist(self.xy, self.path[0]) < .015:
            self.path.pop(0)
        if self.path:
            target = self.path[0]
            d = math.dist(self.xy, target)
            # 单步距离不超过剩余距离，避免越过当前路径点。
            step = min(.95 * DT, d)
            nxt = tuple(self.xy[k] + (target[k] - self.xy[k]) * step / d for k in (0, 1))
            # 真正移动前再检查这一小段，防止人员刚好进入路线。
            if self.segment(self.xy, nxt):
                self.yaw = math.atan2(nxt[1] - self.xy[1], nxt[0] - self.xy[0])
                self.distance += step
                self.xy = nxt
                p.resetBasePositionAndOrientation(self.robot, [*nxt, .245], p.getQuaternionFromEuler([0, 0, self.yaw]))
                p.resetBasePositionAndOrientation(self.marker, [nxt[0] + .22 * math.cos(self.yaw), nxt[1] + .22 * math.sin(self.yaw), .48], [0, 0, 0, 1])
        p.stepSimulation()

    def add_obstacle(self):
        # 从未来路线选择空位，拒绝与机器人、家具或巡检点重叠。
        for xy in self.path:
            if math.dist(xy, self.xy) < 1.5:
                continue
            if any(math.dist(xy, pos) < 1.15 for _, pos in self.ward.stops):
                continue
            if not self.free(xy, radius=.65):
                continue
            b = Box(*xy, .7, .7, .8, (1, .34, .12, 1), "temporary")
            b.body = box_body([*xy, .4], [.7, .7, .8], b.color)
            self.temp.append(b)
            self.replan()
            return True
        print("[INFO] No safe space for a temporary obstacle on the current route.")
        return False

    def clear_obstacles(self):
        for b in self.temp:
            p.removeBody(b.body)
        self.temp.clear()
        self.replan()

    def report(self):
        return dict(scenario=self.ward.kind, completed=self.done,
                    visited=len(self.events), beds=len(self.ward.stops),
                    simulated_seconds=round(self.elapsed, 2), distance_m=round(self.distance, 2),
                    replans=self.replans, arrivals=self.events)

    def screenshot(self, path):
        width, height = 1280, 900
        view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 26 if self.ward.kind == "ring" else 22, 0, -68, 0, 2)
        proj = p.computeProjectionMatrixFOV(55, width / height, .1, 100)
        pixels = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER)[2]
        raw = pixels.tobytes() if hasattr(pixels, 'tobytes') else bytes(pixels)
        # 将相机的 RGBA 像素编码成 PNG；每行开头的 0 表示不使用行滤波。
        rows = b''.join(b'\x00' + raw[y * width * 4:(y + 1) * width * 4] for y in range(height))
        def chunk(tag, data):
            return struct.pack('!I', len(data)) + tag + data + struct.pack('!I', zlib.crc32(tag + data) & 0xffffffff)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 6, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


def main(kind):
    parser = argparse.ArgumentParser(description="Complex ward patrol with A* and dynamic obstacles")
    parser.add_argument('--headless', action='store_true', help='Run without GUI, stop on completion or timeout')
    parser.add_argument('--seconds', type=float, default=240, help='Headless simulation time limit')
    parser.add_argument('--screenshot', help='Save a PNG of the initial scene')
    parser.add_argument('--report', help='Save arrival events and statistics as JSON')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    sim = Simulation(Ward(kind), not args.headless)
    try:
        sim.replan()
        if args.screenshot:
            sim.screenshot(args.screenshot)
        print('O: add obstacle | C: clear | R: replan | SPACE: pause | V: camera | ESC: exit')
        print('A* simulation; kinematic movement; no PPO / MQTT integration.')
        # 无窗口模式按仿真时间运行；有窗口模式还处理按键并控制显示节奏。
        while p.isConnected():
            if args.headless and (sim.done or sim.elapsed >= args.seconds):
                break
            if not args.headless:
                keys = p.getKeyboardEvents()
                def pressed(key):
                    return keys.get(key, 0) & p.KEY_WAS_TRIGGERED
                if pressed(27):
                    break
                for key, action in (('o', sim.add_obstacle), ('c', sim.clear_obstacles), ('r', sim.replan), ('v', sim.camera)):
                    if pressed(ord(key)) or pressed(ord(key.upper())):
                        action()
                if pressed(32):
                    sim.paused = not sim.paused
                status = 'COMPLETE' if sim.done else 'PAUSED' if sim.paused else sim.ward.stops[sim.index][0]
                sim.status_id = p.addUserDebugText(f'{kind.upper()}  {len(sim.events)}/{len(sim.ward.stops)}  {status}  |  O C R SPACE V ESC',
                                                  [-sim.ward.hx, -sim.ward.hy - .7, .1], [.12, .20, .26], 1.3,
                                                  replaceItemUniqueId=sim.status_id)
            sim.tick()
            if not args.headless:
                time.sleep(DT)
        report = sim.report()
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if args.report:
            Path(args.report).parent.mkdir(parents=True, exist_ok=True)
            Path(args.report).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        return 0 if not args.headless or sim.done else 1
    except KeyboardInterrupt:
        return 130
    finally:
        # 正常结束或发生异常时，都释放本次仿真的连接。
        p.disconnect(sim.client)
