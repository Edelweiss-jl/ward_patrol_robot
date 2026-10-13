import pybullet as p
import time
import math
import heapq


# ============================================================
# 1. 基础参数
# ============================================================

ROOM_X_MIN = -5.0
ROOM_X_MAX = 5.0

ROOM_Y_MIN = -4.0
ROOM_Y_MAX = 4.0

# 每个网格边长为 0.25 米；寻路在网格上完成，显示和移动使用实际坐标。
GRID_SIZE = 0.25

ROBOT_RADIUS = 0.28
# 规划时把障碍向外扩大，给机器人本体之外再留一圈余量。
SAFETY_MARGIN = 0.18

MOVE_SPEED = 1.0

# 机器人起点
START = [-3.8, -3.0]

# 目标位置
GOAL = [3.8, 3.0]


# ============================================================
# 2. 连接 PyBullet
# ============================================================

physics_client = p.connect(p.GUI)

p.setGravity(0, 0, -9.8)


# ============================================================
# 3. 创建基础长方体
# ============================================================

def create_box(position, size, color, mass=0):

    # size 是完整尺寸，PyBullet 的 halfExtents 要填写各方向的一半。
    half_extents = [
        size[0] / 2,
        size[1] / 2,
        size[2] / 2
    ]

    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents
    )

    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color
    )

    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )

    return body_id


# ============================================================
# 4. 自己创建地面
#    不使用 plane.urdf
# ============================================================

ground_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[5, 4, 0.05]
)

ground_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[5, 4, 0.05],
    rgbaColor=[0.92, 0.92, 0.92, 1]
)

ground_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=ground_collision,
    baseVisualShapeIndex=ground_visual,
    basePosition=[0, 0, -0.05]
)


# ============================================================
# 5. 静态障碍物列表
# ============================================================

static_obstacles = []


def add_static_obstacle(
        x,
        y,
        sx,
        sy,
        height=0.6,
        color=None
):

    if color is None:
        color = [0.4, 0.6, 0.9, 1]

    body_id = create_box(
        [x, y, height / 2],
        [sx, sy, height],
        color
    )

    static_obstacles.append(
        {
            "id": body_id,
            "x": x,
            "y": y,
            "sx": sx,
            "sy": sy
        }
    )

    return body_id


# ============================================================
# 6. 创建病房墙体
# ============================================================

wall_color = [0.75, 0.75, 0.75, 1]

wall_height = 1.5
wall_thickness = 0.15

# 上墙
create_box(
    [0, 4, wall_height / 2],
    [10, wall_thickness, wall_height],
    wall_color
)

# 下墙
create_box(
    [0, -4, wall_height / 2],
    [10, wall_thickness, wall_height],
    wall_color
)

# 左墙
create_box(
    [-5, 0, wall_height / 2],
    [wall_thickness, 8, wall_height],
    wall_color
)

# 右墙
create_box(
    [5, 0, wall_height / 2],
    [wall_thickness, 8, wall_height],
    wall_color
)


# ============================================================
# 7. 病床
# ============================================================

bed_color = [0.25, 0.55, 1.0, 1]

# 左上床
add_static_obstacle(
    -3.3,
    2.5,
    2.0,
    0.9,
    0.55,
    bed_color
)

# 右上床
add_static_obstacle(
    2.0,
    2.5,
    2.0,
    0.9,
    0.55,
    bed_color
)

# 左下床
add_static_obstacle(
    -3.3,
    -1.4,
    2.0,
    0.9,
    0.55,
    bed_color
)

# 右下床
add_static_obstacle(
    2.0,
    -1.4,
    2.0,
    0.9,
    0.55,
    bed_color
)


# ============================================================
# 8. 护士站
# ============================================================

add_static_obstacle(
    0.0,
    0.7,
    1.6,
    0.8,
    0.9,
    [0.62, 0.42, 0.22, 1]
)


# ============================================================
# 9. 柜子
# ============================================================

add_static_obstacle(
    -0.8,
    -2.5,
    0.8,
    0.8,
    0.8,
    [0.55, 0.55, 0.55, 1]
)


# ============================================================
# 10. 再加一个小设备柜
# ============================================================

add_static_obstacle(
    3.6,
    -2.6,
    0.7,
    1.0,
    0.9,
    [0.55, 0.55, 0.55, 1]
)


# ============================================================
# 11. 创建机器人
# ============================================================

robot_collision = p.createCollisionShape(
    p.GEOM_CYLINDER,
    radius=ROBOT_RADIUS,
    height=0.4
)

robot_visual = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=ROBOT_RADIUS,
    length=0.4,
    rgbaColor=[0.9, 0.1, 0.1, 1]
)

robot_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=robot_collision,
    baseVisualShapeIndex=robot_visual,
    basePosition=[
        START[0],
        START[1],
        0.2
    ]
)


# ============================================================
# 12. 创建机器人正面黄色标记
# ============================================================

marker_visual = p.createVisualShape(
    p.GEOM_SPHERE,
    radius=0.08,
    rgbaColor=[1, 1, 0, 1]
)

marker_id = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=marker_visual,
    basePosition=[
        START[0] + 0.3,
        START[1],
        0.35
    ]
)


# ============================================================
# 13. 创建目标点
# ============================================================

goal_visual = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=0.25,
    length=0.05,
    rgbaColor=[0.0, 1.0, 0.2, 1]
)

goal_id = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=goal_visual,
    basePosition=[
        GOAL[0],
        GOAL[1],
        0.03
    ]
)


# ============================================================
# 14. 创建移动人员
# ============================================================

def create_person(x, y, color):

    collision_shape = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=0.25,
        height=1.4
    )

    visual_shape = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=0.25,
        length=1.4,
        rgbaColor=color
    )

    person_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x, y, 0.7]
    )

    return person_id


person1 = create_person(
    -1.5,
    -0.5,
    [1.0, 0.55, 0.1, 1]
)

person2 = create_person(
    3.2,
    0.2,
    [0.7, 0.2, 0.8, 1]
)


people = [
    {
        "id": person1,
        "x": -1.5,
        "y": -0.5,
        "direction": 1,
        "min_y": -0.5,
        "max_y": 2.0
    },

    {
        "id": person2,
        "x": 3.2,
        "y": 0.2,
        "direction": 1,
        "min_y": -0.2,
        "max_y": 2.0
    }
]


# ============================================================
# 15. 临时障碍物
# ============================================================

temporary_obstacles = []


# ============================================================
# 16. 世界坐标 → 网格坐标
# ============================================================

# 先减去左下角坐标，再按网格边长换算为最近的网格编号。
def world_to_grid(x, y):

    gx = int(
        round(
            (x - ROOM_X_MIN) / GRID_SIZE
        )
    )

    gy = int(
        round(
            (y - ROOM_Y_MIN) / GRID_SIZE
        )
    )

    return gx, gy


# ============================================================
# 17. 网格坐标 → 世界坐标
# ============================================================

def grid_to_world(gx, gy):

    x = ROOM_X_MIN + gx * GRID_SIZE

    y = ROOM_Y_MIN + gy * GRID_SIZE

    return x, y


# ============================================================
# 18. 判断点是否在矩形障碍内部
# ============================================================

# 检查的是扩大的矩形：机器人中心在矩形外，才有足够的通行空间。
def point_in_rectangle(x, y, obstacle):

    inflate = (
        ROBOT_RADIUS
        + SAFETY_MARGIN
    )

    min_x = (
        obstacle["x"]
        - obstacle["sx"] / 2
        - inflate
    )

    max_x = (
        obstacle["x"]
        + obstacle["sx"] / 2
        + inflate
    )

    min_y = (
        obstacle["y"]
        - obstacle["sy"] / 2
        - inflate
    )

    max_y = (
        obstacle["y"]
        + obstacle["sy"] / 2
        + inflate
    )

    return (
        min_x <= x <= max_x
        and
        min_y <= y <= max_y
    )


# ============================================================
# 19. 判断网格是否可通行
# ============================================================

def is_free(gx, gy):

    x, y = grid_to_world(
        gx,
        gy
    )

    margin = ROBOT_RADIUS + 0.15

    # 超出边界
    if (
        x < ROOM_X_MIN + margin
        or
        x > ROOM_X_MAX - margin
        or
        y < ROOM_Y_MIN + margin
        or
        y > ROOM_Y_MAX - margin
    ):
        return False

    # 静态障碍物
    for obstacle in static_obstacles:

        if point_in_rectangle(
            x,
            y,
            obstacle
        ):
            return False

    # 临时障碍物
    for obstacle in temporary_obstacles:

        if point_in_rectangle(
            x,
            y,
            obstacle
        ):
            return False

    # 移动人员
    for person in people:

        dx = x - person["x"]
        dy = y - person["y"]

        distance = math.hypot(
            dx,
            dy
        )

        safety_distance = (
            ROBOT_RADIUS
            + 0.25
            + 0.25
        )

        if distance < safety_distance:
            return False

    return True


# ============================================================
# 20. A* 启发函数
# ============================================================

# 用网格间的直线距离估计剩余路程，引导搜索向目标靠近。
def heuristic(a, b):

    return math.hypot(
        a[0] - b[0],
        a[1] - b[1]
    )


# ============================================================
# 21. A* 路径规划
# ============================================================

def astar(start_world, goal_world):

    start = world_to_grid(
        start_world[0],
        start_world[1]
    )

    goal = world_to_grid(
        goal_world[0],
        goal_world[1]
    )

    # 候选点按估计总路程排序；每次优先取出最有希望的点。
    open_heap = []

    heapq.heappush(
        open_heap,
        (0, start)
    )

    # came_from 记录每个点从哪里到达；g_score 记录到该点的已知最短路程。
    came_from = {}

    g_score = {
        start: 0
    }

    directions = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),

        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1)
    ]

    visited = set()

    while open_heap:

        _, current = heapq.heappop(
            open_heap
        )

        if current in visited:
            continue

        visited.add(current)

        # 找到目标后沿前驱逐步回到起点，最后反转得到正向路线。
        if current == goal:

            path = [
                grid_to_world(
                    current[0],
                    current[1]
                )
            ]

            while current in came_from:

                current = came_from[
                    current
                ]

                path.append(
                    grid_to_world(
                        current[0],
                        current[1]
                    )
                )

            path.reverse()

            return path

        for dx, dy in directions:

            neighbor = (
                current[0] + dx,
                current[1] + dy
            )

            if not is_free(
                neighbor[0],
                neighbor[1]
            ):
                continue

            # 防止斜着穿过墙角
            if dx != 0 and dy != 0:

                side1 = (
                    current[0] + dx,
                    current[1]
                )

                side2 = (
                    current[0],
                    current[1] + dy
                )

                if (
                    not is_free(
                        side1[0],
                        side1[1]
                    )
                    or
                    not is_free(
                        side2[0],
                        side2[1]
                    )
                ):
                    continue

            # 横竖移动为 1 格，斜向移动约为根号 2 格。
            move_cost = (
                1.414
                if dx != 0 and dy != 0
                else 1.0
            )

            tentative_g = (
                g_score[current]
                + move_cost
            )

            if (
                neighbor not in g_score
                or
                tentative_g
                < g_score[neighbor]
            ):

                came_from[
                    neighbor
                ] = current

                g_score[
                    neighbor
                ] = tentative_g

                # 估计总路程 = 已走路程 + 到目标的估计距离。
                f_score = (
                    tentative_g
                    + heuristic(
                        neighbor,
                        goal
                    )
                )

                heapq.heappush(
                    open_heap,
                    (
                        f_score,
                        neighbor
                    )
                )

    return []


# ============================================================
# 22. 绘制路线
# ============================================================

path_lines = []


def clear_path_lines():

    global path_lines

    for line_id in path_lines:

        p.removeUserDebugItem(
            line_id
        )

    path_lines = []


def draw_path(path):

    global path_lines

    clear_path_lines()

    if len(path) < 2:
        return

    for i in range(
        len(path) - 1
    ):

        start_point = path[i]
        end_point = path[i + 1]

        line_id = p.addUserDebugLine(
            [
                start_point[0],
                start_point[1],
                0.06
            ],
            [
                end_point[0],
                end_point[1],
                0.06
            ],
            [0, 1, 0],
            lineWidth=3
        )

        path_lines.append(
            line_id
        )


# ============================================================
# 23. 获取机器人 XY
# ============================================================

def get_robot_xy():

    position, orientation = (
        p.getBasePositionAndOrientation(
            robot_id
        )
    )

    return [
        position[0],
        position[1]
    ]


# ============================================================
# 24. 当前路径
# ============================================================

current_path = []
path_index = 0


# ============================================================
# 25. 重新规划
# ============================================================

def replan():

    global current_path
    global path_index

    start = get_robot_xy()

    new_path = astar(
        start,
        GOAL
    )

    if not new_path:

        print(
            "⚠ 当前找不到可行路径"
        )

        current_path = []

        clear_path_lines()

        return False

    current_path = new_path

    # 第一个点一般就是机器人当前位置附近
    if len(current_path) > 1:
        path_index = 1
    else:
        path_index = 0

    draw_path(
        current_path
    )

    print(
        "✅ 路径规划成功，路径点数量：",
        len(current_path)
    )

    return True


# ============================================================
# 26. 添加临时障碍物
# ============================================================

def add_temporary_obstacle():

    position, orientation = (
        p.getBasePositionAndOrientation(
            robot_id
        )
    )

    _, _, yaw = (
        p.getEulerFromQuaternion(
            orientation
        )
    )

    # 根据机器人朝向，在前方 1.2 米处放置临时障碍并重新寻路。
    distance = 1.2

    obstacle_x = (
        position[0]
        + distance * math.cos(yaw)
    )

    obstacle_y = (
        position[1]
        + distance * math.sin(yaw)
    )

    obstacle_size_x = 0.9
    obstacle_size_y = 0.9

    body_id = create_box(
        [
            obstacle_x,
            obstacle_y,
            0.5
        ],
        [
            obstacle_size_x,
            obstacle_size_y,
            1.0
        ],
        [1.0, 0.3, 0.05, 1]
    )

    temporary_obstacles.append(
        {
            "id": body_id,
            "x": obstacle_x,
            "y": obstacle_y,
            "sx": obstacle_size_x,
            "sy": obstacle_size_y
        }
    )

    print("")
    print(
        "🟧 检测到新的临时障碍物"
    )

    print(
        "正在重新规划路线..."
    )

    replan()


# ============================================================
# 27. 清除临时障碍物
# ============================================================

def clear_temporary_obstacles():

    global temporary_obstacles

    for obstacle in temporary_obstacles:

        p.removeBody(
            obstacle["id"]
        )

    temporary_obstacles = []

    print("")
    print(
        "临时障碍物已清除"
    )

    replan()


# ============================================================
# 28. 更新移动人员
# ============================================================

def update_people(dt):

    speed = 0.55

    for person in people:

        person["y"] += (
            person["direction"]
            * speed
            * dt
        )

        if person["y"] >= person["max_y"]:

            person["y"] = person["max_y"]

            person["direction"] = -1

        elif person["y"] <= person["min_y"]:

            person["y"] = person["min_y"]

            person["direction"] = 1

        p.resetBasePositionAndOrientation(
            person["id"],
            [
                person["x"],
                person["y"],
                0.7
            ],
            [0, 0, 0, 1]
        )


# ============================================================
# 29. 判断当前路径是否被人挡住
# ============================================================

def current_path_blocked():

    if not current_path:
        return False

    if path_index >= len(
        current_path
    ):
        return False

    # 只检查接下来几个路径点
    end_index = min(
        path_index + 6,
        len(current_path)
    )

    for i in range(
        path_index,
        end_index
    ):

        point = current_path[i]

        for person in people:

            distance = math.hypot(
                point[0]
                - person["x"],

                point[1]
                - person["y"]
            )

            if distance < 0.75:
                return True

    return False


# ============================================================
# 30. 机器人沿路径移动
# ============================================================

def follow_path(dt):

    global path_index

    if not current_path:
        return

    if path_index >= len(
        current_path
    ):
        return

    robot_position = get_robot_xy()

    target = current_path[
        path_index
    ]

    dx = (
        target[0]
        - robot_position[0]
    )

    dy = (
        target[1]
        - robot_position[1]
    )

    distance = math.hypot(
        dx,
        dy
    )

    # 到达当前路径点
    if distance < 0.10:

        path_index += 1

        return

    yaw = math.atan2(
        dy,
        dx
    )

    # 限制每帧移动量，避免越过路径点；这里通过直接更新位置实现移动。
    move_distance = min(
        MOVE_SPEED * dt,
        distance
    )

    new_x = (
        robot_position[0]
        + move_distance
        * math.cos(yaw)
    )

    new_y = (
        robot_position[1]
        + move_distance
        * math.sin(yaw)
    )

    quaternion = (
        p.getQuaternionFromEuler(
            [0, 0, yaw]
        )
    )

    p.resetBasePositionAndOrientation(
        robot_id,
        [
            new_x,
            new_y,
            0.2
        ],
        quaternion
    )

    # 黄色前方标记
    marker_x = (
        new_x
        + 0.30
        * math.cos(yaw)
    )

    marker_y = (
        new_y
        + 0.30
        * math.sin(yaw)
    )

    p.resetBasePositionAndOrientation(
        marker_id,
        [
            marker_x,
            marker_y,
            0.35
        ],
        [0, 0, 0, 1]
    )


# ============================================================
# 31. 判断是否到达目标
# ============================================================

goal_reached = False


def check_goal():

    global goal_reached

    robot_position = get_robot_xy()

    distance = math.hypot(
        robot_position[0]
        - GOAL[0],

        robot_position[1]
        - GOAL[1]
    )

    if (
        distance < 0.3
        and
        not goal_reached
    ):

        goal_reached = True

        clear_path_lines()

        print("")
        print(
            "================================"
        )

        print(
            "🎯 机器人已经到达目标床位"
        )

        print(
            "================================"
        )


# ============================================================
# 32. 设置摄像机
# ============================================================

p.resetDebugVisualizerCamera(
    cameraDistance=11,
    cameraYaw=0,
    cameraPitch=-70,
    cameraTargetPosition=[
        0,
        0,
        0
    ]
)


# ============================================================
# 33. 初始规划
# ============================================================

print("")
print(
    "正在生成初始路线..."
)

replan()


# ============================================================
# 34. 使用说明
# ============================================================

print("")
print(
    "=========================================="
)

print(
    "病房动态路径规划 Demo"
)

print(
    "=========================================="
)

print(
    "机器人会自动按照 A* 路径前往绿色目标点"
)

print("")

print(
    "O ：在机器人前方添加临时障碍物"
)

print(
    "C ：清除全部临时障碍物"
)

print(
    "R ：手动重新规划"
)

print(
    "ESC ：退出程序"
)

print(
    "=========================================="
)

print("")


# ============================================================
# 35. 主循环
# ============================================================

dt = 1 / 120

# 防止人员导致路径每一帧都重算
dynamic_replan_timer = 0

REPLAN_INTERVAL = 0.8


try:

    while True:

        keys = p.getKeyboardEvents()

        # ----------------------------------------------------
        # O：添加临时障碍物
        # ----------------------------------------------------

        if ord("o") in keys:

            if (
                keys[ord("o")]
                & p.KEY_WAS_TRIGGERED
            ):

                add_temporary_obstacle()

        if ord("O") in keys:

            if (
                keys[ord("O")]
                & p.KEY_WAS_TRIGGERED
            ):

                add_temporary_obstacle()

        # ----------------------------------------------------
        # C：清除临时障碍物
        # ----------------------------------------------------

        if ord("c") in keys:

            if (
                keys[ord("c")]
                & p.KEY_WAS_TRIGGERED
            ):

                clear_temporary_obstacles()

        if ord("C") in keys:

            if (
                keys[ord("C")]
                & p.KEY_WAS_TRIGGERED
            ):

                clear_temporary_obstacles()

        # ----------------------------------------------------
        # R：手动重规划
        # ----------------------------------------------------

        if ord("r") in keys:

            if (
                keys[ord("r")]
                & p.KEY_WAS_TRIGGERED
            ):

                print("")
                print(
                    "手动触发重新规划"
                )

                replan()

        if ord("R") in keys:

            if (
                keys[ord("R")]
                & p.KEY_WAS_TRIGGERED
            ):

                print("")
                print(
                    "手动触发重新规划"
                )

                replan()

        # ----------------------------------------------------
        # ESC：退出
        # ----------------------------------------------------

        if 27 in keys:

            if (
                keys[27]
                & p.KEY_WAS_TRIGGERED
            ):
                break

        # ----------------------------------------------------
        # 更新移动人员
        # ----------------------------------------------------

        # 先更新人员位置，让本轮路径检查使用最新的障碍信息。
        update_people(dt)

        # ----------------------------------------------------
        # 动态人员阻挡 → 重新规划
        # ----------------------------------------------------

        dynamic_replan_timer += dt

        if (
            dynamic_replan_timer
            >= REPLAN_INTERVAL
        ):

            dynamic_replan_timer = 0

            if (
                not goal_reached
                and
                current_path_blocked()
            ):

                print("")
                print(
                    "👤 检测到人员进入当前路线"
                )

                print(
                    "正在动态重新规划..."
                )

                replan()

        # ----------------------------------------------------
        # 机器人移动
        # ----------------------------------------------------

        if not goal_reached:

            follow_path(dt)

        # ----------------------------------------------------
        # 判断到达
        # ----------------------------------------------------

        check_goal()

        # ----------------------------------------------------
        # PyBullet 仿真
        # ----------------------------------------------------

        p.stepSimulation()

        time.sleep(dt)


except KeyboardInterrupt:

    print("")
    print(
        "程序被用户终止"
    )


finally:

    p.disconnect()