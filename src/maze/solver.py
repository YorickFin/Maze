"""破解方（视野信任 + 方向偏置前沿探索）。

与旧实现的关键差别：
- 视野内的墙与可通行格都可见、完全信任；只有灰色障碍不可见（保持未知）。
- 一刻不知道地图多大、终点在哪；唯一先验是「目标方向偏右」。
- 终点红点是可见标记：一旦进入视野就记录其位置，随后直接朝它规划；
  在此之前按「右 > 上/下 > 左」的方向代价做前沿探索。
- 规划只在已知可通行格上做（必要时可穿过代价很高的「疑似障碍」格），
  目标是「与从未观测过的区域相邻的前沿」；路径可以直接复用已探明区域回溯。
- 方向代价：向右 1、向上/下 2、向左 4。因此天然优先向右，其次上下绕行，
  最后才向左回溯。
- 踏入未知格：成功则并购新视野；受阻则把该格记为已知障碍（灰），清空
  当前路径并就地重规划。没有「上下左右盲扫」。
- 仅当从当前位置经已探明区域再也到不了任何前沿时，才判定卡死。
"""

from __future__ import annotations

import heapq

from .maze import EMPTY, WALL, Maze

# 破解方对格子的认知状态
KNOWN_EMPTY = 1
KNOWN_WALL = 2      # 视野内直接看到的墙
KNOWN_OBSTACLE = 3  # 撞上后确认的灰色障碍

# 踏入未知格时的尝试顺序：右 → 下 → 上 → 左
DIRS = ((1, 0), (0, 1), (0, -1), (-1, 0))


class Solver:
    """在线前沿探索破解方（不读地图尺寸，不读终点坐标）。"""

    def __init__(self, maze: Maze, view_size: int, max_steps: int,
                 cost_right: float = 1.0, cost_vertical: float = 2.0,
                 cost_left: float = 4.0, suspect_penalty: float = 8.0,
                 plan_expand_limit: int = 200_000):
        self.maze = maze
        self.view_size = view_size
        self.max_steps = max_steps
        self.cost_right = cost_right
        self.cost_vertical = cost_vertical
        self.cost_left = cost_left
        self.suspect_penalty = suspect_penalty
        self.plan_expand_limit = plan_expand_limit

        # 认知网格：键为 (x,y)，值为 KNOWN_EMPTY / KNOWN_WALL / KNOWN_OBSTACLE；
        # 缺省（不在字典中）表示未知。
        self.known: dict[tuple[int, int], int] = {}
        self.pos = maze.start
        self.visited: list[tuple[int, int]] = [maze.start]
        self._visited_set: set[tuple[int, int]] = {maze.start}
        self.explored: set[tuple[int, int]] = set()  # 曾被视野扫过的格
        self.goal: tuple[int, int] | None = None      # 视野内看到红点后才获得

        self.step_count = 0
        self.blocked_hits = 0
        self.done = False

        # 当前计划：path 是已知格序列，entry 是走到前沿后要踏入的未知格
        self.path: list[tuple[int, int]] = []
        self.path_idx = 0
        self.entry: tuple[int, int] | None = None

        # 增量（供可视化按帧取走，避免每帧重传全量）
        self.new_explored: list[tuple[int, int]] = []
        self.new_visited: list[tuple[int, int]] = []
        self.new_walls: list[tuple[int, int]] = []
        self.new_obstacles: list[tuple[int, int]] = []

        self.known[maze.start] = KNOWN_EMPTY
        self._observe()  # 初始视野

    # ---- 感知 ----

    def _observe(self) -> None:
        """把视野内可见信息并入认知：墙→已知墙，空白→已知可通行。

        灰色障碍对传感器不可见，保持未知；只有撞上后才会被确认。
        """
        half = self.view_size // 2
        x0 = self.pos[0] - half
        y0 = self.pos[1] - half
        for gy in range(y0, y0 + self.view_size):
            for gx in range(x0, x0 + self.view_size):
                if not self.maze.in_bounds(gx, gy):
                    continue
                cell = (gx, gy)
                if self.goal is None and self.maze.is_goal_marker(gx, gy):
                    self.goal = cell  # 红点进入视野：终点标记可见
                if cell not in self.explored:
                    self.explored.add(cell)
                    self.new_explored.append(cell)
                if self.known.get(cell) is not None:
                    continue
                v = self.maze.sense(gx, gy)
                if v is None:
                    continue  # 灰色障碍不可见 → 维持未知
                if v == WALL:
                    self.known[cell] = KNOWN_WALL
                    self.new_walls.append(cell)
                elif v == EMPTY:
                    self.known[cell] = KNOWN_EMPTY

    def _try_move(self, nx: int, ny: int) -> bool:
        """尝试移动一格。受阻时标记障碍/墙并返回 False。"""
        cell = (nx, ny)
        if self.maze.is_blocked(nx, ny):
            if self.maze.is_wall(nx, ny):
                self.known[cell] = KNOWN_WALL
                self.new_walls.append(cell)
            else:
                self.known[cell] = KNOWN_OBSTACLE
                self.new_obstacles.append(cell)
            self.blocked_hits += 1
            return False
        self.known[cell] = KNOWN_EMPTY
        self.pos = cell
        if cell not in self._visited_set:
            self._visited_set.add(cell)
            self.visited.append(cell)
            self.new_visited.append(cell)
        self._observe()
        return True

    # ---- 规划：方向代价 Dijkstra，目标 = 前沿 ----

    def _direction_cost(self, dx: int) -> float:
        if dx > 0:
            return self.cost_right
        if dx < 0:
            return self.cost_left
        return self.cost_vertical

    def _unexplored_neighbor(self, cell: tuple[int, int]) -> tuple[int, int] | None:
        """返回 cell 的邻格中「从未被观测过」的那一格（右→下→上→左）。"""
        x, y = cell
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if not self.maze.in_bounds(nx, ny):
                continue
            nb = (nx, ny)
            if self.known.get(nb) is None and nb not in self.explored:
                return nb
        return None

    def _plan_to_goal(self) -> bool:
        """看到红点后，直接朝它规划（Dijkstra，方向代价相同）。"""
        goal = self.goal
        if goal is None or self.known.get(goal) != KNOWN_EMPTY:
            return False
        start = self.pos
        if start == goal:
            self.path = [goal]
            self.path_idx = 0
            self.entry = None
            return True

        heap: list[tuple[float, int, tuple[int, int]]] = [(0.0, 0, start)]
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        counter = 1
        expanded = 0

        while heap:
            d, _, cur = heapq.heappop(heap)
            if d > g_score.get(cur, float("inf")):
                continue
            if cur == goal:
                path = [cur]
                while cur in came_from:
                    cur = came_from[cur]
                    path.append(cur)
                path.reverse()
                self.path = path
                self.path_idx = 0
                self.entry = None
                return True
            expanded += 1
            if expanded > self.plan_expand_limit:
                break
            cx, cy = cur
            for dx, dy in DIRS:
                nx, ny = cx + dx, cy + dy
                if not self.maze.in_bounds(nx, ny):
                    continue
                nb = (nx, ny)
                state = self.known.get(nb)
                if state == KNOWN_WALL or state == KNOWN_OBSTACLE:
                    continue
                if state is None:
                    if nb not in self.explored:
                        continue
                    step_cost = self._direction_cost(dx) + self.suspect_penalty
                else:
                    step_cost = self._direction_cost(dx)
                nd = d + step_cost
                if nd < g_score.get(nb, float("inf")):
                    g_score[nb] = nd
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(heap, (nd, counter, nb))
        return False

    def _plan(self) -> bool:
        """用 Dijkstra 找最近（按方向代价）的前沿并生成路径。

        只扩展「已知可通行」格；「已观测但未知」的格视为疑似障碍，
        允许穿过但附加较高代价，从而优先绕行、无路可走时才硬闯。
        从未观测过的格只作为前沿目标，不作为路径中间点。

        若当前已无任何前沿（可达区域都探明），则退化为「向最右已知点
        收束」：走向可达的最右可通行格，体现「目标方向偏右」这一先验。
        """
        if self._plan_to_goal():
            return True

        start = self.pos
        heap: list[tuple[float, int, tuple[int, int]]] = [(0.0, 0, start)]
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        counter = 1
        expanded = 0
        best_cell: tuple[int, int] | None = None
        best_key: tuple[int, int] | None = None

        while heap:
            d, _, cur = heapq.heappop(heap)
            if d > g_score.get(cur, float("inf")):
                continue
            if self.known.get(cur) == KNOWN_EMPTY:
                entry = self._unexplored_neighbor(cur)
                if entry is not None:
                    path = [cur]
                    while cur in came_from:
                        cur = came_from[cur]
                        path.append(cur)
                    path.reverse()
                    self.path = path
                    self.path_idx = 0
                    self.entry = entry
                    return True
                key = (cur[0], -abs(cur[1] - start[1]))
                if best_key is None or key > best_key:
                    best_key = key
                    best_cell = cur

            expanded += 1
            if expanded > self.plan_expand_limit:
                break

            cx, cy = cur
            for dx, dy in DIRS:
                nx, ny = cx + dx, cy + dy
                if not self.maze.in_bounds(nx, ny):
                    continue
                nb = (nx, ny)
                state = self.known.get(nb)
                if state == KNOWN_WALL or state == KNOWN_OBSTACLE:
                    continue
                if state is None:
                    if nb not in self.explored:
                        continue  # 未观测格只作为前沿目标
                    step_cost = self._direction_cost(dx) + self.suspect_penalty
                else:
                    step_cost = self._direction_cost(dx)
                nd = d + step_cost
                if nd < g_score.get(nb, float("inf")):
                    g_score[nb] = nd
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(heap, (nd, counter, nb))

        # 无前沿可去 → 向最右已知点收束
        if best_cell is not None and best_cell != start:
            cur = best_cell
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            self.path = path
            self.path_idx = 0
            self.entry = None
            return True
        return False

    def _clear_plan(self) -> None:
        self.path = []
        self.path_idx = 0
        self.entry = None

    # ---- 主步进 ----

    def step(self) -> bool:
        """执行一步（移动或踏入未知格）。返回 True 表示仍在推进。"""
        if self.done:
            return False
        if self.goal is not None and self.pos == self.goal:
            self.done = True  # 已走到看到过的红点
            return False
        if self.step_count >= self.max_steps:
            self.done = True
            return False

        if not self.path:
            if not self._plan():
                self.done = True
                return False

        self.step_count += 1

        # 沿已探明路径走一格
        if self.path_idx + 1 < len(self.path):
            nxt = self.path[self.path_idx + 1]
            self.path_idx += 1
            if not self._try_move(nxt[0], nxt[1]):
                self._clear_plan()  # 疑似障碍挡路 → 下一步重规划
            return True

        # 已到前沿格 → 踏入未知格（这一步会揭示新视野或确认障碍）
        entry = self.entry
        self._clear_plan()
        if entry is None:
            return True
        self._try_move(entry[0], entry[1])
        return True
