"""破解方（沿墙为主 + 方向偏置前沿规划兜底）。

- 视野内的墙与可通行格都可见、完全信任；只有灰色障碍不可见（保持未知）。
- 不知道地图多大、也不预先知道终点；只先验「目标方向偏右」。
- 红点是可见标记：进入视野后记录其位置，随后直接规划过去。
- 默认「沿墙模式」：右手法则，尝试顺序 右 → 直 → 左 → 后（无墙优先），
  只用已知地图判断相邻格；已知墙/障碍直接跳过，未知格尝试，撞到灰斑即标记。
  每次成功后记录 (位置, 朝向)，状态重复（打转）或四向都不可行时切到规划模式。
- 「规划模式」兜底：方向代价 Dijkstra（右 1、上/下 3、左 10）走向
  「与未观测区域相邻的前沿」；路径可复用已探明区域回溯，没有四邻盲扫。
- 感知增量：初始完整 25×25 视野，之后每移动一格只观测新进入的一条 25 格视野边。
"""

from __future__ import annotations

import heapq

from .maze import EMPTY, WALL, Maze

# 破解方对格子的认知状态
KNOWN_EMPTY = 1
KNOWN_WALL = 2      # 视野内直接看到的墙
KNOWN_OBSTACLE = 3  # 撞上后确认的灰色障碍

# 四邻域（前沿聚类连通性用）：右 → 下 → 上 → 左
DIRS = ((1, 0), (0, 1), (0, -1), (-1, 0))

# 八邻域，顺时针：0=E, 1=SE, 2=S, 3=SW, 4=W, 5=NW, 6=N, 7=NE
DIRS8 = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))

# 前沿入口尝试顺序：先正交，后对角
ENTRY_DIRS = ((1, 0), (0, 1), (0, -1), (-1, 0),
              (1, 1), (1, -1), (-1, 1), (-1, -1))

# 四向沿墙：E, S, W, N；相对转向 右90 → 直 → 左90 → 掉头
WALL_DIRS4 = ((1, 0), (0, 1), (-1, 0), (0, -1))
WALL_TURNS4 = (1, 0, 3, 2)

# 八向沿墙转向顺序：+1(45°右) → +2(90°右) → 0 → -1(45°左)
# → -2(90°左) → +3(135°右) → -3(135°左) → +4(掉头)
WALL_TURNS8 = (1, 2, 0, 7, 6, 3, 5, 4)

MODE_WALL = "沿墙"
MODE_PLAN = "规划"


class Solver:
    """在线混合破解方（不读地图尺寸，不读终点坐标）。"""

    def __init__(self, maze: Maze, view_size: int, max_steps: int,
                 cost_right: float = 1.0, cost_vertical: float = 3.0,
                 cost_left: float = 10.0, suspect_penalty: float = 20.0,
                 plan_expand_limit: int = 200_000, wall_follow: bool = False,
                 cluster_enabled: bool = True,
                 cluster_gain_weight: float = 2.0,
                 cluster_right_weight: float = 0.0,
                 cluster_cost_weight: float = 1.0,
                 cluster_min_size: int = 5,
                 cluster_top_k: int = 4,
                 cluster_gain_radius: int = 10,
                 cluster_expand_limit: int = 10_000,
                 allow_diagonal: bool = True,
                 diagonal_factor: float = 1.414):
        self.maze = maze
        self.view_size = view_size
        self.max_steps = max_steps
        self.cost_right = cost_right
        self.cost_vertical = cost_vertical
        self.cost_left = cost_left
        self.suspect_penalty = suspect_penalty
        self.plan_expand_limit = plan_expand_limit
        self.wall_follow = wall_follow
        self.cluster_enabled = cluster_enabled
        self.cluster_gain_weight = cluster_gain_weight
        self.cluster_right_weight = cluster_right_weight
        self.cluster_cost_weight = cluster_cost_weight
        self.cluster_min_size = cluster_min_size
        self.cluster_top_k = cluster_top_k
        self.cluster_gain_radius = cluster_gain_radius
        self.cluster_expand_limit = cluster_expand_limit
        # 八向分支因子更大，限制聚类评估规模以控制耗时
        if allow_diagonal:
            self.cluster_top_k = min(self.cluster_top_k, 2)
            self.cluster_expand_limit = min(self.cluster_expand_limit, 3000)
        self.allow_diagonal = allow_diagonal
        self.diagonal_factor = diagonal_factor
        self.plan_dirs = DIRS8 if allow_diagonal else DIRS
        self.wall_dirs = DIRS8 if allow_diagonal else WALL_DIRS4
        self.wall_turns = WALL_TURNS8 if allow_diagonal else WALL_TURNS4
        self.entry_dirs = ENTRY_DIRS if allow_diagonal else DIRS

        # 模式状态：默认沿墙；打转/八向都不可行后永久切到规划
        self.mode = MODE_WALL if wall_follow else MODE_PLAN
        self.facing = 0            # 相对 wall_dirs 的朝向索引
        self.wall_turn = 0         # 当前尝试的转向序号
        self.wall_states: set[tuple[tuple[int, int], int]] = {(maze.start, 0)}
        self.wall_failed = False   # 是否因打转/卡住触发过兜底

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
        # 与起点 4-连通的已知可通行格；用于 O(1) 判断终点是否已可直达
        self.reach: set[tuple[int, int]] = {maze.start}
        self.frontier: set[tuple[int, int]] = set()  # 已知可通行且邻接未观测区域的格
        self._obs_pos: tuple[int, int] | None = None
        self._observe(full=True)  # 初始完整视野

    @classmethod
    def from_config(cls, maze: Maze, cfg) -> "Solver":
        """按 Config 构造求解器（main 与重开共用）。"""
        return cls(
            maze,
            view_size=cfg.view_size,
            max_steps=cfg.max_steps,
            cost_right=cfg.cost_right,
            cost_vertical=cfg.cost_vertical,
            cost_left=cfg.cost_left,
            suspect_penalty=cfg.suspect_penalty,
            plan_expand_limit=cfg.plan_expand_limit,
            wall_follow=cfg.wall_follow,
            cluster_enabled=cfg.cluster_enabled,
            cluster_gain_weight=cfg.cluster_gain_weight,
            cluster_right_weight=cfg.cluster_right_weight,
            cluster_cost_weight=cfg.cluster_cost_weight,
            cluster_min_size=cfg.cluster_min_size,
            cluster_top_k=cfg.cluster_top_k,
            cluster_gain_radius=cfg.cluster_gain_radius,
            cluster_expand_limit=cfg.cluster_expand_limit,
            allow_diagonal=cfg.allow_diagonal,
            diagonal_factor=cfg.diagonal_factor,
        )

    def set_strategy(self, wall_follow: bool, cluster_enabled: bool) -> None:
        """即时切换策略：沿墙 / 聚类 gain / 前沿规划。"""
        self.wall_follow = wall_follow
        self.cluster_enabled = cluster_enabled
        self._clear_plan()
        if wall_follow:
            self.mode = MODE_WALL
            self.facing = 0
            self.wall_turn = 0
            self.wall_failed = False
            self.wall_states = {(self.pos, 0)}
        else:
            self.mode = MODE_PLAN

    def set_view_size(self, value: int) -> None:
        """即时修改视野边长；重置观测锚点并按新视野做一次完整观测。"""
        value = max(1, int(value))
        if value == self.view_size:
            return
        self.view_size = value
        self._obs_pos = None
        self._observe(full=True)

    def set_move_mode(self, allow_diagonal: bool) -> None:
        """即时切换 4 向 / 8 向（同时作用于规划与沿墙）。"""
        allow_diagonal = bool(allow_diagonal)
        self.allow_diagonal = allow_diagonal
        self.plan_dirs = DIRS8 if allow_diagonal else DIRS
        self.wall_dirs = DIRS8 if allow_diagonal else WALL_DIRS4
        self.wall_turns = WALL_TURNS8 if allow_diagonal else WALL_TURNS4
        self.entry_dirs = ENTRY_DIRS if allow_diagonal else DIRS
        self._clear_plan()
        self.facing = 0
        self.wall_turn = 0
        self.wall_failed = False
        self.wall_states = {(self.pos, 0)}
        self.mode = MODE_WALL if self.wall_follow else MODE_PLAN

    # ---- 感知（初始全量，之后只观测新进入的一条视野边） ----

    def _observe(self, full: bool = False) -> None:
        """把视野内可见信息并入认知：墙→已知墙，空白→已知可通行。

        灰色障碍对传感器不可见，保持未知；只有撞上后才会被确认。
        非全量模式下只扫描本次移动新进入的那一条视野边（25 格）。
        """
        half = self.view_size // 2
        n0 = len(self.new_explored)
        if full or self._obs_pos is None:
            x0 = self.pos[0] - half
            y0 = self.pos[1] - half
            cells = [(x0 + i, y0 + j)
                     for j in range(self.view_size)
                     for i in range(self.view_size)]
        else:
            ox, oy = self._obs_pos
            px, py = self.pos
            if px == ox and py == oy:
                return
            cells = []
            if px != ox:
                x = px + (half if px > ox else -half)
                cells.extend((x, y) for y in range(py - half, py + half + 1))
            if py != oy:
                y = py + (half if py > oy else -half)
                cells.extend((x, y) for x in range(px - half, px + half + 1))

        for (gx, gy) in cells:
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
                self._grow_reach(cell)
        self._obs_pos = self.pos

        # 增量维护 frontier：只检查本次新观测格及其八邻
        for cell in self.new_explored[n0:]:
            self._update_frontier(cell)
            for dx, dy in DIRS8:
                nb = (cell[0] + dx, cell[1] + dy)
                if self.maze.in_bounds(nb[0], nb[1]):
                    self._update_frontier(nb)

    def _update_frontier(self, cell: tuple[int, int]) -> None:
        """重算某格是否属于 frontier（已知可通行且邻接未观测区域）。"""
        if (self.known.get(cell) == KNOWN_EMPTY
                and self._unexplored_neighbor(cell) is not None):
            self.frontier.add(cell)
        else:
            self.frontier.discard(cell)

    def _mark_blocked(self, x: int, y: int) -> None:
        """记录一次受阻：墙/障碍并入认知，并累计受阻次数。"""
        cell = (x, y)
        if self.maze.is_wall(x, y):
            self.known[cell] = KNOWN_WALL
            self.new_walls.append(cell)
        else:
            self.known[cell] = KNOWN_OBSTACLE
            self.new_obstacles.append(cell)
        self.blocked_hits += 1

    def _grow_reach(self, seed: tuple[int, int]) -> None:
        """新确认可通行的格若接上起点连通集，就增量并入（4-连通洪泛）。

        只做一次均摊 O(1) 的邻接检查；真正连通时顺带把整片已知区域并入，
        因此无需反复对全图做可达性搜索。
        """
        if seed in self.reach or self.known.get(seed) != KNOWN_EMPTY:
            return
        x, y = seed
        if not any((x + dx, y + dy) in self.reach for dx, dy in DIRS):
            return
        stack = [seed]
        self.reach.add(seed)
        while stack:
            cx, cy = stack.pop()
            for dx, dy in DIRS:
                nb = (cx + dx, cy + dy)
                if nb not in self.reach and self.known.get(nb) == KNOWN_EMPTY:
                    self.reach.add(nb)
                    stack.append(nb)

    def _try_move(self, nx: int, ny: int) -> bool:
        """尝试移动一格（允许对角，但禁止切角）。受阻时标记并返回 False。"""
        px, py = self.pos
        dx, dy = nx - px, ny - py
        if dx and dy and not self.allow_diagonal:
            return False  # 四向模式不允许对角
        if self.maze.is_blocked(nx, ny):
            self._mark_blocked(nx, ny)
            return False
        if dx and dy:
            # 禁止切角：两个相邻正交格都必须可通行
            for sx, sy in ((nx, py), (px, ny)):
                if self.maze.is_blocked(sx, sy):
                    self._mark_blocked(sx, sy)
                    return False
        cell = (nx, ny)
        self.known[cell] = KNOWN_EMPTY
        self._grow_reach(cell)
        self.pos = cell
        if cell not in self._visited_set:
            self._visited_set.add(cell)
            self.visited.append(cell)
            self.new_visited.append(cell)
        self._observe()
        return True

    # ---- 规划：方向代价 Dijkstra，目标 = 前沿 ----

    def _direction_cost(self, dx: int, dy: int = 0) -> float:
        if dx > 0:
            base = self.cost_right
        elif dx < 0:
            base = self.cost_left
        else:
            base = self.cost_vertical
        if dx and dy:
            base *= self.diagonal_factor
        return base

    def _unexplored_neighbor(self, cell: tuple[int, int]) -> tuple[int, int] | None:
        """返回 cell 的邻格中「从未被观测过」的那一格（正交优先，对角需不切角）。"""
        x, y = cell
        for dx, dy in self.entry_dirs:
            nx, ny = x + dx, y + dy
            if not self.maze.in_bounds(nx, ny):
                continue
            nb = (nx, ny)
            if self.known.get(nb) is not None or nb in self.explored:
                continue
            if dx and dy:
                if (self.known.get((nx, y)) != KNOWN_EMPTY
                        or self.known.get((x, ny)) != KNOWN_EMPTY):
                    continue
            return nb
        return None

    def _plan_to_goal(self) -> bool:
        """看到红点且已与起点连通时，直接朝它规划（Dijkstra，方向代价相同）。

        终点只是「看到了」但尚未连通时立即返回：避免每次重规划都对整张
        已知地图做一次注定失败的搜索（大视野下会造成明显卡顿）。
        """
        goal = self.goal
        if (goal is None or self.known.get(goal) != KNOWN_EMPTY
                or goal not in self.reach):
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
            for dx, dy in self.plan_dirs:
                nx, ny = cx + dx, cy + dy
                if not self.maze.in_bounds(nx, ny):
                    continue
                nb = (nx, ny)
                # 直达终点只走已确认可通行的格，避开灰色障碍带来的反复试错
                if self.known.get(nb) != KNOWN_EMPTY:
                    continue
                if dx and dy:
                    if (self.known.get((nx, cy)) != KNOWN_EMPTY
                            or self.known.get((cx, ny)) != KNOWN_EMPTY):
                        continue  # 禁止切角
                step_cost = self._direction_cost(dx, dy)
                nd = d + step_cost
                if nd < g_score.get(nb, float("inf")):
                    g_score[nb] = nd
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(heap, (nd, counter, nb))
        return False

    # ---- 前沿聚类 + gain 效用（主策略） ----

    def _frontier_clusters(self) -> list[list[tuple[int, int]]]:
        """把 frontier 格按 4-邻接聚类，过滤过小的碎簇。"""
        frontier = self.frontier
        seen: set[tuple[int, int]] = set()
        clusters: list[list[tuple[int, int]]] = []
        for c in frontier:
            if c in seen:
                continue
            comp: list[tuple[int, int]] = []
            stack = [c]
            seen.add(c)
            while stack:
                q = stack.pop()
                comp.append(q)
                for dx, dy in DIRS:
                    nb = (q[0] + dx, q[1] + dy)
                    if nb in frontier and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
            if len(comp) >= self.cluster_min_size:
                clusters.append(comp)
        return clusters

    def _gain(self, cell: tuple[int, int], radius: int) -> int:
        """信息增益估计：代表周围窗口内尚未观测过的格数。"""
        cnt = 0
        for y in range(cell[1] - radius, cell[1] + radius + 1):
            for x in range(cell[0] - radius, cell[0] + radius + 1):
                if self.maze.in_bounds(x, y) and (x, y) not in self.explored:
                    cnt += 1
        return cnt

    def _plan_cluster(self) -> bool:
        """用一次 Dijkstra 精确评估最近的 K 个前沿簇，再按 gain 效用选目标。"""
        clusters = self._frontier_clusters()
        if not clusters:
            return False
        start = self.pos
        reps = [max(c, key=lambda p: (p[0], -abs(p[1] - start[1])))
                for c in clusters]

        heap: list[tuple[float, int, tuple[int, int]]] = [(0.0, 0, start)]
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        counter = 1
        expanded = 0
        remaining = set(reps)
        found: list[tuple[float, tuple[int, int]]] = []

        while heap and len(found) < self.cluster_top_k:
            d, _, cur = heapq.heappop(heap)
            if d > g_score.get(cur, float("inf")):
                continue
            if cur in remaining:
                found.append((d, cur))
                remaining.discard(cur)
            expanded += 1
            if expanded > self.cluster_expand_limit:
                break
            for dx, dy in self.plan_dirs:
                nb = (cur[0] + dx, cur[1] + dy)
                if not self.maze.in_bounds(nb[0], nb[1]):
                    continue
                if self.known.get(nb) != KNOWN_EMPTY:
                    continue
                if dx and dy:
                    if (self.known.get((nb[0], cur[1])) != KNOWN_EMPTY
                            or self.known.get((cur[0], nb[1])) != KNOWN_EMPTY):
                        continue
                nd = d + self._direction_cost(dx, dy)
                if nd < g_score.get(nb, float("inf")):
                    g_score[nb] = nd
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(heap, (nd, counter, nb))

        if not found:
            return False

        def utility(item: tuple[float, tuple[int, int]]) -> float:
            cost, rep = item
            gain = self._gain(rep, self.cluster_gain_radius)
            dx = rep[0] - start[0]
            return (self.cluster_gain_weight * gain
                    + self.cluster_right_weight * dx
                    - self.cluster_cost_weight * cost)

        _, best = max(found, key=utility)
        entry = self._unexplored_neighbor(best)
        if entry is None:
            return False
        path = [best]
        cur = best
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        self.path = path
        self.path_idx = 0
        self.entry = entry
        return True

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
        if self.cluster_enabled and self._plan_cluster():
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
            for dx, dy in self.plan_dirs:
                nx, ny = cx + dx, cy + dy
                if not self.maze.in_bounds(nx, ny):
                    continue
                nb = (nx, ny)
                state = self.known.get(nb)
                if state == KNOWN_WALL or state == KNOWN_OBSTACLE:
                    continue
                if state is None:
                    if dx and dy:
                        continue
                    if nb not in self.explored:
                        continue  # 未观测格只作为前沿目标
                    step_cost = self._direction_cost(dx, dy) + self.suspect_penalty
                else:
                    if dx and dy:
                        if (self.known.get((nx, cy)) != KNOWN_EMPTY
                                or self.known.get((cx, ny)) != KNOWN_EMPTY):
                            continue
                    step_cost = self._direction_cost(dx, dy)
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

    # ---- 沿墙模式 ----

    def _wall_step(self) -> bool:
        """右手法则走一步。返回 False 表示需要切到规划模式。

        已知墙/已知障碍直接跳过（不盲目尝试）；未知相邻格尝试，
        受阻即标记障碍并换下一个方向。成功后记录 (位置, 朝向) 用于打转检测。
        """
        pos = self.pos
        for k in range(self.wall_turn, len(self.wall_turns)):
            nd = (self.facing + self.wall_turns[k]) % len(self.wall_dirs)
            dx, dy = self.wall_dirs[nd]
            nx, ny = pos[0] + dx, pos[1] + dy
            if not self.maze.in_bounds(nx, ny):
                self.wall_turn = k + 1
                continue
            state = self.known.get((nx, ny))
            if state == KNOWN_WALL or state == KNOWN_OBSTACLE:
                self.wall_turn = k + 1
                continue
            if dx and dy:
                # 已知切角阻挡时直接跳过，避免无谓撞墙
                s1 = self.known.get((nx, pos[1]))
                s2 = self.known.get((pos[0], ny))
                if (s1 == KNOWN_WALL or s1 == KNOWN_OBSTACLE
                        or s2 == KNOWN_WALL or s2 == KNOWN_OBSTACLE):
                    self.wall_turn = k + 1
                    continue
            self.wall_turn = k
            if self._try_move(nx, ny):
                self.facing = nd
                self.wall_turn = 0
                st = (self.pos, self.facing)
                if st in self.wall_states:
                    self.wall_failed = True      # 打转 → 交给规划模式
                    self.mode = MODE_PLAN
                else:
                    self.wall_states.add(st)
            else:
                self.wall_turn = k + 1
            return True
        self.wall_failed = True
        return False

    # ---- 主步进 ----

    def step(self) -> bool:
        """执行一步。返回 True 表示仍在推进。"""
        if self.done:
            return False
        if self.goal is not None:
            if self.pos == self.goal:
                self.done = True  # 已走到看到过的红点
                return False
            self.mode = MODE_PLAN  # 看到红点后直接规划过去
        if self.step_count >= self.max_steps:
            self.done = True
            return False

        self.step_count += 1

        if self.mode == MODE_WALL:
            if self._wall_step():
                return True
            self.mode = MODE_PLAN

        return self._planner_step()

    def _planner_step(self) -> bool:
        """规划模式走一步（沿已知路径前进或踏入前沿）。"""
        if not self.path:
            if not self._plan():
                self.done = True
                return False

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
