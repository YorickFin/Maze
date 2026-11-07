"""破解方（在线 A* 求解器）。

破解方不知道终点坐标，只知道起点在左边界、目标是到达右边界；
视野 view_size×view_size 内障碍不可见，只能通过「尝试移动」获得
受阻反馈，从而逐步推断障碍位置。

策略：在「已知可通行 + 未知」图上朝右边界做 A*（启发偏向右），
未知格允许通行但代价略高，鼓励走已确认安全的格；撞墙或撞灰色障碍
受阻则标记为已知阻挡物并重新规划。如此反复直到到达右边界或确认
无法更右。
"""

from __future__ import annotations

import heapq

from .maze import EMPTY, WALL, Maze

# 破解方对格子的认知状态
UNKNOWN = 0
KNOWN_EMPTY = 1
KNOWN_WALL = 2

# 四邻域（上下左右）
DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))


class Solver:
    """在线探索破解方。"""

    def __init__(self, maze: Maze, view_size: int, max_steps: int):
        self.maze = maze
        self.view_size = view_size
        self.max_steps = max_steps

        # 认知网格：UNKNOWN / KNOWN_EMPTY / KNOWN_WALL
        self.known: dict[tuple[int, int], int] = {}
        self.pos = maze.start
        self.visited: set[tuple[int, int]] = {maze.start}  # 实际到达过的格
        self.explored: set[tuple[int, int]] = set()  # 视野扫过的格（已感知）
        self.step_count = 0
        self.done = False
        self.blocked_hits = 0  # 受阻次数
        self.known[maze.start] = KNOWN_EMPTY
        self._observe()  # 初始视野

        # 当前待走路径（格序列，含当前格在首）
        self.path: list[tuple[int, int]] = [maze.start]
        self.path_idx = 0

    def _observe(self) -> None:
        """把当前位置 view_size×view_size 视野内的格子加入 explored。

        注意：视野内障碍「不可见」（用户要求），所以只记录该格被感知到
        （作为「已扫过区域」），不读取真实墙信息——障碍仍需靠移动受阻推断。
        """
        half = self.view_size // 2
        x0 = self.pos[0] - half
        y0 = self.pos[1] - half
        for dy in range(self.view_size):
            for dx in range(self.view_size):
                gx, gy = x0 + dx, y0 + dy
                if self.maze.in_bounds(gx, gy):
                    self.explored.add((gx, gy))

    # ---- 感知与移动 ----

    def _try_move(self, nx: int, ny: int) -> bool:
        """尝试移动到 (nx,ny)。受阻（墙或灰色障碍）返回 False 并记录。"""
        if self.maze.is_blocked(nx, ny):
            self.known[(nx, ny)] = KNOWN_WALL
            self.blocked_hits += 1
            return False
        self.known[(nx, ny)] = KNOWN_EMPTY
        self.pos = (nx, ny)
        self.visited.add((nx, ny))
        self._observe()  # 移动后更新视野感知
        return True

    # ---- A* 寻路（朝右边界）----

    def _astar_right(self, max_expand: int = 60000, window_w: int = 400,
                     window_back: int = 300) -> list[tuple[int, int]]:
        """在「已知 EMPTY + 未知」图上朝右边界做 A*，限制在 x 窗口内。

        窗口 x ∈ [pos.x - window_back, pos.x + window_w]，允许向后回退绕行
        （房间式迷宫需要绕行找门）。未知格允许通行但代价略高；已知 WALL
        禁止。返回路径（含当前格），不可达或扩展超限返回空列表。
        """
        target_x = self.maze.width - 1
        start = self.pos
        if start[0] == target_x:
            return [start]

        x_min = max(0, start[0] - window_back)
        x_max = min(target_x, start[0] + window_w)

        open_heap: list[tuple[float, int, tuple[int, int]]] = []
        counter = 0
        heapq.heappush(open_heap, (0.0, counter, start))
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        closed: set[tuple[int, int]] = set()
        expanded = 0

        def h(cell: tuple[int, int]) -> float:
            # 到右边界的曼哈顿距离 + 向右偏好
            return float(target_x - cell[0]) - 0.01 * cell[0]

        def cost(cell: tuple[int, int]) -> float:
            return 1.0 if self.known.get(cell) == KNOWN_EMPTY else 1.5

        while open_heap:
            f, _, cur = heapq.heappop(open_heap)
            if cur[0] == target_x:
                path = [cur]
                while cur in came_from:
                    cur = came_from[cur]
                    path.append(cur)
                path.reverse()
                return path
            if cur in closed:
                continue
            closed.add(cur)
            expanded += 1
            if expanded > max_expand:
                return []
            for dx, dy in DIRS:
                nx, ny = cur[0] + dx, cur[1] + dy
                nb = (nx, ny)
                if nb in closed:
                    continue
                if not self.maze.in_bounds(nx, ny):
                    continue
                # 限制在 x 窗口内（允许向后绕行）
                if nx < x_min or nx > x_max:
                    continue
                if self.known.get(nb) == KNOWN_WALL:
                    continue
                tentative = g_score[cur] + cost(nb)
                if tentative < g_score.get(nb, float("inf")):
                    g_score[nb] = tentative
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(open_heap, (tentative + h(nb), counter, nb))
        return []

    # ---- 主步进 ----

    def step(self) -> bool:
        """执行一步探索。返回 True 表示仍在推进，False 表示结束（到达或卡死）。"""
        if self.done:
            return False
        if self.step_count >= self.max_steps:
            self.done = True
            return False

        # 到达右边界 → 成功
        if self.pos[0] == self.maze.width - 1:
            self.done = True
            return False

        # 沿当前路径走（下一格若已知 EMPTY 直接走；若是未知/墙则试探）
        if self.path_idx + 1 < len(self.path):
            nxt = self.path[self.path_idx + 1]
            ok = self._try_move(nxt[0], nxt[1])
            self.path_idx += 1
            self.step_count += 1
            if not ok:
                # 受阻：标记已发生，重规划
                self._replan()
            return True

        # 无有效路径 → 重新规划
        return self._replan()

    def _replan(self) -> bool:
        """重新规划朝右的 A* 路径。返回 True 若执行了一次有效移动。"""
        path = self._astar_right()
        if len(path) > 1:
            self.path = path
            self.path_idx = 0
            nxt = self.path[1]
            ok = self._try_move(nxt[0], nxt[1])
            self.path_idx = 1
            self.step_count += 1
            if not ok:
                # 撞墙，标记已发生，下次重规划
                self.path = [self.pos]
                self.path_idx = 0
            return True

        # A* 无法朝右推进：尝试从当前格向四邻试探（优先右）
        x, y = self.pos
        order = [(x + 1, y), (x, y - 1), (x, y + 1), (x - 1, y)]
        for (nx, ny) in order:
            if not self.maze.in_bounds(nx, ny):
                continue
            if self.known.get((nx, ny)) == KNOWN_WALL:
                continue
            if self.known.get((nx, ny)) == KNOWN_EMPTY:
                continue
            ok = self._try_move(nx, ny)
            self.step_count += 1
            if ok:
                self.path = [self.pos]
                self.path_idx = 0
                return True
        # 四邻均受阻/已探索 → 卡死
        self.done = True
        return False

    @property
    def reached_end(self) -> bool:
        """是否已到达右边界。"""
        return self.pos[0] == self.maze.width - 1

    def frontier(self) -> list[tuple[int, int]]:
        """已探索（到达过）的格列表，供左栏合成图。"""
        return list(self.visited)
