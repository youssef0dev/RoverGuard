import heapq
import math
from dataclasses import dataclass

from .geometry import Point2D
from .grid_map import GridCell, OccupancyGrid


class PathNotFoundError(RuntimeError):
    pass


@dataclass(frozen=True)
class PlannedPath:
    raw_cells: tuple[GridCell, ...]
    simplified_cells: tuple[GridCell, ...]
    points: tuple[Point2D, ...]
    length_m: float


class AStarPlanner:
    CARDINAL_MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1))
    DIAGONAL_MOVES = ((1, 1), (1, -1), (-1, 1), (-1, -1))

    def __init__(self, grid: OccupancyGrid, allow_diagonal: bool = True):
        self.grid = grid
        self.allow_diagonal = allow_diagonal

    def plan(self, start: Point2D, goal: Point2D) -> PlannedPath:
        start_cell = self.grid.world_to_cell(start)
        goal_cell = self.grid.world_to_cell(goal)

        if not self.grid.is_free(start_cell):
            raise PathNotFoundError(f"Start cell blocked: {start_cell}")
        if not self.grid.is_free(goal_cell):
            raise PathNotFoundError(f"Goal cell blocked: {goal_cell}")

        frontier: list[tuple[float, int, GridCell]] = []
        counter = 0
        heapq.heappush(frontier, (0.0, counter, start_cell))
        came_from: dict[GridCell, GridCell | None] = {start_cell: None}
        cost_so_far: dict[GridCell, float] = {start_cell: 0.0}

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break

            for neighbour, step_cost in self._neighbours(current):
                new_cost = cost_so_far[current] + step_cost
                if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
                    cost_so_far[neighbour] = new_cost
                    priority = new_cost + self._heuristic(neighbour, goal_cell)
                    counter += 1
                    heapq.heappush(frontier, (priority, counter, neighbour))
                    came_from[neighbour] = current

        if goal_cell not in came_from:
            raise PathNotFoundError(f"No safe path from {start} to {goal}")

        raw = self._reconstruct(came_from, goal_cell)
        simplified = self._simplify(raw)
        points = [self.grid.cell_to_world(cell) for cell in simplified]
        points[0] = start
        points[-1] = goal
        length = sum(
            points[index].distance_to(points[index + 1])
            for index in range(len(points) - 1)
        )
        return PlannedPath(tuple(raw), tuple(simplified), tuple(points), length)

    def _neighbours(self, cell: GridCell):
        x, y = cell
        for dx, dy in self.CARDINAL_MOVES:
            neighbour = (x + dx, y + dy)
            if self.grid.is_free(neighbour):
                yield neighbour, 1.0

        if not self.allow_diagonal:
            return

        for dx, dy in self.DIAGONAL_MOVES:
            neighbour = (x + dx, y + dy)
            if not self.grid.is_free(neighbour):
                continue
            if not self.grid.is_free((x + dx, y)):
                continue
            if not self.grid.is_free((x, y + dy)):
                continue
            yield neighbour, math.sqrt(2.0)

    @staticmethod
    def _heuristic(first: GridCell, second: GridCell) -> float:
        dx = abs(first[0] - second[0])
        dy = abs(first[1] - second[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    @staticmethod
    def _reconstruct(
        came_from: dict[GridCell, GridCell | None],
        goal: GridCell,
    ) -> list[GridCell]:
        result = [goal]
        current = goal
        while came_from[current] is not None:
            current = came_from[current]  # type: ignore[assignment]
            result.append(current)
        result.reverse()
        return result

    def _simplify(self, cells: list[GridCell]) -> list[GridCell]:
        if len(cells) <= 2:
            return cells

        result = [cells[0]]
        anchor_index = 0
        while anchor_index < len(cells) - 1:
            furthest = anchor_index + 1
            for candidate in range(anchor_index + 2, len(cells)):
                if self.grid.line_is_free(cells[anchor_index], cells[candidate]):
                    furthest = candidate
                else:
                    break
            result.append(cells[furthest])
            anchor_index = furthest
        return result

