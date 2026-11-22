import math
from dataclasses import dataclass, field

from .geometry import AABB, Point2D


GridCell = tuple[int, int]


@dataclass
class OccupancyGrid:
    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float
    resolution_m: float
    occupied: set[GridCell] = field(default_factory=set)

    @property
    def width(self) -> int:
        return math.ceil((self.max_x_m - self.min_x_m) / self.resolution_m)

    @property
    def height(self) -> int:
        return math.ceil((self.max_y_m - self.min_y_m) / self.resolution_m)

    def in_bounds(self, cell: GridCell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: GridCell) -> bool:
        return self.in_bounds(cell) and cell not in self.occupied

    def world_to_cell(self, point: Point2D) -> GridCell:
        return (
            math.floor((point.x - self.min_x_m) / self.resolution_m),
            math.floor((point.y - self.min_y_m) / self.resolution_m),
        )

    def cell_to_world(self, cell: GridCell) -> Point2D:
        return Point2D(
            self.min_x_m + (cell[0] + 0.5) * self.resolution_m,
            self.min_y_m + (cell[1] + 0.5) * self.resolution_m,
        )

    def add_obstacle(self, obstacle: AABB, inflation_m: float) -> None:
        inflated = obstacle.inflated(inflation_m)
        min_cell = self.world_to_cell(Point2D(inflated.min_x, inflated.min_y))
        max_cell = self.world_to_cell(Point2D(inflated.max_x, inflated.max_y))

        for x in range(min_cell[0], max_cell[0] + 1):
            for y in range(min_cell[1], max_cell[1] + 1):
                cell = (x, y)
                if self.in_bounds(cell) and inflated.contains(self.cell_to_world(cell)):
                    self.occupied.add(cell)

    def line_is_free(self, start: GridCell, end: GridCell) -> bool:
        """Bresenham line check used to simplify the A* path safely."""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy

        previous = (x0, y0)
        while True:
            current = (x0, y0)
            if not self.is_free(current):
                return False

            # A diagonal segment cannot squeeze through two touching obstacles.
            if current != previous:
                delta_x = current[0] - previous[0]
                delta_y = current[1] - previous[1]
                if delta_x and delta_y:
                    if not self.is_free((previous[0] + delta_x, previous[1])):
                        return False
                    if not self.is_free((previous[0], previous[1] + delta_y)):
                        return False
            previous = current

            if x0 == x1 and y0 == y1:
                return True
            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x0 += step_x
            if twice_error <= dx:
                error += dx
                y0 += step_y

