from typing import List, Tuple, Dict, Optional
import heapq
from math import inf
import numpy as np 
import matplotlib.pyplot as plt


# =========================
# 1. Cấu hình chi phí ô
# =========================
CELL_COST = {
    0: 1,   # ô trống
    1: None, # vật cản
    2: 3,   # bùn lầy
    3: 5    # đá
}


# =========================
# 2. Helper functions
# =========================
def create_node(position: Tuple[int, int],
                g: float = inf,
                h: float = 0.0,
                parent: Optional[Dict] = None) -> Dict:
    return {
        "position": position,
        "g": g,
        "h": h,
        "f": g + h,
        "parent": parent
    }


def calculate_heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
    """
    Heuristic cho 4 hướng: Manhattan
    Vì chi phí nhỏ nhất để đi 1 ô là 1, nên dùng Manhattan * 1
    để heuristic không vượt quá chi phí thực.
    """
    x1, y1 = pos1
    x2, y2 = pos2
    return abs(x1 - x2) + abs(y1 - y2)


def get_valid_neighbors(grid: np.ndarray, position: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    Chỉ xét 4 hướng: lên, xuống, trái, phải
    """
    x, y = position
    rows, cols = grid.shape

    possible_moves = [
        (x - 1, y),  # lên
        (x + 1, y),  # xuống
        (x, y - 1),  # trái
        (x, y + 1),  # phải
    ]

    neighbors = []
    for nx, ny in possible_moves:
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] != 1:
            neighbors.append((nx, ny))
    return neighbors


def get_move_cost(grid: np.ndarray, position: Tuple[int, int]) -> float:
    """
    Chi phí để robot bước vào ô position.
    """
    cell_value = int(grid[position[0], position[1]])
    if cell_value not in CELL_COST or CELL_COST[cell_value] is None:
        return inf
    return CELL_COST[cell_value]


def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int]]:
    path = []
    current = goal_node
    while current is not None:
        path.append(current["position"])
        current = current["parent"]
    return path[::-1]


# =========================
# 3. A* algorithm
# =========================
def find_path(grid: np.ndarray,
              start: Tuple[int, int],
              goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    Tìm đường đi tối ưu bằng A*.
    """
    if grid[start[0], start[1]] == 1 or grid[goal[0], goal[1]] == 1:
        return []

    start_node = create_node(
        position=start,
        g=0,
        h=calculate_heuristic(start, goal),
        parent=None
    )

    open_heap = []
    heapq.heappush(open_heap, (start_node["f"], start))
    open_dict = {start: start_node}
    closed_set = set()

    while open_heap:
        _, current_pos = heapq.heappop(open_heap)

        if current_pos in closed_set:
            continue

        current_node = open_dict[current_pos]

        if current_pos == goal:
            return reconstruct_path(current_node)

        closed_set.add(current_pos)

        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            if neighbor_pos in closed_set:
                continue

            step_cost = get_move_cost(grid, neighbor_pos)
            if step_cost == inf:
                continue

            tentative_g = current_node["g"] + step_cost

            if neighbor_pos not in open_dict:
                neighbor_node = create_node(
                    position=neighbor_pos,
                    g=tentative_g,
                    h=calculate_heuristic(neighbor_pos, goal),
                    parent=current_node
                )
                open_dict[neighbor_pos] = neighbor_node
                heapq.heappush(open_heap, (neighbor_node["f"], neighbor_pos))

            elif tentative_g < open_dict[neighbor_pos]["g"]:
                neighbor_node = open_dict[neighbor_pos]
                neighbor_node["g"] = tentative_g
                neighbor_node["f"] = tentative_g + neighbor_node["h"]
                neighbor_node["parent"] = current_node
                heapq.heappush(open_heap, (neighbor_node["f"], neighbor_pos))

    return []


# =========================
# 4. Hiển thị kết quả
# =========================
def visualize_path(grid: np.ndarray, path: List[Tuple[int, int]]) -> None:
    """
    In grid ra dạng ký hiệu:
    0: trống
    1: tường
    2: bùn lầy
    3: đá
    *: đường đi
    S: start
    G: goal
    """
    grid_copy = np.copy(grid)

    path_set = set(path)

    for i, row in enumerate(grid_copy):
        line = []
        for j, cell in enumerate(row):
            if (i, j) == path[0]:
                line.append("S")
            elif (i, j) == path[-1]:
                line.append("G")
            elif (i, j) in path_set:
                line.append("*")
            else:
                line.append(str(int(cell)))
        print(" ".join(line))


def plot_grid(grid: np.ndarray, path: List[Tuple[int, int]]) -> None:
    """
    Vẽ lưới và đường đi.
    """
    fig, ax = plt.subplots()

    ax.imshow(grid, interpolation="none")

    if path:
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        ax.plot(path_x, path_y, marker="o", linewidth=2, label="Path")

        start = path[0]
        goal = path[-1]
        ax.plot(start[1], start[0], marker="s", markersize=10, label="Start")
        ax.plot(goal[1], goal[0], marker="s", markersize=10, label="Goal")

    ax.set_title("A* Path on Warehouse Grid")
    ax.legend()
    plt.show()


# =========================
# 5. Main
# =========================
def main():
    grid = np.array([
        [0, 0, 0, 0, 0, 1, 0, 0],
        [0, 2, 2, 0, 0, 1, 0, 3],
        [0, 1, 0, 0, 2, 1, 0, 3],
        [0, 1, 0, 1, 2, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 3, 0],
        [1, 1, 0, 1, 0, 2, 3, 0],
        [0, 0, 0, 0, 0, 2, 0, 0],
        [0, 3, 3, 0, 0, 0, 0, 0],
    ])

    start_pos = (0, 0)
    goal_pos = (7, 7)

    path = find_path(grid, start_pos, goal_pos)

    if path:
        total_cost = 0
        for p in path[1:]:
            total_cost += get_move_cost(grid, p)

        print(f"Đã tìm thấy đường đi tối ưu!")
        print(f"Số bước: {len(path) - 1}")
        print(f"Tổng chi phí: {total_cost}\n")

        visualize_path(grid, path)
        plot_grid(grid, path)
    else:
        print("Không tìm thấy đường đi phù hợp.")


if __name__ == "__main__":
    main()