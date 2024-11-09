import math
import matplotlib.pyplot as plt

class DijkstraPlanner:
    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y

        self.Delta_C1 = 0.3
        self.Delta_C2 = 0.15
        self.costPerGrid = 1

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Goal found!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid, c_id)

                if self.calc_grid_position(node.x, self.min_x) in self.tc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.tc_y:
                        node.cost += self.Delta_C1 * self.motion[i][2]

                if self.calc_grid_position(node.x, self.min_x) in self.fc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.fc_y:
                        node.cost += self.Delta_C2 * self.motion[i][2]

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry, total_cost = self.calc_final_path(goal_node, closed_set)
        print(f"Total path cost: {total_cost:.4f}")  # Print the total path cost
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        total_cost = goal_node.cost
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry, total_cost

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                  [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]
        return motion

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break


def main():
    sx, sy, gx, gy = 50.0, 50.0, -10.0, 20.0
    grid_size, robot_radius = 1.0, 1.0
    ox, oy = [], []
    for i in range(-12, 60): ox.append(i); oy.append(-10.0)
    for i in range(-10, 60): ox.append(60.0); oy.append(i)
    for i in range(-12, 60): ox.append(i); oy.append(60.0)
    for i in range(-10, 60): ox.append(-12.0); oy.append(i)
    for i in range(10, 20): ox.append(i); oy.append(-2 * i + 60)
    for i in range(20, 30): ox.append(i); oy.append(2 * i - 40)
    for i in range(35, 60): ox.append(40); oy.append(i)
    tc_x, tc_y, fc_x, fc_y = [], [], [], []
    for i in range(0, 10): tc_x += [i] * 40; tc_y += list(range(-10, 30))
    for i in range(40, 60): fc_x += [i] * 20; fc_y += list(range(10, 30))

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.plot(fc_x, fc_y, "oy")
    plt.plot(tc_x, tc_y, "or")
    plt.grid(True)
    plt.axis("equal")

    planner = DijkstraPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    rx, ry = planner.planning(sx, sy, gx, gy)

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == '__main__':
    main()
