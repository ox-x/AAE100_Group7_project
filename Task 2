import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq

# Define the grid size
grid_size = (70, 70)

# Define the start and goal positions
start = (-10, 20)
goal = (50, 50)

# Define obstacles as a set of coordinates
obstacles = {
    (x, y) for x in range(10, 21) for y in range(20, 41)
}.union({
    (40, y) for y in range(35, 61)
}).union({
    (x, 20) for x in range(20, 31)
}).union({
    (x, y) for x in range(10, 21) for y in range(-10, 1)
})

# Define high-cost areas
fuel_area = {(x, y) for x in range(40, 61) for y in range(10, 61)}
time_area = {(x, y) for x in range(0, 11) for y in range(0, 31)}

# Heuristic function (Euclidean distance)
def heuristic(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

# A* algorithm
def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in obstacles or not (0 <= neighbor[0] < grid_size[0] and 0 <= neighbor[1] < grid_size[1]):
                continue

            tentative_g_score = g_score[current] + 1
            if neighbor in fuel_area:
                tentative_g_score += 5  # Increase cost for fuel area
            if neighbor in time_area:
                tentative_g_score += 3  # Increase cost for time area

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

# Find the path
path = a_star(start, goal)

# Plotting
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-20, 70)
ax.set_ylim(-20, 70)
ax.set_xticks(range(-20, 70, 10))
ax.set_yticks(range(-20, 70, 10))
ax.grid(True)

# Plot Start and Goal nodes
ax.plot(*start, 'g*', markersize=15, label='Goal node')
ax.plot(*goal, 'b*', markersize=15, label='Start node')

# Plot Fuel-consuming area
fuel_area_patch = patches.Rectangle((40, 10), 20, 50, linewidth=1, edgecolor='gray', facecolor='khaki', label='Fuel-consuming area')
ax.add_patch(fuel_area_patch)

# Plot Time-consuming area
time_area_patch = patches.Rectangle((0, 0), 10, 30, linewidth=1, edgecolor='gray', facecolor='pink', label='Time-consuming area')
ax.add_patch(time_area_patch)

# Plot obstacles
for obs in obstacles:
    ax.plot(*obs, 'ks', markersize=5)

# Plot path
if path:
    path_x, path_y = zip(*path)
    ax.plot(path_x, path_y, 'c-', linewidth=2, label='Path')

# Add legend
ax.legend(loc='upper left')

# Show plot
plt.title('A* Path-Planning Simulation')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.show()
