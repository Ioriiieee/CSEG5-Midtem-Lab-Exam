from queue import PriorityQueue
import time

# Function to print the grid with the path if found
def print_grid(grid, path=None):
    for i, row in enumerate(grid):
        row_str = ""
        for j, val in enumerate(row):
            if path and (i, j) in path:
                row_str += " * "  # Mark the path
            elif val == 1:
                row_str += " # "  # Obstacle
            else:
                row_str += " . "  # Open cell
        print(row_str)
    print()

# Function to calculate the Manhattan distance heuristic
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Greedy Best-First Search implementation
def greedy_best_first_search(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    came_from[start] = None
    visited = set()

    while not open_set.empty():
        _, current = open_set.get()
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        visited.add(current)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                if neighbor not in visited:
                    open_set.put((heuristic(neighbor, goal), neighbor))
                    came_from[neighbor] = current
                    visited.add(neighbor)
    
    return None  # No path found

# A* Search implementation
def a_star_search(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    came_from[start] = None

    while not open_set.empty():
        _, current = open_set.get()
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    open_set.put((f_score, neighbor))
                    came_from[neighbor] = current
    
    return None  # No path found

# Main function to set up the grid, take user input, and run the algorithms
def main():
    # Get grid dimensions from the user
    rows = int(input("Enter the number of rows: "))
    cols = int(input("Enter the number of columns: "))

    # Initialize an empty grid
    grid = []
    print("\nEnter the grid row by row (use '0' for open cells and '1' for obstacles):")
    
    # Get grid rows from the user
    for i in range(rows):
        row = input(f"Row {i + 1}: ").split()
        grid.append([int(cell) for cell in row])

    print("\nInitial grid:")
    print_grid(grid)
    
    # Get start and goal positions from the user
    try:
        start_x, start_y = map(int, input("Enter start coordinates (x y): ").split())
        goal_x, goal_y = map(int, input("Enter goal coordinates (x y): ").split())
        start, goal = (start_x, start_y), (goal_x, goal_y)
    except ValueError:
        print("Invalid input. Using default start (0, 0) and goal (4, 4).")
        start, goal = (0, 0), (4, 4)

    # Run Greedy Best-First Search
    print("\nRunning Greedy Best-First Search...")
    start_time = time.time()
    path_greedy = greedy_best_first_search(grid, start, goal)
    time_greedy = time.time() - start_time
    print("Greedy Best-First Search:")
    print("Path found:", path_greedy)
    print("Time taken: {:.6f} seconds".format(time_greedy))
    print_grid(grid, path_greedy)

    # Run A* Search
    print("\nRunning A* Search...")
    start_time = time.time()
    path_a_star = a_star_search(grid, start, goal)
    time_a_star = time.time() - start_time
    print("A* Search:")
    print("Path found:", path_a_star)
    print("Time taken: {:.6f} seconds".format(time_a_star))
    print_grid(grid, path_a_star)

    # Path Quality Comparison
    print("\nPath Quality Comparison:")
    print("Length of path found by Greedy:", len(path_greedy) if path_greedy else "No path found")
    print("Length of path found by A*:", len(path_a_star) if path_a_star else "No path found")

if __name__ == "__main__":
    main()
