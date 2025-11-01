import heapq
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Set, Optional

class Node:
    """Represents a node in the grid for A* pathfinding"""
    def __init__(self, position: Tuple[int, int], parent: Optional['Node'] = None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost from current node to goal
        self.f = 0  # Total cost (g + h)
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)

class AStarPathfinder:
    """A* pathfinding algorithm implementation with visualization"""
    
    def __init__(self, grid: np.ndarray):
        """
        Initialize the pathfinder with a grid
        
        Args:
            grid: 2D numpy array where 0 = walkable, 1 = obstacle
        """
        self.grid = grid
        self.rows = grid.shape[0]
        self.cols = grid.shape[1]
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate Manhattan distance heuristic"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def get_neighbors(self, position: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring positions (up, down, left, right)"""
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        
        for dx, dy in directions:
            new_pos = (position[0] + dx, position[1] + dy)
            
            # Check if position is within bounds and walkable
            if (0 <= new_pos[0] < self.rows and 
                0 <= new_pos[1] < self.cols and 
                self.grid[new_pos[0]][new_pos[1]] == 0):
                neighbors.append(new_pos)
        
        return neighbors
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Find the shortest path from start to goal using A* algorithm
        
        Args:
            start: Starting position (row, col)
            goal: Goal position (row, col)
        
        Returns:
            List of positions representing the path, or None if no path exists
        """
        start_node = Node(start)
        goal_node = Node(goal)
        
        open_list = []
        closed_set: Set[Tuple[int, int]] = set()
        
        heapq.heappush(open_list, start_node)
        
        while open_list:
            current_node = heapq.heappop(open_list)
            closed_set.add(current_node.position)
            
            # Check if we reached the goal
            if current_node == goal_node:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Return reversed path
            
            # Explore neighbors
            for neighbor_pos in self.get_neighbors(current_node.position):
                if neighbor_pos in closed_set:
                    continue
                
                neighbor_node = Node(neighbor_pos, current_node)
                neighbor_node.g = current_node.g + 1
                neighbor_node.h = self.heuristic(neighbor_pos, goal)
                neighbor_node.f = neighbor_node.g + neighbor_node.h
                
                # Check if neighbor is already in open list with better cost
                if any(node.position == neighbor_pos and node.f <= neighbor_node.f 
                       for node in open_list):
                    continue
                
                heapq.heappush(open_list, neighbor_node)
        
        return None  # No path found
    
    def visualize(self, path: Optional[List[Tuple[int, int]]] = None, 
                  start: Optional[Tuple[int, int]] = None, 
                  goal: Optional[Tuple[int, int]] = None):
        """Visualize the grid, obstacles, and path"""
        plt.figure(figsize=(10, 10))
        
        # Create a color map
        display_grid = np.copy(self.grid).astype(float)
        
        # Mark obstacles
        display_grid[self.grid == 1] = 0.3
        
        # Mark path
        if path:
            for pos in path:
                display_grid[pos[0]][pos[1]] = 0.7
        
        # Mark start and goal
        if start:
            display_grid[start[0]][start[1]] = 1.0
        if goal:
            display_grid[goal[0]][goal[1]] = 0.9
        
        plt.imshow(display_grid, cmap='RdYlGn', interpolation='nearest')
        plt.colorbar(label='Legend: Green=Path, Yellow=Start/Goal, Red=Obstacle')
        plt.title('A* Pathfinding Visualization')
        plt.xlabel('Column')
        plt.ylabel('Row')
        plt.grid(True, alpha=0.3)
        plt.show()

# Example usage
if __name__ == "__main__":
    # Create a sample grid (0 = walkable, 1 = obstacle)
    grid = np.array([
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 1, 0, 0, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
        [0, 1, 1, 0, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ])
    
    # Initialize pathfinder
    pathfinder = AStarPathfinder(grid)
    
    # Define start and goal positions
    start = (0, 0)
    goal = (9, 9)
    
    # Find path
    print("Finding path from", start, "to", goal)
    path = pathfinder.find_path(start, goal)
    
    if path:
        print(f"Path found! Length: {len(path)} steps")
        print("Path:", path)
        pathfinder.visualize(path, start, goal)
    else:
        print("No path found!")
        pathfinder.visualize(start=start, goal=goal)
