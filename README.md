# 🔍 A* Pathfinding Algorithm

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

A professional implementation of the A* (A-Star) pathfinding algorithm in Python, featuring real-time visualization and optimized performance for finding shortest paths in grid-based environments.

## 📋 Project Overview

This project provides a clean, efficient, and well-documented implementation of the A* pathfinding algorithm. A* is one of the most popular pathfinding algorithms used in games, robotics, and navigation systems. It combines the benefits of Dijkstra's algorithm with heuristic-based search to efficiently find the shortest path between two points.

The implementation includes:
- **Efficient pathfinding** using priority queues (heapq)
- **Visual representation** of the grid, obstacles, and discovered path
- **Customizable grid environments** with obstacle placement
- **Manhattan distance heuristic** for optimal grid-based pathfinding
- **Clean, object-oriented design** for easy integration

## ✨ Features

- **🎯 Optimal Path Finding**: Guaranteed to find the shortest path if one exists
- **📊 Visual Feedback**: Built-in matplotlib visualization showing:
  - Grid layout
  - Obstacles (red)
  - Discovered path (green)
  - Start and goal positions (yellow/green gradient)
- **⚡ Optimized Performance**: Uses heap queue for efficient node selection
- **🔧 Flexible Grid System**: Easy to customize grid size and obstacle placement
- **📝 Well-Documented Code**: Clear documentation and type hints throughout
- **🧪 Example Usage**: Includes working example with sample grid

## 🛠️ Tech Stack

- **Language**: Python 3.8+
- **Core Libraries**:
  - `heapq` - Priority queue implementation for efficient pathfinding
  - `numpy` - Grid representation and array operations
  - `matplotlib` - Path visualization and plotting
  - `typing` - Type hints for better code clarity

## 🚀 Usage

### Basic Example

```python
import numpy as np
from astar import AStarPathfinder

# Create a grid (0 = walkable, 1 = obstacle)
grid = np.array([
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0]
])

# Initialize pathfinder
pathfinder = AStarPathfinder(grid)

# Define start and goal positions
start = (0, 0)  # Top-left corner
goal = (4, 4)   # Bottom-right corner

# Find the shortest path
path = pathfinder.find_path(start, goal)

if path:
    print(f"Path found! Length: {len(path)} steps")
    print("Path:", path)
    # Visualize the result
    pathfinder.visualize(path, start, goal)
else:
    print("No path found!")
```

### Advanced Usage

```python
# Create a custom grid with more obstacles
custom_grid = np.zeros((20, 20))
# Add vertical wall
custom_grid[5:15, 10] = 1
# Add horizontal wall with gap
custom_grid[10, 5:9] = 1

pathfinder = AStarPathfinder(custom_grid)
path = pathfinder.find_path((0, 0), (19, 19))

if path:
    pathfinder.visualize(path, (0, 0), (19, 19))
```

## 📦 Installation

### Prerequisites

Ensure you have Python 3.8 or higher installed on your system.

### Step 1: Clone the Repository

```bash
git clone https://github.com/rahulkumar7189/astar-pathfinding.git
cd astar-pathfinding
```

### Step 2: Install Dependencies

```bash
pip install numpy matplotlib
```

Or install from requirements file (if available):

```bash
pip install -r requirements.txt
```

### Step 3: Run the Example

```bash
python astar.py
```

This will run the built-in example and display a visualization of the pathfinding result.

## 🎓 How A* Algorithm Works

The A* algorithm uses a best-first search strategy, combining:

1. **g(n)**: The actual cost from the start node to the current node
2. **h(n)**: The estimated cost from the current node to the goal (heuristic)
3. **f(n) = g(n) + h(n)**: The total estimated cost

The algorithm maintains two lists:
- **Open List**: Nodes to be evaluated
- **Closed Set**: Nodes already evaluated

At each step, it selects the node with the lowest f(n) value, ensuring optimal pathfinding.

## 📁 Project Structure

```
astar-pathfinding/
│
├── astar.py              # Main implementation file
├── README.md             # Project documentation
├── LICENSE               # MIT License
└── .gitignore           # Python gitignore
```

## 🔍 Algorithm Details

- **Time Complexity**: O(b^d) where b is branching factor and d is depth
- **Space Complexity**: O(b^d) for storing nodes
- **Optimality**: Yes, when using an admissible heuristic
- **Completeness**: Yes, will find a solution if one exists
- **Heuristic Used**: Manhattan distance (optimal for grid-based movement)

## 🤝 Contributing

Contributions are welcome! Feel free to:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

The MIT License is a permissive license that allows you to use, modify, and distribute this code for any purpose, including commercial applications.

## 📞 Contact

**Rahul Kumar**

- GitHub: [@rahulkumar7189](https://github.com/rahulkumar7189)
- Repository: [astar-pathfinding](https://github.com/rahulkumar7189/astar-pathfinding)

For questions, suggestions, or collaboration opportunities, feel free to open an issue or reach out!

## 🌟 Acknowledgments

- Inspired by classic pathfinding algorithms used in game development and robotics
- Built with clean code principles and best practices
- Thanks to the open-source community for continuous inspiration

---

⭐ If you find this project helpful, please consider giving it a star on GitHub! ⭐
