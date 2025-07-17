[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/EVORJHqO)
Dungeon Pathfinder — Final Project
======================================
Generate and solve ASCII dungeons using recursive backtracking and BFS pathfinding algorithms.

---

## Story
You're trapped in a mysterious dungeon and need to find the shortest escape route! Your task is to build an engine that generates random ASCII dungeons with walls, rooms, keys, and doors, then finds the optimal path from start to exit using graph algorithms.

### Problem Variants
1. **Basic Pathfinding** – Find shortest route from S (start) to E (exit)
2. **Key-Door System** – Navigate through locked doors by collecting keys
3. **Perfect Maze Generation** – Create solvable dungeons using recursive backtracking

---

## Files

```
dungeon_pathfinder.pro      Qt project
BITMASK_BFS_GUIDE.md        Educational guide explaining bitmask BFS concepts
src/
  cell.h                    Position structure for dungeon coordinates
  generator.h / .cpp        Maze generation algorithms (with TODOs)
  solver.h / .cpp           BFS pathfinding algorithms (with TODOs)
  main.cpp                  Driver program and test cases
```

---

## Project Features

### Comprehensive Scaffolding
Each file provides extensive helper functions and clear structure:
- **generator.cpp**: Helper functions for coordinate math, carving, room placement - students implement core recursive logic
- **solver.cpp**: Helper functions for pathfinding, neighbor exploration - students implement BFS logic and key-door mechanics
- **main.cpp**: Complete test framework with visualization and validation

### Safety Mechanisms
The starter code includes **anti-hanging safety measures** that prevent infinite loops:
- **Early returns** with TODO messages when functions aren't implemented
- **Iteration counters** to catch infinite loops in BFS
- **Validation checks** to detect incomplete implementations

**[WARNING] IMPORTANT**: You **must remove** the safety code when implementing your solutions!  
Look for comments like `// SAFETY:` and `// TODO:` - remove these lines as you implement.

### Dungeon Elements
- `#` walls and ` ` (space) open floor
- `S` start position and `E` exit position  
- `a‥f` keys and `A‥F` matching doors
- Perfect maze guarantee: exactly one path between any two cells

### Algorithm Features
- **Maze Generation**: Recursive backtracking with random room placement
- **Basic BFS**: Shortest path finding on unweighted grid
- **Extended BFS**: State augmentation with key collection bitmask

---

## Tasks

### 1 · generateDungeon (Recursive Backtracking)

```cpp
std::vector<std::string> generateDungeon(int rows, int cols, int roomRate = 20);
```

**Implementation Requirements:**
* Complete the core recursive backtracking logic
* Helper functions provided for coordinate math and passage carving
* Room placement and start/exit positioning handled automatically

**What You Need to Implement:**
- Core recursive algorithm in `recursiveBacktrack()` function
- Use provided helper functions for neighbor finding and passage carving
- Main maze generation loop with proper initialization

**[WARNING] Important**: Remove the safety code (marked with `// SAFETY:`) when implementing!

**Provided Helper Functions:**
- `getUnvisitedNeighbors()` - finds valid cells to explore
- `carvePassage()` - handles coordinate math for carving
- `addRandomRooms()` - automatically adds rooms after maze generation
- `placeStartAndExit()` - automatically places S and E positions

### 2 · bfsPath (Basic BFS Pathfinding)

```cpp
std::vector<Cell> bfsPath(const std::vector<std::string>& dungeon);
```

**Implementation Requirements:**
* Complete the main BFS exploration loop
* Data structures and setup provided
* Helper functions handle complex logic

**What You Need to Implement:**
- Main BFS loop: dequeue, check goal, explore neighbors
- Use provided helper functions for neighbor exploration
- Basic queue and visited set management

**[WARNING] Important**: Remove the safety code (marked with `// SAFETY:`) when implementing!

**Provided Helper Functions:**
- `findPosition()` - locates start and exit positions
- `getNeighbors()` - finds all valid neighboring cells
- `reconstructPath()` - handles path reconstruction from parents
- `isPassable()` - checks if a position can be entered

### 3 · bfsPathKeys (Extended BFS with Keys & Doors)

```cpp
std::vector<Cell> bfsPathKeys(const std::vector<std::string>& dungeon);
```

**Implementation Requirements:**
* Complete door checking logic using helper functions
* Implement key collection when stepping on keys
* Create proper KeyState with updated key information
* Core BFS infrastructure provided

**What You Need to Implement:**
- Door permission checking (can we pass door 'A' with key 'a'?)
- Key collection logic (update keyMask when finding keys)
- State creation with correct key information

**[WARNING] Important**: This function has 3 specific TODOs marked in the code!

**Provided Infrastructure:**
- State augmentation framework (KeyState structure)
- BFS loop and neighbor exploration
- Parent tracking and path reconstruction
- Helper functions for door/key operations

---

## Build / Run

```bash
qmake
make
./dungeon_pathfinder
```

**Default size is 21×41.** The program generates a dungeon and attempts to solve it.

### Expected Output
```
Generated dungeon:
#########
#S#     #
# ##### #
#     # #
##### # #
# #   # #
# # ##  #
#      E#
#########

Solved path (length 17):
#########
#S#     #
#*##### #
#*****# #
#####*# #
# #***# #
# #*##  #
#  ****E#
#########
```

---

## Algorithm Details

### Maze Generation - Recursive Backtracking
- **Perfect Maze**: Guarantees exactly one simple path between any two points
- **Random Exploration**: Uses shuffled directions for maze variety
- **Room Addition**: Randomly punches extra openings for strategic gameplay

### BFS Pathfinding
- **Optimal Solution**: BFS guarantees shortest path on unweighted grids
- **Level-by-Level**: Explores all positions at distance d before d+1
- **State Augmentation**: Extends basic BFS to handle game mechanics

### Implementation Complexity
- **Maze Generation**: ~20 lines of recursive logic (helpers provided)
- **Basic BFS**: ~15 lines of main loop (data structures provided)
- **Key-Door BFS**: ~10 lines across 3 TODOs (infrastructure provided)

### Algorithm Complexity
- **Generation**: O(rows × cols) for maze carving
- **Basic BFS**: O(V + E) where V is open cells, E is connections
- **Key BFS**: O(V × 2^k) where k is number of keys (up to 6)

---

## Data Structures

### Cell Structure
```cpp
struct Cell {
    int r, c;  // row, column coordinates
    bool operator==(const Cell& other) const;
};
```

### State Encoding (Key BFS)
- **Position**: (row, col) coordinates
- **Key Mask**: 6-bit integer for collected keys
- **Encoding**: `((keyMask<<16) | (row<<8) | col)` fits in 32 bits

---

## Tips & Development Notes

### Implementation Strategy
- Test with small mazes (7×7) first for easier debugging
- Focus on core algorithm logic - helper functions handle details
- Use provided debugging output to trace algorithm progress
- Follow the clear TODO structure in each function

### [WARNING] Removing Safety Code
**CRITICAL**: When implementing your solutions, you **must remove** the safety mechanisms:

1. **Look for these patterns** in your code:
   ```cpp
   // ==================== REMOVE THIS SAFETY CODE WHEN IMPLEMENTING ====================
   // Safety code here...
   return;  // <- REMOVE THIS LINE
   // ==================== END SAFETY CODE TO REMOVE ====================
   ```

2. **Delete the entire safety sections** - they're only there to prevent hanging

3. **Test frequently** after removing safety code to catch infinite loops early

### Memory Management
- Use STL containers (no raw pointers)
- No memory leaks
- Clean resource management
- Proper error handling for edge cases

### Debugging
- Visualize intermediate steps
- Verify maze connectivity
- Check key collection logic
- Test with different dungeon sizes

---

## Grading Breakdown

| Component | Points |
|-----------|--------|
| Maze generator (perfect maze + rooms) | 6 |
| Basic BFS pathfinding (S→E) | 8 |
| Extended BFS with keys & doors | 10 |
| Clean code, decomposition, comments | 4 |
| No memory leaks / proper I/O | 2 |
| **Total** | 30 |

## Submission

Submit only the implementation files:
```
src/generator.cpp
src/solver.cpp
src/generator.h    (if you added helpers)
src/solver.h       (if you added helpers)
```

**Do not submit** binaries, object files, or project configuration files.

