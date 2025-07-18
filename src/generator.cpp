/**
 * Dungeon Pathfinder - Maze Generation
 *
 * This file implements recursive backtracking maze generation.
 * The algorithm creates a "perfect maze" - exactly one path between any two points.
 */

#include "generator.h"
#include <random>
#include <algorithm>
#include <iostream>

using namespace std;

// Directions for maze carving: North, East, South, West (2-step moves)
const int CARVE_DIRECTIONS[4][2] = {{-2, 0}, {0, 2}, {2, 0}, {0, -2}};

// Random number generator (better than rand())
static mt19937 rng(random_device{}());

/**
 * Helper function: Check if a coordinate is within bounds
 */
bool isInBounds(int row, int col, int rows, int cols) {
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

/**
 * Helper function: Check if a cell has been visited (carved)
 * In our representation, carved cells contain ' ' (space)
 */
bool isCarved(const vector<string>& maze, int row, int col) {
    return maze[row][col] == ' ';
}

/**
 * Helper function: Carve a passage between two cells
 * This removes the wall between start and end positions
 */
void carvePassage(vector<string>& maze, int startRow, int startCol, int endRow, int endCol) {
    // Carve the destination cell
    maze[endRow][endCol] = ' ';

    // Carve the wall between start and end (the cell in the middle)
    int wallRow = (startRow + endRow) / 2;
    int wallCol = (startCol + endCol) / 2;
    maze[wallRow][wallCol] = ' ';
}

/**
 * Helper function: Get all valid unvisited neighbors
 * Returns list of cells that can be carved to (2 steps away and not yet carved)
 */
vector<pair<int, int>> getUnvisitedNeighbors(const vector<string>& maze, int row, int col) {
    vector<pair<int, int>> neighbors;
    size_t rows = maze.size();
    size_t cols = maze[0].size();

    // Check all 4 directions (2 steps away)
    for (int i = 0; i < 4; i++) {
        int newRow = row + CARVE_DIRECTIONS[i][0];
        int newCol = col + CARVE_DIRECTIONS[i][1];

        // Cast size_t to int to avoid signed/unsigned mismatch in isInBounds
        if (isInBounds(newRow, newCol, static_cast<int>(rows), static_cast<int>(cols)) && !isCarved(maze, newRow, newCol)) {
            neighbors.push_back({newRow, newCol});
        }
    }

    return neighbors;
}

/**
 * Helper function: Shuffle a vector randomly
 */
template<typename T>
void shuffleVector(vector<T>& vec) {
    shuffle(vec.begin(), vec.end(), rng);
}

/**
 * TODO: Implement recursive backtracking maze carving
 *
 * ALGORITHM:
 * 1. Mark current cell as carved (set to ' ')
 * 2. Get all unvisited neighbors (2 steps away)
 * 3. Shuffle the neighbors for randomness
 * 4. For each neighbor:
 *    - If unvisited, carve passage to it and recurse
 *
 * COORDINATE SYSTEM:
 * - Odd coordinates (1,1), (1,3), (3,1) etc. are potential room centers
 * - Even coordinates are walls between rooms
 * - We carve passages between odd coordinates
 *
 * HINTS:
 * - Use the helper functions provided above
 * - Base case: no unvisited neighbors (backtrack naturally)
 * - Remember to carve both the destination AND the wall between
 */
void recursiveBacktrack(vector<string>& maze, int row, int col) {
    // Mark current cell as carved
    maze[row][col] = ' ';

    // 1. Get unvisited neighbors using getUnvisitedNeighbors()
    vector<pair<int, int>> unvisitedNeighbors = getUnvisitedNeighbors(maze, row, col);

    // 2. Shuffle them using shuffleVector()
    shuffleVector(unvisitedNeighbors);

    // 3. For each neighbor, if it's still unvisited:
    for (auto neighbor : unvisitedNeighbors) {
        if (!isCarved(maze, neighbor.first, neighbor.second)) {
            // Carve passage between current cell and neighbor (carves neighbor and wall between)
            carvePassage(maze, row, col, neighbor.first, neighbor.second);
            // Recurse into the neighbor cell
            recursiveBacktrack(maze, neighbor.first, neighbor.second);
        }
    }
}

/**
 * Helper function: Add random rooms to the maze
 * Punches holes in walls to create larger open areas
 */
void addRandomRooms(vector<string>& maze, int roomRate) {
    size_t rows = maze.size();
    size_t cols = maze[0].size();

    // Calculate how many rooms to add based on roomRate percentage
    int totalWalls = (rows * cols) / 4;  // Rough estimate of wall cells
    int roomsToAdd = (totalWalls * roomRate) / 100;

    for (int i = 0; i < roomsToAdd; i++) {
        // Avoid borders; cast size_t to int for modulo operations
        int row = 2 + (rng() % (static_cast<int>(rows) - 4));
        int col = 2 + (rng() % (static_cast<int>(cols) - 4));

        // Only carve if it's currently a wall
        if (maze[row][col] == '#') {
            maze[row][col] = ' ';
        }
    }
}

/**
 * Helper function: Place start and exit positions
 * Finds two open cells that are far apart and places S and E
 */
void placeStartAndExit(vector<string>& maze) {
    vector<pair<size_t, size_t>> openCells;
    size_t rows = maze.size();
    size_t cols = maze[0].size();

    // Find all open cells
    for (size_t r = 1; r < rows - 1; r++) {
        for (size_t c = 1; c < cols - 1; c++) {
            if (maze[r][c] == ' ') {
                openCells.push_back({r, c});
            }
        }
    }

    if (openCells.size() < 2) {
        cout << "Warning: Not enough open cells for start/exit placement!" << endl;
        return;
    }

    // Place start at first open cell
    maze[openCells[0].first][openCells[0].second] = 'S';

    // Place exit at last open cell (likely far from start)
    maze[openCells.back().first][openCells.back().second] = 'E';
}


std::vector<std::string> generateDungeon(int rows, int cols, int roomRate) {
    // Ensure odd dimensions for proper maze structure
    if (rows % 2 == 0) rows++;
    if (cols % 2 == 0) cols++;

    // Minimum size check
    if (rows < 5 || cols < 5) {
        rows = max(rows, 5);
        cols = max(cols, 5);
    }

    // Initialize maze with all walls
    vector<string> maze(rows, string(cols, '#'));

    recursiveBacktrack(maze, 1,1);
    // Position (1,1) ensures we start at an odd coordinate (proper cell center)

    // Add random rooms (provided)
    addRandomRooms(maze, roomRate);

    // Place start and exit (provided)
    placeStartAndExit(maze);

    return maze;
}
