/**
 * Dungeon Pathfinder - BFS Solver
 *
 * This file implements BFS pathfinding algorithms for dungeon navigation.
 */

#include "solver.h"
#include "cell.h"
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace std;

// Find the position of a target character in the dungeon
Cell findPosition(const vector<string>& dungeon, char target) {
    for (size_t row = 0; row < dungeon.size(); row++) {
        for (size_t col = 0; col < dungeon[row].size(); col++) {
            if (dungeon[row][col] == target) {
                return Cell(static_cast<int>(row), static_cast<int>(col));
            }
        }
    }
    return Cell(-1, -1);
}

// Check if a cell is passable for the basic BFS (walls and doors except exit block)
bool isPassableBasic(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || row >= static_cast<int>(dungeon.size()) ||
        col < 0 || col >= static_cast<int>(dungeon[0].size())) {
        return false;
    }
    char cell = dungeon[row][col];
    if (cell == '#') return false; // wall
    if (cell >= 'A' && cell <= 'F' && cell != 'E') return false; // locked door except exit
    return true;
}

// Check if a cell is passable ignoring keys (used for keys BFS neighbor validation)
bool isPassableKeys(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || row >= static_cast<int>(dungeon.size()) ||
        col < 0 || col >= static_cast<int>(dungeon[0].size())) {
        return false;
    }
    return dungeon[row][col] != '#';
}

// Check if we can pass a door given current keys held (bitmask)
bool canPassDoor(char door, int keyMask) {
    if (door < 'A' || door > 'F') return true;
    int keyBit = door - 'A'; // Door 'A' requires key bit 0 ('a')
    return (keyMask & (1 << keyBit)) != 0;
}

// Update key bitmask when collecting a new key
int collectKey(char key, int keyMask) {
    if (key < 'a' || key > 'f') return keyMask;
    int keyBit = key - 'a';
    return keyMask | (1 << keyBit);
}

// Reconstruct path from parent map for basic BFS
vector<Cell> reconstructPath(const unordered_map<Cell, Cell, CellHash>& parents,
                             const Cell& start, const Cell& goal) {
    vector<Cell> path;
    Cell current = goal;
    while (!(current.r == start.r && current.c == start.c)) {
        path.push_back(current);
        auto it = parents.find(current);
        if (it == parents.end()) return vector<Cell>(); // path broken
        current = it->second;
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

// Get valid neighbors for basic BFS
vector<Cell> getNeighbors(const vector<string>& dungeon, const Cell& current) {
    vector<Cell> neighbors;
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        int newRow = current.r + DIRECTIONS[i][0];
        int newCol = current.c + DIRECTIONS[i][1];
        if (isPassableBasic(dungeon, newRow, newCol)) {
            neighbors.push_back(Cell(newRow, newCol));
        }
    }
    return neighbors;
}

// Basic BFS to find shortest path ignoring keys and doors
vector<Cell> bfsPath(const vector<string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');
    if (start.r == -1 || exit.r == -1) return vector<Cell>();

    queue<Cell> bfsQueue;
    unordered_set<Cell, CellHash> visited;
    unordered_map<Cell, Cell, CellHash> parents;

    bfsQueue.push(start);
    visited.insert(start);

    while (!bfsQueue.empty()) {
        Cell current = bfsQueue.front();
        bfsQueue.pop();

        if (current == exit) {
            return reconstructPath(parents, start, current);
        }

        for (auto neighbor : getNeighbors(dungeon, current)) {
            if (!visited.count(neighbor)) {
                bfsQueue.push(neighbor);
                visited.insert(neighbor);
                parents[neighbor] = current;
            }
        }
    }
    return vector<Cell>(); // no path found
}

// State struct for extended BFS with keys
struct KeyState {
    int row, col, keyMask;
    KeyState() : row(0), col(0), keyMask(0) {}
    KeyState(int r, int c, int keys) : row(r), col(c), keyMask(keys) {}

    bool operator==(const KeyState& other) const {
        return row == other.row && col == other.col && keyMask == other.keyMask;
    }
};

// Hash for KeyState for unordered containers
struct KeyStateHash {
    size_t operator()(const KeyState& state) const {
        return (static_cast<size_t>(state.keyMask) << 16) ^
               (static_cast<size_t>(state.row) << 8) ^
               static_cast<size_t>(state.col);
    }
};

// Reconstruct path for key BFS from parent map
vector<Cell> reconstructKeyPath(const unordered_map<KeyState, KeyState, KeyStateHash>& parents,
                                const KeyState& start, const KeyState& goal) {
    vector<Cell> path;
    KeyState current = goal;
    while (!(current.row == start.row && current.col == start.col && current.keyMask == start.keyMask)) {
        path.push_back(Cell(current.row, current.col));
        auto it = parents.find(current);
        if (it == parents.end()) return vector<Cell>(); // path broken
        current = it->second;
    }
    path.push_back(Cell(start.row, start.col));
    reverse(path.begin(), path.end());
    return path;
}

// Extended BFS that tracks keys collected and doors passed
vector<Cell> bfsPathKeys(const vector<string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');
    if (start.r == -1 || exit.r == -1) return vector<Cell>();

    queue<KeyState> bfsQueue;
    unordered_set<KeyState, KeyStateHash> visited;
    unordered_map<KeyState, KeyState, KeyStateHash> parents;

    KeyState startState(start.r, start.c, 0);
    bfsQueue.push(startState);
    visited.insert(startState);

    while (!bfsQueue.empty()) {
        KeyState current = bfsQueue.front();
        bfsQueue.pop();

        // If reached exit
        if (current.row == exit.r && current.col == exit.c) {
            return reconstructKeyPath(parents, startState, current);
        }

        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int newRow = current.row + DIRECTIONS[i][0];
            int newCol = current.col + DIRECTIONS[i][1];

            // Check bounds first
            if (newRow < 0 || newRow >= (int)dungeon.size() ||
                newCol < 0 || newCol >= (int)dungeon[0].size()) {
                continue;
            }

            char cellChar = dungeon[newRow][newCol];

            // Wall check
            if (cellChar == '#') continue;

            // Door check: must have corresponding key
            // Exclude exit 'E' from door blocking check
            if (cellChar >= 'A' && cellChar <= 'F' && cellChar != 'E') {
                if (!canPassDoor(cellChar, current.keyMask)) continue;
            }


            // Collect key if present
            int newKeyMask = current.keyMask;
            if (cellChar >= 'a' && cellChar <= 'f') {
                newKeyMask = collectKey(cellChar, newKeyMask);
            }

            KeyState newState(newRow, newCol, newKeyMask);

            // Check if new state visited
            if (visited.count(newState) == 0) {
                bfsQueue.push(newState);
                visited.insert(newState);
                parents[newState] = current;
            }
        }
    }
    return vector<Cell>(); // no path found
}





// Optional function: count how many keys are reachable from start without considering doors
#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
int countReachableKeys(const vector<string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    if (start.r == -1) return 0;

    queue<Cell> bfsQueue;
    unordered_set<Cell, CellHash> visited;
    int keyMask = 0;

    bfsQueue.push(start);
    visited.insert(start);

    while (!bfsQueue.empty()) {
        Cell current = bfsQueue.front();
        bfsQueue.pop();

        char cellChar = dungeon[current.r][current.c];
        if (cellChar >= 'a' && cellChar <= 'f') {
            keyMask = collectKey(cellChar, keyMask);
        }

        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int newRow = current.r + DIRECTIONS[i][0];
            int newCol = current.c + DIRECTIONS[i][1];

            if (newRow < 0 || newRow >= static_cast<int>(dungeon.size()) ||
                newCol < 0 || newCol >= static_cast<int>(dungeon[0].size())) {
                continue;
            }

            if (dungeon[newRow][newCol] == '#') continue;

            Cell neighbor(newRow, newCol);
            if (!visited.count(neighbor)) {
                visited.insert(neighbor);
                bfsQueue.push(neighbor);
            }
        }
    }

    int count = 0;
    for (int i = 0; i < 6; i++) {
        if ((keyMask >> i) & 1) count++;
    }

    return count;
}
#else
int countReachableKeys(const vector<string>& /*dungeon*/) {
    return 0;
}

#endif
