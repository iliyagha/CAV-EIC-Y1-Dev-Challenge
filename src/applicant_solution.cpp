//
// Created by dusan on 9/15/26.
//

#include "../include/antworld.h"
#include <vector>
#include <set>
#include <cmath>
#include <cstdint>
#include <queue>
#include <functional>
#include <cstdio> //DEBUG

namespace {
    // ===== DEBUG ====
    constexpr bool DEBUG_LOG = true; // true to show, false to silence
    int debugStep = 0;

    // Print function used for debugging (only activated when DEBUG_LOG = true)
    void logMove(size_t antIdx, const char *action, Coord from, int energyBefore,
        bool carryingBefore, const Ant &ant) {
        if (!DEBUG_LOG) return;
        int spent = energyBefore - ant.energy;
        std::printf("[step %3d] ant %zu %-12s (%2d, %2d)->(%2d, %2d) energy %3d -> %3d (spent %2d)%s%s\n",
            debugStep, antIdx, action, from.first, from.second, ant.position.first, ant.position.second,
            energyBefore, ant.energy, spent,
            ant.carryingFood ? " [carrying]" : "", //Check if ant is carrying food, print if it is
            (spent == 0 && ant.energy > 0 && ant.carryingFood == carryingBefore) ? " <-- STUCK (no move made)" : "");
    }
    // ==== END DEBUG ====

    // Shared colony memory, persists across forage() calls
    // Built ONLY from what ants have sensed (never read directly from the food map)
    std::set<Coord> knownFood;

    // Track which cells we've already explored, to bias exploration
    std::vector<std::vector<bool>> visited;

    void ensureVisitedInitialized( int rows, int cols) {
        if (visited.empty()) {
            visited.assign(rows, std::vector<bool>(cols, false)); // Initialize a vector of vectors with all false
        }
    }

    // Terrain the colony has actually seen, use foodRadius (3) as the view radius
    std::vector<std::vector<bool>> knownTerrain;

    void ensureKnownTerrainInitialized(int rows, int cols) {
        if (knownTerrain.empty()) {
            knownTerrain.assign(rows, std::vector<bool>(cols, false));
        }
    }

    // Mark every cell inside this ant's view square as seen so its height can be used for planning
    void updateKnownTerrainFromScan(Ant &ant, int rows, int cols) {
        // Check every cell in this ant's food radius
        for (int r = ant.position.first - ant.foodRadius; r <= ant.position.first + ant.foodRadius; ++r) {
            for (int c = ant.position.second - ant.foodRadius; c <= ant.position.second + ant.foodRadius; ++c) {
                if (r < 0 || r >= rows || c < 0 || c >= cols ) continue; // If cell is out of bounds; ignore
                knownTerrain[r][c] = true;
            }
        }
    }



    // Old path cost function  includes shortestPath, a function that looks at the whole map
    // including terrain no ant has seen and we are not allowed to use this information to plan
    // We had to write our own path cost function since we are not allowed to edit shortestPath

    constexpr int UNSEEN_STEP_COST = 2; // Assuming worst case the maximum step cost is 2
    // since neighbouring heights differ by at most 1

    // New pathCost function with similar algorithm to shortestPath()
    // Estimated energy cost of cheapest path between two cells, using ONLY seen terrain
    // When both cells have been seen, step cost is the real 1 + |height difference|
    // Otherwise, it is UNSEEN_STEP_COST (the worst case) since we never think an ant can afford a trip it can't
    int pathCost(MapTemplate &terrainMap, Coord from, Coord to) {
        int rows = (int)terrainMap.size();
        int cols = (int)terrainMap[0].size();

        //dist[r][c] = cheapest known cost from 'from' to (r,c) so far
        std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INT32_MAX));
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq; // Pq that acts as a to-do list in this
        // function, cheapest cell first

        dist[from.first][from.second] = 0; // Start cell
        pq.push({0, from});

        // The four directions the ant can move in
        const int dr[4] = {-1, 1, 0, 0}; // Up, Down
        const int dc[4] = {0, 0, -1, 1}; // Left, or Right

        while (!pq.empty()) {
            Node current = pq.top(); // Take the cheapest cell from the pq
            pq.pop(); // Remove the cell so you can iterate onto the next

            int r = current.pos.first;
            int c = current.pos.second;

            if (current.cost != dist[r][c]) continue; // Outdated queue entry, this cell must have appeared before
            if (current.pos == to) return current.cost; // Reached the target: this is the cheapest cost

            for (int i = 0; i < 4; ++i) {
                // Iterate through each of the four neighbours of the current position one at a time
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue; // Off the map

                // If both cells have been seen, use the real step cost. Otherwise use the worst case step cost
                int stepCost = (knownTerrain[r][c] && knownTerrain[nr][nc]) ? 1 + std::abs(terrainMap[r][c] -
                    terrainMap[nr][nc]) : UNSEEN_STEP_COST;

                // Update if a cheaper route to the neighbour has been found and record it in dist
                if (current.cost + stepCost < dist[nr][nc]) {
                    dist[nr][nc] = current.cost + stepCost;
                    pq.push({dist[nr][nc], {nr, nc}}); // Add neighbour to the pq
                    // at it's new cost
                }
            }


        }
        return dist[to.first][to.second]; // Fallback -> since every cell can be reached,
        // the other return always happens first
    }

    // Update colony memory using ONLY what this ant's sensor reports
    // Cells inside the scan square that foodScan() did not return are known to be empty
    // so food that another ant already took gets removed without ever reading the real map
    // an ant might remember food at (5,8) but another ant picks it up later
    // Originally this was handled by checking the real food map which I feel breaks the rules
    // of "you do not know food outside sensor range"
    // This function keeps it accurate using only what an ant's sensor reports
    void updateKnownFoodFromScan(Ant &ant, MapTemplate &foodMap, int rows, int cols) {
        std::vector<Coord> seen = ant.foodScan(foodMap); // This is the only place the foodMap is used, through the ants sensor
        std::set<Coord> seenSet(seen.begin(), seen.end());

        // The for loops go over every cell in the scan square around the ants position
        for (int r = ant.position.first - ant.foodRadius; r<= ant.position.first + ant.foodRadius; ++r) {
         for (int c = ant.position.second - ant.foodRadius; c <= ant.position.second + ant.foodRadius; ++c) {
             if ( r < 0 || r >= rows || c < 0 || c >= cols ) continue;
             if (seenSet.count({r, c})) knownFood.insert({r, c}); // If the sensor saw food, insert it.
             // If it is already known, set ignores the duplicate
             else knownFood.erase({r, c}); // If the sensor saw no food here, forget it
         }
        }
    }

    // Keep colony  food memory correct after a move, using only the ant's own state
    // if it just picked up food -> cell becomes empty
    // if it dies while carrying -> updateWorld() will drop food on this cell
    void recordMoveResult(Ant &ant, bool carryingBefore, Coord home) {
        if (!carryingBefore && ant.carryingFood) {
            knownFood.erase(ant.position);
        }
        if (ant.carryingFood && ant.energy == 0 && ant.position != home) {
            knownFood.insert(ant.position);
        }
    }

    // Find best food using shortestPath() function (takes energy + terrain + distance into account)
    // "Best" -> cheapest full trip, only trips ants can afford
    // Returns {-1, -1} if no known food can be brought all the way home
    Coord determineBestFood(Ant &ant, MapTemplate &terrainMap, Coord homeCoordinates) {
        // {-1, -1} -> "no food found yet", any real trip will beat INT32_MAX distance
        Coord bestFood = {-1, -1};
        int bestCost = INT32_MAX;

        // Check every known food
        for (auto &f : knownFood) {
            int cost = pathCost(terrainMap, ant.position, f) + pathCost(terrainMap, f, homeCoordinates);

            // Using exactly all its energy is fine, updateWorld() scores the food before removing ant
            if (cost > ant.energy)
                continue;
            if (cost < bestCost) {
                bestCost = cost;
                bestFood = f;
            }
        }
        return bestFood;
    }

    // Last trip: used when no full round trip is affordable
    // Picks food with cheapest full trip that the ant can still REACH with energy to spare
    // This way it can bring it part of the way home
    // Returns {-1, -1} if no such trip exists
    Coord determineLastTripFood(Ant &ant, MapTemplate &terrainMap, Coord homeCoordinates) {
        Coord bestFood = {-1, -1};
        int bestCost = INT32_MAX;

        for (auto &f : knownFood) {
            int costToFood = pathCost(terrainMap, ant.position, f);

            // Must arrive with energy left, otherwise it just dies on food cell
            if (costToFood >= ant.energy) continue;

            int cost = costToFood + pathCost(terrainMap, f, homeCoordinates);
            if (cost < bestCost) {
                bestCost = cost;
                bestFood = f;
            }
        }
        return bestFood;
    }

    // If an ant's order didn't move it (e.g. 1 energy left but next step on its path costs 2),
    // spend its remaining energy on a neighbouring cell that is affordable. Ants are removed at exactly 0 energy
    // so without this function an ant keeps the game running until the step limit meaning the game never ends
    // Carrying ants step onto home if they can afford it, so the food still scores
    void spendLeftoverEnergy(Ant &ant, MapTemplate &terrainMap, MapTemplate &foodMap, Coord home) {
        // Similar methods to pathCost
        int rows = (int)terrainMap.size();
        int cols = (int)terrainMap[0].size();
        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        Coord target = {-1, -1};
        // Check all four directions from current position
        for (int i = 0; i < 4; ++i) {
            int nr = ant.position.first + dr[i]; // next row
            int nc = ant.position.second + dc[i]; // next coloumn
            // Make sure ant does not leave map
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols ) continue;


            // Neighbouring cells are always inside the ant's view, so using their real height is always allowed
            int stepCost = 1 + std::abs(terrainMap[ant.position.first][ant.position.second] - terrainMap[nr][nc]);
            if (stepCost > ant.energy) continue; // Unable to make this step

            if (ant.carryingFood && Coord(nr, nc) == home) {
                target = home; break; // Deliver if possible
            }
            if (target.first == -1) target = {nr,nc};
            }
        if (target.first != -1) ant.move(terrainMap, target, foodMap);
        }


    // Pick some unvisited cell to explore
    Coord pickExploreTarget(Ant &ant, int rows, int cols) {
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; ++c) {
                if (!visited[r][c]) {
                    return {r, c};

                }
            }
        }
        return ant.homeCoord; // fallback: nothing unexplored
    }
}

/** @brief this is where you as the applicant will make use of the above functions to develop your solution.
 * here are some existing examples of how calling these functions works to help get you started!
 */
void AntWorld::forage() {
    int rows = (int)this->terrainMap.size();
    int cols = (int)this->terrainMap[0].size();
    ensureVisitedInitialized(rows, cols);
    ensureKnownTerrainInitialized(rows, cols);
    debugStep++; // DEBUG: count steps

    for (auto &ant : this->ants) {
        // Mark current cell explored
        visited[ant.position.first][ant.position.second] = true;

        // DEBUG: snapshot before this ant acts
        size_t antIdx = &ant - &this->ants[0];
        Coord startPos = ant.position;
        int startEnergy = ant.energy;
        bool startCarrying = ant.carryingFood;

        // Deleted SAFETY-OVERRIDE, energy never refills at home, so no ant has a reason to "save itself"

        // --- 1. Scanning is free, so every ant scans at every step ---
        updateKnownFoodFromScan(ant, this->foodMap, rows, cols);
        updateKnownTerrainFromScan(ant, rows, cols);


        // --- 2. Already carrying food: bring it home ---
        // If energy runs out on the way home, food is dropped closer to home (and remembered in knownFood)
        if (ant.carryingFood) {
            ant.returnHome(this->terrainMap, this->foodMap);
            // If the order changed nothing (stuck), spend leftover energy so ant can't stall the game
            if (ant.position == startPos && ant.energy == startEnergy && ant.carryingFood == startCarrying) {
                spendLeftoverEnergy(ant, this->terrainMap, this->foodMap, this->homeCoordinates);
            }
            recordMoveResult(ant, startCarrying, this->homeCoordinates);

            // DEBUG: if this ant is bringing food home log it
            logMove(antIdx, "deliver", startPos, startEnergy, startCarrying, ant);

            continue;
        }

        // We should not know food outside of sensor range
        // The code that was here checked if food is still there anywhere on the map

        // --- 3. Cheapest known food this ant can bring all the way home ---
        Coord bestFood = determineBestFood(ant, this->terrainMap, this->homeCoordinates);
        const char *action = "get-food"; // DEBUG label

        // --- 4. Last Trip: no full round trip affordable, so grab food and bring it closer
        // Die for the colony
        if (bestFood.first == -1) {
            bestFood = determineLastTripFood(ant, this->terrainMap, this->homeCoordinates);
            action = "last-trip"; // DEBUG label
        }

        // Go to the chosen food (move() picks it up on arrival)
        if (bestFood.first != -1) {
            ant.move(this->terrainMap, bestFood, this->foodMap);

            // If the order changed nothing (stuck), spend leftover energy so ant can't stall the game
            if (ant.position == startPos && ant.energy == startEnergy && ant.carryingFood == startCarrying) {
                spendLeftoverEnergy(ant, this->terrainMap, this->foodMap, this->homeCoordinates);
            }
            recordMoveResult(ant, startCarrying, this->homeCoordinates);

            // DEBUG: if the ant is getting food log it
            logMove(antIdx, action, startPos, startEnergy, startCarrying, ant);
            continue;
        }


        // --- 5. Otherwise, explore ---
        Coord target = pickExploreTarget(ant, rows, cols);
        ant.move(this->terrainMap, target, this->foodMap);

        // If the order changed nothing (stuck), spend leftover energy so ant can't stall the game
        if (ant.position == startPos && ant.energy == startEnergy && ant.carryingFood == startCarrying) {
            spendLeftoverEnergy(ant, this->terrainMap, this->foodMap, this->homeCoordinates);
        }
        recordMoveResult(ant, startCarrying, this->homeCoordinates);

        // DEBUG: ant is doing no other move so log that it is
        logMove(antIdx, "explore", startPos, startEnergy, startCarrying, ant);

    }
}

/** You may insert any custom functions below **/
