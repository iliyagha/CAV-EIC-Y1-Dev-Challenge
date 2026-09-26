//
// Created by dusan on 9/15/26.
//

#include "../include/antworld.h"
#include <vector>
#include <set>
#include <cmath>
#include <cstdint>
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
            visited.assign(rows, std::vector<bool>(cols, false));
        }
    }

    // NOTE: I am going to remove usage of this function in forage() for now
    // If ants can't afford to make the trip, make them do it anyway to potentially bring food closer
    // Ants die for the colony, like in real life

    // Rough distance-based check: can this ant afford a trip out to 'target'
    // and still get home afterward?
    // bool canAffordRoundTrip(Ant &ant, MapTemplate &terrainMap, Coord target, Coord home) {
       //  auto outPath = shortestPath(terrainMap, ant.position, target);
        // auto backPath = shortestPath(terrainMap, target, home);
    // int cost = calculatePathCost(terrainMap, outPath) + calculatePathCost(terrainMap, backPath);
       // return cost <= ant.energy;
    // }

    // Energy cost of the cheapest path between two cells (accounts for terrain height)
    int pathCost(MapTemplate &terrainMap, Coord from, Coord to) {
        return calculatePathCost(terrainMap, shortestPath(terrainMap, from, to));
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

    // Pick some unvisited cell to explore
    // // TODO: replace with something smarter than "first unvisited cell found"
    // e.g. nearest unvisited cell, or a frontier/spiral pattern
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
        // TODO: we can still fold in pheromoneScan results if we find an optimal use


        // --- 2. Already carrying food: bring it home ---
        // If energy runs out on the way home, food is dropped closer to home (and remembered in knownFood)
        if (ant.carryingFood) {
            ant.returnHome(this->terrainMap, this->foodMap);
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
            recordMoveResult(ant, startCarrying, this->homeCoordinates);

            // DEBUG: if the ant is getting food log it
            logMove(antIdx, action, startPos, startEnergy, startCarrying, ant);
            continue;
        }


        // --- 5. Otherwise, explore ---
        Coord target = pickExploreTarget(ant, rows, cols);
        ant.move(this->terrainMap, target, this->foodMap);
        recordMoveResult(ant, startCarrying, this->homeCoordinates);

        // DEBUG: ant is doing no other move so log that it is
        logMove(antIdx, "explore", startPos, startEnergy, startCarrying, ant);

    }
}

/** You may insert any custom functions below **/
