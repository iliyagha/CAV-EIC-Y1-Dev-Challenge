//
// Created by dusan on 9/15/26.
//

#include "../include/antworld.h"
#include <vector>
#include <set>
#include <cmath>
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
    std::set<Coord> knownFood;

    // Track which cells we've already explored, to bias exploration
    std::vector<std::vector<bool>> visited;

    void ensureVisitedInitialized( int rows, int cols) {
        if (visited.empty()) {
            visited.assign(rows, std::vector<bool>(cols, false));
        }
    }

    // Rough distance-based check: can this ant afford a trip out to 'target'
    // and still get home afterward?
    bool canAffordRoundTrip(Ant &ant, MapTemplate &terrainMap, Coord target, Coord home) {
        auto outPath = shortestPath(terrainMap, ant.position, target);
        auto backPath = shortestPath(terrainMap, target, home);
        int cost = calculatePathCost(terrainMap, outPath) + calculatePathCost(terrainMap, backPath);
        return cost <= ant.energy;
    }

    // Pick some unvisited cell to explore
    // TODO: replace with something smarter than "first unvisited cell found" -
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
    // Gets length and height of the terrain
    int rows = (int)this->terrainMap.size();
    int cols = (int)this->terrainMap[0].size();
    ensureVisitedInitialized(rows, cols);
    debugStep++; // DEBUG: count steps

    // For-loop that iterates for EVERY ant
    for (auto &ant : this->ants) {
        // Mark current cell explored
        visited[ant.position.first][ant.position.second] = true;

        // DEBUG: snapshot before this ant acts
        size_t antIdx = &ant - &this->ants[0];
        Coord startPos = ant.position;
        int startEnergy = ant.energy;
        bool startCarrying = ant.carryingFood;

        // --- SAFETY OVERRIDE: always protect the ability to get home ---
        int costHome = calculatePathCost(this->terrainMap,
            shortestPath(this->terrainMap, ant.position, this->homeCoordinates));
        if (ant.energy <= costHome) {
            ant.returnHome(this->terrainMap, this->foodMap);

            // DEBUG: if this ant is returning home for safety, log it
            logMove(antIdx, "safety-home", startPos, startEnergy, startCarrying, ant);

            continue;
        }

        // --- Already carrying food: bring it home ---
        if (ant.carryingFood) {
            ant.returnHome(this->terrainMap, this->foodMap);

            // DEBUG: if this ant is bringing food home log it
            logMove(antIdx, "deliver", startPos, startEnergy, startCarrying, ant);

            continue;
        }

        // --- Update shared knowledge from this ant's senses ---
        for (auto &f : ant.foodScan(this->foodMap)) knownFood.insert(f);
        // TODO: also fold in phermoneScan results if you want phermone-based coordination

        // determineBestFood() would replace THIS function here

        // --- Is there known, reachable food? Go get the nearest one ---
        Coord bestFood = {-1, -1}; // Initializes farthest setting for food as default
        int bestDist = INT32_MAX;
        for (auto &f : knownFood) {
            if (this->foodMap[f.first][f.second] != 1) continue; // already taken; food is absent if the location contains 0
            if (!canAffordRoundTrip(ant, this->terrainMap, f, this->homeCoordinates)) continue; // If ant can't make it back with the food,
            int dist = std::abs(f.first - ant.position.first) + std::abs(f.second - ant.position.second); // Compares ant to next food
            if (dist < bestDist) { bestDist = dist; bestFood = f; } // Saves it as best if it is the closest
        }

        // With it ENDING HERE

        // This makes it go to the best food
        if (bestFood.first != -1) {
            ant.move(this->terrainMap, bestFood, this->foodMap);

            // DEBUG: if the ant is getting food log it
            logMove(antIdx, "get-food", startPos, startEnergy, startCarrying, ant);

            continue;
        }

        // --- Otherwise, explore ---
        Coord target = pickExploreTarget(ant, rows, cols);
        ant.move(this->terrainMap, target, this->foodMap);

        // DEBUG: ant is doing no other move so log that it is
        logMove(antIdx, "explore", startPos, startEnergy, startCarrying, ant);

    }
}

/** You may insert any custom functions below **/

// Function to find the best food using the shortestPath() function (takes into account energy based off terrain and distance)
Coord determineBestFood(Ant &ant, MapTemplate &terrainMap, MapTemplate &foodMap, Coord homeCoordinates)
{
    // Initialize best food and best cost as far as possible as a placeholder
    Coord bestFood = {-1, -1};
    int bestCost = INT32_MAX;

    // Go over every known food
    for(auto &f : knownFood)
    {
        // Check if the food is still there; ignore if not
        if(foodMap[f.first][f.second] != 1)
            continue;

        // Find its shortest path home using the built-in function
        auto pathToFood = shortestPath(terrainMap, ant.position, f);

        // Find its shortest path home in the same manner
        auto pathToHome = shortestPath(terrainMap, f, homeCoordinates);

        // Calculate the amount of energy required for the trip
        int cost = calculatePathCost(terrainMap, pathToFood) + calculatePathCost(terrainMap, pathToHome);

        // Make sure the ant can actually do the trip; ignore if not
        if(cost >= ant.energy)
            continue;

        // Now actually check if this is the best food so far; switch it if so
        if(cost < bestCost)
        {
            bestCost = cost;
            bestFood = f;
        }
    }

    return bestFood;
}