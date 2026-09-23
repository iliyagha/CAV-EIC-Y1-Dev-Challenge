//
// Created by dusan on 9/15/26.
//

#include "../include/antworld.h"
#include <vector>
#include <set>
#include <cmath>

namespace {
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
    int rows = (int)this->terrainMap.size();
    int cols = (int)this->terrainMap[0].size();
    ensureVisitedInitialized(rows, cols);

    for (auto &ant : this->ants) {
        // Mark current cell explored
        visited[ant.position.first][ant.position.second] = true;

        // --- SAFETY OVERRIDE: always protect the ability to get home ---
        int costHome = calculatePathCost(this->terrainMap,
            shortestPath(this->terrainMap, ant.position, this->homeCoordinates));
        if (ant.energy <= costHome) {
            ant.returnHome(this->terrainMap, this->foodMap);
            continue;
        }

        // --- Already carrying food: bring it home ---
        if (ant.carryingFood) {
            ant.returnHome(this->terrainMap, this->foodMap);
            continue;
        }

        // --- Update shared knowledge from this ant's senses ---
        for (auto &f : ant.foodScan(this->foodMap)) knownFood.insert(f);
        // TODO: also fold in phermoneScan results if you want phermone-based coordination

        // --- Is there known, reachable food? Go get the nearest one ---
        Coord bestFood = {-1, -1};
        int bestDist = INT32_MAX;
        for (auto &f : knownFood) {
            if (this->foodMap[f.first][f.second] != 1) continue; // already taken
            if (!canAffordRoundTrip(ant, this->terrainMap, f, this->homeCoordinates)) continue;
            int dist = std::abs(f.first - ant.position.first) + std::abs(f.second - ant.position.second);
            if (dist < bestDist) { bestDist = dist; bestFood = f; }
        }

        if (bestFood.first != -1) {
            ant.move(this->terrainMap, bestFood, this->foodMap);
            continue;
        }

        // --- Otherwise, explore ---
        Coord target = pickExploreTarget(ant, rows, cols);
        ant.move(this->terrainMap, target, this->foodMap);
    }
}

/** You may insert any custom functions below **/
