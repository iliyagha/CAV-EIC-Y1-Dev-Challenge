# AntWorld Submission

- *Iliya Ghamoushi*
- *Luca Burattini*

## Approach

'forage()' trates the colony as one controller that decides each ant's action every step:

1. **Scan** - every ant scans every step (since scanning cost no energy). 
Colony memory ('knownFood') is built only from what ants sense: food seen is added, cells seen with no food are removed 
2. **Deliver** - an ant carrying food heads home.
If it runs out on the way, the food drops closer to home and we remember where, so another ant finishes the job.
3. **Collect** - if an ant is not carrying, go to the known food with the cheapest full trip (ant to food to home).
4. **Last trip** - if no full trip is affordable, grab the nearest food and bring it closer to home.
Energy never refills at home, so an ant can never "save itself", let it die for the colony.
5. **Explore** - if no food is known, head to the first unexplored cell.

**Rules we followed:** 
- We never read the foodMap outside of the ant's view. 
- We planned paths using terrain an ant has already (scan radius: 3), per the organizer's answer on Teams. 
- Unseen steps are assumed to cost 2, the maximum possible (neighbouring heights differ by at most 1).
This way we never think an ant can afford a trip it can't.

## Results
Measured on seeds 1-100.

| Change | Avg score | Games not finished | Seed 12345 |
|-----|---|---|---|
| First version (explore / collect / return, safety override) | [23.8] | [57] | 25 |
| Scan every step, sensor-only memory, cheapest-trip food, last trip, no safety override | [35.4] | [82] | 42 |
| Plan with seen terrain only | [35.4] | [82] | 42 |
|Spend leftover energy when stuck | [35.4] | [7] | 42 |

## What we learned

- **The biggest gain was a one-line bug.** Ants only scanned when not carrying food, so the colony only ever knew the
area around home. The per-move debug log exposed all the ants running to the same far corner at step 5. Scanning every
step increased the average by 11 points.
- **Information is not the bottleneck, energy is.** [An experiment where ants knew every food location scored only ~1 
point higher.] With ~40% of cells holding food, ants almost always see food nearby. This is why we decided not to use 
pheromones in our code, for repulsive or attractive communication purposes.
- **Games not finishing.** The engine only removes an ant at exactly 0 energy. Ants left with 1 energy next to cells
cost 2 could never move again, so games ran to the step limit with no score shown.

## Next steps

- **Avoid energy traps on final moves.** A 2-cost step never changes whether energy is odd or even, so an ant with odd
energy needs a flat step before it runs out. For an ant's last move, we'd walk one cell at a time and skip any step that
leaves it unable to spend its remaining energy. This would reduce the number of unfinished games significantly.
- **Smarter exploration:** ants begin searching from the nearest unexplored cell instead of the top-left corner. We
didn't bother to implement this since ants are rarely exploring due to the sheer amount of food there is on the map.

## Tools and help
- **We used Claude to review our code and run tests to gather data.**