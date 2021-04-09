# Project 06 Write Up

- Name: Todd Nguyen
- Student ID: noid
- Date: April 6, 2021

# High Level Software Requirements

I will employ **Dijkstra's algorithm** to try to get the shortest path to all squares of the maze. By doing this, the turtle shall reach the end square eventually.

- R-1. The turtle shall stop moving / rotating once it is at the end square of the maze.
- R-2. The turtle shall keep track of the previous square that it just left after its movement.
- R-3. After moving, the turtle shall mark the previous square as "visited".
- R-4. The turtle shall prioritize any unvisited square, and de-prioritize any visited square.
- R-5. The turtle shall move in its current orientation if there is no wall.
- R-6. If there is a wall, the turtle shall rotate right and check if there is a wall. The turtle shall continue rotating until it has a direction with no wall.
  - R-6a. If the current orientation contains a visited square, the turtle shall continue to rotate until the only choice left is a visited square.
- R-7. If there are only visited squares / walls, the turtle shall prioritize the square with LESS number of visits.
  - R-7a. The turtle shall rotate until it finds a square with the less number of visits.
  - R-7b. If the number of visits are the same, the turtle shall choose the first square that it rotates to and continue in that route.
- R-8. The turtle shall not travel across a wall.

# Traceability Table

|      | R-1 | R-2 | R-3 | R-4 | R-5 | R-6 | R-6a | R-7 | R-7a | R-7b | R-8 |
| ---- | --- | --- | --- | --- | --- | --- | ---- | --- | ---- | ---- | --- |
| SD1  | x   |     |     |     |     |     |      |     |      |      |     |
| SD2  |     | x   | x   |     |     |     |      |     |      |      |     |
| SD3  |     |     |     | x   | x   | x   | x    | x   | x    | x    | x   |
| SD4  |     |     |     |     |     | x   | x    | x   |      |      |     |
| SD5  |     |     |     | x   | x   |     |      |     |      |      |     |
| SD6  |     |     | x   |     |     |     |      | x   | x    | x    |     |
| SD7a |     |     |     |     |     |     |      | x   | x    |      |     |
| SD7b |     |     |     |     |     | x   | x    | x   |      | x    |     |
