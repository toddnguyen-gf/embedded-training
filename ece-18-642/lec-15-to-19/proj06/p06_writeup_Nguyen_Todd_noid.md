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

# Write-Up Questions

_Q1. In one or two sentences, why did your turtle fail at the more complicated mazes?_
The turtle failed because the left-hand / right-hand rule only applies to mazes that are inter-connected. That is, every wall must be conneted to another wall or to an outer wall (the wall on the edges).

_Q2. Briefly describe your solution strategy (your high level algorithmic approach). We have in mind 50-100 words, but shorter or longer is OK. This should be how you'd explain it to a friend, not a detailed algorithmic step-by-step description._
Essentially, we will make sure the turtle visits every single cell, where the turtle will eventually visit the end cell. To do this, we will find the shortest path to all cells.

_Q3. What difficulties did you encounter in this project?_
Thinking of how the algorithm will work for the turtle was challenging for me, as the basic algorithm assumes that we'll have an omnipotent view of the maze. However, the turtle only has a limited view of the maze, so I had to tweak the algorithm a little bit so that it can work with the turtle's limited view.

_Q4. The final version of your high level requirements after iteration. Start numbering at R1._
Please see the section [High Level Software Requirements](#high-level-software-requirements)

_Q5. The final version of your sequence diagrams. SD1._

![SD1](sequence-diagrams/SD1%20-%20Turtle%20Reach%20End.svg "SD1")

![SD2](sequence-diagrams/SD2%20-%20Turtle%20Keeping%20Track%20of%20Squares.svg "SD2")

![SD3](sequence-diagrams/SD3%20-%20Turtle%20Before%20Decision%20No%20Wall.svg "SD3")

![SD3A](sequence-diagrams/SD3A%20-%20There%20is%20a%20wall.svg "SD3A")

![SD4](sequence-diagrams/SD4%20-%20hasVisited%20is%20true%20and%20orientation%20!=%20beginning%20ori.svg "SD4")

![SD5](sequence-diagrams/SD5%20-%20hasVisited%20is%20false.svg "SD5")

![SD6](sequence-diagrams/SD6%20-%20hasVisited%20is%20true%20and%20orientation%20==%20beginning%20ori.svg "SD6")

![SD7A](sequence-diagrams/SD7A%20-%20One%20Square%20WIth%20Lowest%20Number%20of%20Vistis.svg "SD7A")

![SD7B](sequence-diagrams/SD7B%20-%20Multiple%20squares%20with%20the%20lowest%20number%20of%20visits.svg "SD7B")
