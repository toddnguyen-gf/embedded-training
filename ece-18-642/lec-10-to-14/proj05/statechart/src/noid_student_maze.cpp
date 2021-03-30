/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Todd Nguyen
 * ANDREW ID: noid
 * LAST UPDATE: Mar 23, 2021
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "noid_student_maze.h"
#include "properties.h"

// o-------------------------------------------------------
// | PRIVATE VARIABLES / FUNCTIONS DECLARATIONS
// o-------------------------------------------------------
static Coordinates _fs_upper_left_coords, _fs_lower_right_coords;
static Orientation _fs_cur_orientation;

static bool _turtleBumped(QPointF &pos_, int &nw_or);
static void _setInitialTurtlePosition(QPointF &pos_, int &nw_or);
static QPointF _moveTurtleSetCoords(QPointF &pos_);
static int _rotateLeft(int orientation);
static int _rotateRight(int orientation);

// o-------------------------------------------------------
// | PUBLIC FUNCTION IMPLEMENATIONS
// o-------------------------------------------------------

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes,
 * false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and
 * NO other turtle methods or maze methods (no peeking at the maze!)
 * The three helper functions:
 *   bool bumped(int x1, int y1,int x2,int y2);
 *   bool atend(int x, int y);
 *   void displayVisits(int visits);
 * This file interfaces with functions in student_turtle.cpp
 * @return true = accept changes, false = do NOT accept changes
 */
bool moveTurtle(QPointF &pos_, int &nw_or) {
  bool bumped = _turtleBumped(pos_, nw_or);
  bool is_at_end = atend(pos_.x(), pos_.y());
  bool did_turtle_move = false;

  // define your own turtleMove enum or structure
  turtleMove nextMove = studentTurtleStep(bumped, is_at_end);

  const char *nextMoveStr[] = {"NO MOVE", "MOVE", "TURN LEFT", "TURN RIGHT"};
  if (nextMove != turtleMove::NO_MOVE) {
    did_turtle_move = true;

    pos_ = translatePos(pos_, nextMove);
    nw_or = translateOrnt(nw_or, nextMove);
  }

  return did_turtle_move;
}

/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove) {
  switch (nextMove) {
    case turtleMove::NO_MOVE:
      // Do nothing!
      break;
    case turtleMove::MOVE:
      pos_ = _moveTurtleSetCoords(pos_);
      displayVisits(STUDENT_TURTLE_getNumberOfVisits());
      break;
    case turtleMove::TURN_LEFT:
      // Do nothing!
      break;
    case turtleMove::TURN_RIGHT:
      // Do nothing!
      break;
    default:
      break;
  }

  return pos_;
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
  switch (nextMove) {
    case turtleMove::NO_MOVE:
      // Do nothing!
      break;
    case turtleMove::MOVE:
      // Do nothing!
      break;
    case turtleMove::TURN_LEFT:
      orientation = _rotateLeft(orientation);
      break;
    case turtleMove::TURN_RIGHT:
      orientation = _rotateRight(orientation);
      break;
    default:
      break;
  }
  return orientation;
}

// o-------------------------------------------------------
// | PRIVATE FUNCTION IMPLEMENATIONS
// o-------------------------------------------------------

/**
 * See if the turtle bumped into a wall. Also sets the turtle's current
 * position and current orientation.
 * @return `true` if turtle bumped into a wall at its current position and orientation,
 * `false` otherwise.
 */
static bool _turtleBumped(QPointF &pos_, int &nw_or) {
  _setInitialTurtlePosition(pos_, nw_or);
  bool orientation_contains_wall = bumped(
          _fs_upper_left_coords.x, _fs_upper_left_coords.y,
          _fs_lower_right_coords.x, _fs_lower_right_coords.y);
  return orientation_contains_wall;
}

static void _setInitialTurtlePosition(QPointF &pos_, int &nw_or) {
  _fs_upper_left_coords.x = pos_.x();
  _fs_upper_left_coords.y = pos_.y();
  _fs_lower_right_coords.x = pos_.x();
  _fs_lower_right_coords.y = pos_.y();
  _fs_cur_orientation = static_cast<Orientation>(nw_or);

  switch (_fs_cur_orientation) {
    case Orientation::kWest:
      _fs_lower_right_coords.y += 1;
      break;
    case Orientation::kNorth:
      _fs_lower_right_coords.x += 1;
      break;
    case Orientation::kEast:
      _fs_lower_right_coords.x += 1;
      _fs_lower_right_coords.y += 1;
      _fs_upper_left_coords.x += 1;
      break;
    case Orientation::kSouth:
      _fs_lower_right_coords.x += 1;
      _fs_lower_right_coords.y += 1;
      _fs_upper_left_coords.y += 1;
      break;
    default:
      ROS_ERROR("%s", "Should never reach this default case");
      break;
  }
}

static QPointF _moveTurtleSetCoords(QPointF &pos_) {
  switch (_fs_cur_orientation) {
    case Orientation::kNorth:
      pos_.setY(pos_.y() - 1);
      break;
    case Orientation::kEast:
      pos_.setX(pos_.x() + 1);
      break;
    case Orientation::kSouth:
      pos_.setY(pos_.y() + 1);
      break;
    case Orientation::kWest:
      pos_.setX(pos_.x() - 1);
      break;
    default:
      ROS_ERROR("%s", "Should never reach this default case");
      break;
  }

  return pos_;
}

static int _rotateRight(int orientation) {
  Orientation ori = static_cast<Orientation> (orientation);
  switch (ori) {
    case Orientation::kWest:
      ori = Orientation::kNorth;
      break;
    case Orientation::kNorth:
      ori = Orientation::kEast;
      break;
    case Orientation::kEast:
      ori = Orientation::kSouth;
      break;
    case Orientation::kSouth:
      ori = Orientation::kWest;
      break;
  }

  return static_cast<int> (ori);
}

static int _rotateLeft(int orientation) {
  Orientation ori = static_cast<Orientation> (orientation);
  switch (ori) {
    case Orientation::kWest:
      ori = Orientation::kSouth;
      break;
    case Orientation::kNorth:
      ori = Orientation::kWest;
      break;
    case Orientation::kEast:
      ori = Orientation::kNorth;
      break;
    case Orientation::kSouth:
      ori = Orientation::kEast;
      break;
  }

  return static_cast<int> (ori);
}
