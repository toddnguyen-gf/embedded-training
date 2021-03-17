/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Todd Nguyen
 * ANDREW ID: noid
 * LAST UPDATE: Sunday 2021-03-15 at 13:13
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include <stdint.h>

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

/**
 *  this procedure takes the current turtle position and orientation and returns
 *  true=submit changes, false=do not submit changes
 *  Ground rule -- you are only allowed to call the helper functions
 * "bumped(..)" and "atend(..)", and NO other turtle methods or maze methods (no
 * peeking at the maze!)
 */
bool studentMoveTurtle(QPointF &pos_, int &nw_or);

#ifndef NOID_STUDENT_TURTLE_CLASS_H_
#define NOID_STUDENT_TURTLE_CLASS_H_
class StudentTurtle {
 public:
  StudentTurtle();
  ~StudentTurtle();

  typedef uint16_t Timeout;

  /**
   * Move the turtle using the left hand rule.
   */
  bool moveLeftHandRule(QPointF &pos_, int &nw_or);

  /**
   * Move the turtle using the right hand rule.
   */
  bool moveRightHandRule(QPointF &pos_, int &nw_or);

  const static Timeout TIMEOUT_MAX = 40;

 private:
  typedef struct Coordinates {
    uint8_t x;
    uint8_t y;

    // Initialization list!
    Coordinates() : x(0), y(0) {}
  } Coordinates;
  typedef enum Orientation {
    kWest = 0,
    kNorth,
    kEast,
    kSouth,
  } Orientation;

  typedef enum State {
    kNoWall = 0,
    kHasWall,
    kGoThisOrientation,
  } State;

  const char *STATE_STR[3] = {"NO WALL", "HAS A WALL", "GO THIS ORIENTATION"};
  const char *ORIENTATION_STR[4] = {"WEST", "NORTH", "EAST", "SOUTH"};
  Timeout _current_timeout;
  Coordinates _upper_left_coords, _lower_right_coords;
  State _current_state;
  bool _has_this_orientation_been_traveled, _is_end_of_maze,
      _maze_contains_wall;

  /**
   * Set the orientation and the current state base on the turtle's CURRENT
   * orientation.
   */
  Orientation _setOrientationAndCurrentState(
      Orientation orientation, Orientation orientationGoThisOrientation,
      Orientation orientationMazeContainsWall);

  /**
   * Calculate and return if timeout is 0 for the next movement.
   */
  bool _returnTimeout();

  /**
   * Move the turtle base on its orientation.
   */
  void _moveTurtleHelper(QPointF &pos_, Orientation orientation);

  /**
   * Actually set the turtle's position e.g. have its y incremented if we are
   * moving SOUTH.
   */
  void _setTurtlePosition(QPointF &pos_, Orientation orientation);
};
#endif  // NOID_STUDENT_TURTLE_CLASS_H_

static StudentTurtle _student_turtle;

bool studentMoveTurtle(QPointF &pos_, int &nw_or) {
  // return _student_turtle.moveLeftHandRule(pos_, nw_or);
  return _student_turtle.moveRightHandRule(pos_, nw_or);
}

// --------------------------------------------------------
// |  IMPLEMENTATION
// --------------------------------------------------------

StudentTurtle::StudentTurtle() : _current_timeout(0) {}
StudentTurtle::~StudentTurtle() {}

bool StudentTurtle::moveLeftHandRule(QPointF &pos_, int &nw_or) {
  // ROS_INFO("Turtle update Called  w=%f", _current_timeout);
  Orientation orientation = static_cast<Orientation>(nw_or);

  if (0 == _current_timeout) {
    this->_setTurtlePosition(pos_, orientation);
    this->_maze_contains_wall =
        bumped(_upper_left_coords.x, _upper_left_coords.y,
               _lower_right_coords.x, _lower_right_coords.y);
    this->_is_end_of_maze = atend(pos_.x(), pos_.y());

    switch (orientation) {
      case Orientation::kWest:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kSouth, Orientation::kNorth);
        break;
      case Orientation::kNorth:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kWest, Orientation::kEast);
        break;
      case Orientation::kEast:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kNorth, Orientation::kSouth);
        break;
      case Orientation::kSouth:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kEast, Orientation::kWest);
        break;
      default:
        ROS_ERROR("%s", "Should never reach this default case");
        break;
    }

    ROS_INFO("Orientation='%s' State='%s'",
             ORIENTATION_STR[static_cast<int>(orientation)],
             STATE_STR[static_cast<int>(_current_state)]);
    this->_moveTurtleHelper(pos_, orientation);
  }

  nw_or = static_cast<int>(orientation);
  return this->_returnTimeout();
}

bool StudentTurtle::moveRightHandRule(QPointF &pos_, int &nw_or) {
  // ROS_INFO("Turtle update Called  w=%f", _current_timeout);
  Orientation orientation = static_cast<Orientation>(nw_or);

  if (0 == _current_timeout) {
    this->_setTurtlePosition(pos_, orientation);
    this->_maze_contains_wall =
        bumped(_upper_left_coords.x, _upper_left_coords.y,
               _lower_right_coords.x, _lower_right_coords.y);
    this->_is_end_of_maze = atend(pos_.x(), pos_.y());

    switch (orientation) {
      case Orientation::kWest:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kNorth, Orientation::kSouth);
        break;
      case Orientation::kNorth:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kEast, Orientation::kWest);
        break;
      case Orientation::kEast:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kSouth, Orientation::kNorth);
        break;
      case Orientation::kSouth:
        orientation = this->_setOrientationAndCurrentState(
            orientation, Orientation::kWest, Orientation::kEast);
        break;
      default:
        ROS_ERROR("%s", "Should never reach this default case");
        break;
    }

    ROS_INFO("Orientation='%s' State='%s'",
             ORIENTATION_STR[static_cast<int>(orientation)],
             STATE_STR[static_cast<int>(_current_state)]);

    this->_moveTurtleHelper(pos_, orientation);
  }

  nw_or = static_cast<int>(orientation);
  return this->_returnTimeout();
}

StudentTurtle::Orientation StudentTurtle::_setOrientationAndCurrentState(
    Orientation orientation, Orientation orientationGoThisOrientation,
    Orientation orientationMazeContainsWall) {
  if (this->_current_state == State::kGoThisOrientation) {
    orientation = orientationGoThisOrientation;
    this->_current_state = State::kHasWall;
  } else if (this->_maze_contains_wall) {
    orientation = orientationMazeContainsWall;
    this->_current_state = State::kNoWall;
  } else {
    this->_current_state = State::kGoThisOrientation;
  }
  return orientation;
}

bool StudentTurtle::_returnTimeout() {
  if (this->_is_end_of_maze) {
    return false;
  }

  if (this->_current_timeout == 0) {
    this->_current_timeout = StudentTurtle::TIMEOUT_MAX;
  } else {
    this->_current_timeout -= 1;
  }

  return StudentTurtle::TIMEOUT_MAX == this->_current_timeout;
}

void StudentTurtle::_moveTurtleHelper(QPointF &pos_, Orientation orientation) {
  this->_has_this_orientation_been_traveled =
      this->_current_state == State::kGoThisOrientation;
  if (this->_has_this_orientation_been_traveled == true &&
      this->_is_end_of_maze == false) {
    switch (orientation) {
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

    this->_has_this_orientation_been_traveled = false;
  }
}

void StudentTurtle::_setTurtlePosition(QPointF &pos_, Orientation orientation) {
  this->_upper_left_coords.x = pos_.x();
  this->_upper_left_coords.y = pos_.y();
  this->_lower_right_coords.x = pos_.x();
  this->_lower_right_coords.y = pos_.y();

  switch (orientation) {
    case Orientation::kWest:
      this->_lower_right_coords.y += 1;
      break;
    case Orientation::kNorth:
      this->_lower_right_coords.x += 1;
      break;
    case Orientation::kEast:
      this->_lower_right_coords.x += 1;
      this->_lower_right_coords.y += 1;
      this->_upper_left_coords.x += 1;
      break;
    case Orientation::kSouth:
      this->_lower_right_coords.x += 1;
      this->_lower_right_coords.y += 1;
      this->_upper_left_coords.y += 1;
      break;
    default:
      ROS_ERROR("%s", "Should never reach this default case");
      break;
  }
}
