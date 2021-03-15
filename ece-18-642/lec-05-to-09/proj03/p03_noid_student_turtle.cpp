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

typedef uint16_t Timeout;
typedef struct Coordinates {
  uint8_t x;
  uint8_t y;

  // Initialization list!
  Coordinates() : x(0), y(0) {}
} Coordinates;

/** bigger number slows down simulation so you can see what's happening */
const Timeout TIMEOUT_MAX = 40;
Timeout _current_timeout;

Coordinates _upper_left_coords, _lower_right_coords;
bool _has_this_orientation_been_traveled, _is_end_of_maze, _maze_contains_wall;

typedef enum Orientation {
  kWest = 0,
  kNorth,
  kEast,
  kSouth,
} Orientation;
const char *ORIENTATION_STR[] = {"WEST", "NORTH", "EAST", "SOUTH"};

typedef enum State {
  kNoWall = 0,
  kHasWall,
  kGoThisOrientation,
} State;
const char *STATE_STR[] = {"NO WALL", "HAS A WALL", "GO THIS ORIENTATION"};
State _current_state;

/**
 * Move the turtle using the left hand rule.
 */
bool moveLeftHandRule(QPointF &pos_, int &nw_or);

/**
 * Move the turtle using the right hand rule.
 */
bool moveRightHandRule(QPointF &pos_, int &nw_or);

/**
 * Set the orientation and the current state base on the turtle's CURRENT
 * orientation.
 */
Orientation _set_orientation_and_current_state(
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

/**
 *  this procedure takes the current turtle position and orientation and returns
 *  true=submit changes, false=do not submit changes
 *  Ground rule -- you are only allowed to call the helper functions
 * "bumped(..)" and "atend(..)", and NO other turtle methods or maze methods (no
 * peeking at the maze!)
 */
bool studentMoveTurtle(QPointF &pos_, int &nw_or) {
  // return moveLeftHandRule(pos_, nw_or);
  return moveRightHandRule(pos_, nw_or);
}

bool moveRightHandRule(QPointF &pos_, int &nw_or) {
  // ROS_INFO("Turtle update Called  w=%f", _current_timeout);
  Orientation orientation = static_cast<Orientation>(nw_or);

  if (0 == _current_timeout) {
    _setTurtlePosition(pos_, orientation);
    _maze_contains_wall = bumped(_upper_left_coords.x, _upper_left_coords.y,
                                 _lower_right_coords.x, _lower_right_coords.y);
    _is_end_of_maze = atend(pos_.x(), pos_.y());

    switch (orientation) {
      case Orientation::kWest:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kNorth, Orientation::kSouth);
        break;
      case Orientation::kNorth:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kEast, Orientation::kWest);
        break;
      case Orientation::kEast:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kSouth, Orientation::kNorth);
        break;
      case Orientation::kSouth:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kWest, Orientation::kEast);
        break;
      default:
        ROS_ERROR("%s", "Should never reach this default case");
        break;
    }

    ROS_INFO("Orientation='%s' State='%s'",
             ORIENTATION_STR[static_cast<int>(orientation)],
             STATE_STR[static_cast<int>(_current_state)]);

    _moveTurtleHelper(pos_, orientation);
  }

  nw_or = static_cast<int>(orientation);
  return _returnTimeout();
}

bool moveLeftHandRule(QPointF &pos_, int &nw_or) {
  // ROS_INFO("Turtle update Called  w=%f", _current_timeout);
  Orientation orientation = static_cast<Orientation>(nw_or);

  if (0 == _current_timeout) {
    _setTurtlePosition(pos_, orientation);
    _maze_contains_wall = bumped(_upper_left_coords.x, _upper_left_coords.y,
                                 _lower_right_coords.x, _lower_right_coords.y);
    _is_end_of_maze = atend(pos_.x(), pos_.y());

    switch (orientation) {
      case Orientation::kWest:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kSouth, Orientation::kNorth);
        break;
      case Orientation::kNorth:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kWest, Orientation::kEast);
        break;
      case Orientation::kEast:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kNorth, Orientation::kSouth);
        break;
      case Orientation::kSouth:
        orientation = _set_orientation_and_current_state(
            orientation, Orientation::kEast, Orientation::kWest);
        break;
      default:
        ROS_ERROR("%s", "Should never reach this default case");
        break;
    }

    ROS_INFO("Orientation='%s' State='%s'",
             ORIENTATION_STR[static_cast<int>(orientation)],
             STATE_STR[static_cast<int>(_current_state)]);
    _moveTurtleHelper(pos_, orientation);
  }

  nw_or = static_cast<int>(orientation);
  return _returnTimeout();
}

Orientation _set_orientation_and_current_state(
    Orientation orientation, Orientation orientationGoThisOrientation,
    Orientation orientationMazeContainsWall) {
  if (_current_state == State::kGoThisOrientation) {
    orientation = orientationGoThisOrientation;
    _current_state = State::kHasWall;
  } else if (_maze_contains_wall) {
    orientation = orientationMazeContainsWall;
    _current_state = State::kNoWall;
  } else {
    _current_state = State::kGoThisOrientation;
  }
  return orientation;
}

bool _returnTimeout() {
  if (_is_end_of_maze) {
    return false;
  }

  if (_current_timeout == 0) {
    _current_timeout = TIMEOUT_MAX;
  } else {
    _current_timeout -= 1;
  }

  return TIMEOUT_MAX == _current_timeout;
}

void _moveTurtleHelper(QPointF &pos_, Orientation orientation) {
  _has_this_orientation_been_traveled =
      _current_state == State::kGoThisOrientation;
  if (_has_this_orientation_been_traveled == true && _is_end_of_maze == false) {
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

    _has_this_orientation_been_traveled = false;
  }
}

void _setTurtlePosition(QPointF &pos_, Orientation orientation) {
  _upper_left_coords.x = pos_.x();
  _upper_left_coords.y = pos_.y();
  _lower_right_coords.x = pos_.x();
  _lower_right_coords.y = pos_.y();
  if (orientation < Orientation::kEast) {
    if (orientation == Orientation::kWest) {
      _lower_right_coords.y += 1;
    } else {
      _lower_right_coords.x += 1;
    }
  } else {
    _lower_right_coords.x += 1;
    _lower_right_coords.y += 1;
    if (orientation == Orientation::kEast) {
      _upper_left_coords.x += 1;
    } else {
      _upper_left_coords.y += 1;
    }
  }
}
