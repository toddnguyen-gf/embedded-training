/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "noid_student_turtle.h"

#include <stdint.h>

static TurtleHandRule _fs_turtle_rule = TurtleHandRule::kRightHand;

// o-------------------------------------------------------
// | PRIVATE DECLARATIONS
// o-------------------------------------------------------
typedef uint16_t Timeout;

typedef enum State {
  kAfterMove = 0,
  kIdle,
  kWallDetected,
  kMove,
} State;

/** bigger number slows down simulation so you can see what's happening */
static const Timeout TIMEOUT_MAX = 5;
static const uint8_t _FS_SQUARE_SIZE = 23;
static const char *STATE_STR[] = {"NO WALL", "HAS A WALL",
        "GO THIS ORIENTATION"
    };

static Timeout _fs_current_timeout = 0;
static uint8_t _fsTurtleMap[_FS_SQUARE_SIZE * _FS_SQUARE_SIZE] = {0};
static State _fs_current_state = State::kAfterMove;
static Coordinates _fs_relative_coords(
    (_FS_SQUARE_SIZE / 2), (_FS_SQUARE_SIZE / 2)
);
static Orientation _fs_relative_orientation = Orientation::kWest;

static bool _checkBounds(uint32_t row, uint32_t col);
static bool _decrementAndCheckCurTime();
static int16_t _getTurtleMapNumVisits(Coordinates coordinates);
static void _setTurtleMapValue(Coordinates coordinates, uint16_t value);
static turtleMove _handleAfterMoveState();
static turtleMove _handleIdleState(bool bumped);
static turtleMove _handleWallDetectedState();
static turtleMove _handleMoveState();
static void _moveActualTurtle();
static void _rotateLeft();
static void _rotateRight();

// o-------------------------------------------------------
// | PUBLIC FUNCTION IMPLEMENTATIONS
// o-------------------------------------------------------

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped, bool atend) {
  turtleMove turtle_move_decision = turtleMove::NO_MOVE;
  if (atend) {
    return turtle_move_decision;
  }

  if (0 == _fs_current_timeout) {
    switch(_fs_current_state) {
      case State::kAfterMove:
        turtle_move_decision = _handleAfterMoveState();
        break;
      case State::kIdle:
        turtle_move_decision = _handleIdleState(bumped);
        break;
      case State::kWallDetected:
        turtle_move_decision = _handleWallDetectedState();
        break;
      case State::kMove:
        turtle_move_decision = _handleMoveState();
        break;
      default:
        ROS_ERROR("No proper state");
        break;
    }

    if (turtle_move_decision != turtleMove::NO_MOVE) {
      ROS_INFO("Row (y): %2d, Col (x): %2d\n  Orientation: %5s, Num Visits: %2d, Move: '%s'",
          _fs_relative_coords.y,
          _fs_relative_coords.x,
          PROPERTIES_getOrientationString(_fs_relative_orientation),
          _getTurtleMapNumVisits(_fs_relative_coords),
          STUDENT_getTurtleMoveDecisionString(turtle_move_decision)
      );
    }
  }

  bool isTimeoutAtMax = _decrementAndCheckCurTime();
  if (!isTimeoutAtMax) {
    // Not time yet OR we are already at goal, do not move
    turtle_move_decision = turtleMove::NO_MOVE;
  }

  return turtle_move_decision;
}

int16_t STUDENT_TURTLE_getNumberOfVisits() {
  return _getTurtleMapNumVisits(_fs_relative_coords);
}

// o-------------------------------------------------------
// | PRIVATE FUNCTION IMPLEMENTATIONS
// o-------------------------------------------------------

static bool _checkBounds(uint32_t row, uint32_t col) {
  bool is_within_bounds = true;
  if (row < 0 || row >= _FS_SQUARE_SIZE) {
    is_within_bounds = false;
  } else if (col < 0 || col >= _FS_SQUARE_SIZE) {
    is_within_bounds = false;
  }

  return is_within_bounds;
}

/**
 * Calculate and return if timeout is 0 for the next movement.
 */
static bool _decrementAndCheckCurTime() {
  if (_fs_current_timeout == 0) {
    _fs_current_timeout = TIMEOUT_MAX;
  } else {
    _fs_current_timeout -= 1;
  }

  return TIMEOUT_MAX == _fs_current_timeout;
}

static int16_t _getTurtleMapNumVisits(Coordinates coordinates) {
  uint8_t x = coordinates.x;
  uint8_t y = coordinates.y;
  if (!_checkBounds(x, y)) {
    return -1;
  }
  const uint16_t index = y * _FS_SQUARE_SIZE + x;
  return _fsTurtleMap[index];
}

static void _setTurtleMapValue(Coordinates coordinates, uint16_t value) {
  int x = coordinates.x;
  int y = coordinates.y;
  if (!_checkBounds(x, y)) {
    return;
  }
  const uint16_t index = y * _FS_SQUARE_SIZE + x;
  _fsTurtleMap[index] = value;
}

static turtleMove _handleAfterMoveState() {
  _fs_current_state = State::kIdle;
  turtleMove turtle_move_decision = turtleMove::NO_MOVE;
  switch (_fs_turtle_rule) {
    case TurtleHandRule::kLeftHand:
      _rotateLeft();
      turtle_move_decision = turtleMove::TURN_LEFT;
      break;
    case TurtleHandRule::kRightHand:
      _rotateRight();
      turtle_move_decision = turtleMove::TURN_RIGHT;
      break;
    default:
      ROS_ERROR("Improper turtle hand rule selected");
      break;
  }
  return turtle_move_decision;
}

static turtleMove _handleIdleState(bool bumped) {
  if (bumped) {
    _fs_current_state = State::kWallDetected;
  } else {
    _fs_current_state = State::kMove;
  }
  return turtleMove::NO_MOVE;
}

static turtleMove _handleWallDetectedState() {
  _fs_current_state = State::kIdle;
  turtleMove turtle_move_decision = turtleMove::NO_MOVE;
  switch (_fs_turtle_rule) {
    case TurtleHandRule::kLeftHand:
      _rotateRight();
      turtle_move_decision = turtleMove::TURN_RIGHT;
      break;
    case TurtleHandRule::kRightHand:
      _rotateLeft();
      turtle_move_decision = turtleMove::TURN_LEFT;
      break;
    default:
      ROS_ERROR("Improper turtle hand rule selected");
      break;
  }
  return turtle_move_decision;
}

static turtleMove _handleMoveState() {
  _fs_current_state = State::kAfterMove;
  _moveActualTurtle();
  return turtleMove::MOVE;
}

static void _moveActualTurtle() {
  switch (_fs_relative_orientation) {
    case Orientation::kWest:
      _fs_relative_coords.x -= 1;
      break;
    case Orientation::kNorth:
      _fs_relative_coords.y -= 1;
      break;
    case Orientation::kEast:
      _fs_relative_coords.x += 1;
      break;
    case Orientation::kSouth:
      _fs_relative_coords.y += 1;
      break;
    default:
      ROS_ERROR("Not proper orientation");
      break;
  }
  int num_visits = _getTurtleMapNumVisits(_fs_relative_coords);
  _setTurtleMapValue(_fs_relative_coords, num_visits + 1);
}

static void _rotateLeft() {
  switch (_fs_relative_orientation) {
    case Orientation::kWest:
      _fs_relative_orientation = Orientation::kSouth;
      break;
    case Orientation::kNorth:
      _fs_relative_orientation = Orientation::kWest;
      break;
    case Orientation::kEast:
      _fs_relative_orientation = Orientation::kNorth;
      break;
    case Orientation::kSouth:
      _fs_relative_orientation = Orientation::kEast;
      break;
    default:
      ROS_ERROR("Not proper orientation");
      break;
  }
}

static void _rotateRight() {
  switch (_fs_relative_orientation) {
    case Orientation::kWest:
      _fs_relative_orientation = Orientation::kNorth;
      break;
    case Orientation::kNorth:
      _fs_relative_orientation = Orientation::kEast;
      break;
    case Orientation::kEast:
      _fs_relative_orientation = Orientation::kSouth;
      break;
    case Orientation::kSouth:
      _fs_relative_orientation = Orientation::kWest;
      break;
    default:
      ROS_ERROR("Not proper orientation");
      break;
  }
}
