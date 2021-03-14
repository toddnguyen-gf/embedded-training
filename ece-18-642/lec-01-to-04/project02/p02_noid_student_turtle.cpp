/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Todd Nguyen
 * ANDREW ID: noid
 * LAST UPDATE: Sunday 2021-03-14 10:49
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"
#include <stdint.h>

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT 40 // bigger number slows down simulation so you can see what's happening
float _current_timeout;
float _fx1, _fy1, _fx2, _fy2;
bool _has_this_orientation_been_traveled, _is_end_of_maze, _maze_contains_wall;

enum class Orientation
{
    kWest = 0,
    kNorth,
    kEast,
    kSouth,
};
const char *ORIENTATION_STR[] = {"WEST", "NORTH", "EAST", "SOUTH"};

enum class State
{
    kNoWall = 0,
    kHasWall,
    kGoThisOrientation,
};
const char *STATE_STR[] = {"NO WALL", "HAS A WALL", "GO THIS ORIENTATION"};
State _current_state;

bool moveLeftHandRule(QPointF &pos_, int &nw_or);
bool moveRightHandRule(QPointF &pos_, int &nw_or);
Orientation _set_orientation_and_current_state(
    Orientation orientation, Orientation orientationGoThisOrientation,
    Orientation orientationMazeContainsWall);
bool _returnTimeout();
void _moveTurtle(QPointF &pos_, Orientation orientation);
void _setTurtlePosition(QPointF &pos_, Orientation orientation);

/**
 *  this procedure takes the current turtle position and orientation and returns
 *  true=submit changes, false=do not submit changes
 *  Ground rule -- you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
 *  and NO other turtle methods or maze methods (no peeking at the maze!)
*/
bool studentMoveTurtle(QPointF &pos_, int &nw_or)
{
    // return moveLeftHandRule(pos_, nw_or);
    return moveRightHandRule(pos_, nw_or);
}

bool moveRightHandRule(QPointF &pos_, int &nw_or)
{
    // ROS_INFO("Turtle update Called  w=%f", _current_timeout);
    Orientation orientation = static_cast<Orientation>(nw_or);

    if (0 == _current_timeout)
    {
        _setTurtlePosition(pos_, orientation);
        _maze_contains_wall = bumped(_fx1, _fy1, _fx2, _fy2);
        _is_end_of_maze = atend(pos_.x(), pos_.y());

        switch (orientation)
        {
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
        }

        ROS_INFO("Orientation='%s' State='%s'",
                 ORIENTATION_STR[static_cast<int>(orientation)],
                 STATE_STR[static_cast<int>(_current_state)]);

        _moveTurtle(pos_, orientation);
    }

    nw_or = static_cast<int>(orientation);
    return _returnTimeout();
}

bool moveLeftHandRule(QPointF &pos_, int &nw_or)
{
    // ROS_INFO("Turtle update Called  w=%f", _current_timeout);
    Orientation orientation = static_cast<Orientation>(nw_or);

    if (0 == _current_timeout)
    {
        _setTurtlePosition(pos_, orientation);
        _maze_contains_wall = bumped(_fx1, _fy1, _fx2, _fy2);
        _is_end_of_maze = atend(pos_.x(), pos_.y());

        switch (orientation)
        {
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
        }

        ROS_INFO("Orientation='%s' State='%s'",
                 ORIENTATION_STR[static_cast<int>(orientation)],
                 STATE_STR[static_cast<int>(_current_state)]);
        _moveTurtle(pos_, orientation);
    }

    nw_or = static_cast<int>(orientation);
    return _returnTimeout();
}

Orientation _set_orientation_and_current_state(
    Orientation orientation, Orientation orientationGoThisOrientation,
    Orientation orientationMazeContainsWall)
{
    if (_current_state == State::kGoThisOrientation)
    {
        orientation = orientationGoThisOrientation;
        _current_state = State::kHasWall;
    }
    else if (_maze_contains_wall)
    {
        orientation = orientationMazeContainsWall;
        _current_state = State::kNoWall;
    }
    else
    {
        _current_state = State::kGoThisOrientation;
    }
    return orientation;
}

bool _returnTimeout()
{
    if (_is_end_of_maze)
        return false;

    if (_current_timeout == 0)
        _current_timeout = TIMEOUT;
    else
        _current_timeout -= 1;

    if (_current_timeout == TIMEOUT)
        return true;
    return false;
}

void _moveTurtle(QPointF &pos_, Orientation orientation)
{
    _has_this_orientation_been_traveled = _current_state == State::kGoThisOrientation;
    if (_has_this_orientation_been_traveled == true && _is_end_of_maze == false)
    {
        switch (orientation)
        {
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
        }

        _has_this_orientation_been_traveled = false;
    }
}

void _setTurtlePosition(QPointF &pos_, Orientation orientation)
{
    _fx1 = pos_.x();
    _fy1 = pos_.y();
    _fx2 = pos_.x();
    _fy2 = pos_.y();
    if (orientation < Orientation::kEast)
    {
        if (orientation == Orientation::kWest)
            _fy2 += 1;
        else
            _fx2 += 1;
    }
    else
    {
        _fx2 += 1;
        _fy2 += 1;
        if (orientation == Orientation::kEast)
            _fx1 += 1;
        else
            _fy1 += 1;
    }
}
