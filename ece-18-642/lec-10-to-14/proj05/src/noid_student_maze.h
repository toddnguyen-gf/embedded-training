#ifndef ECE642RTLE_STUDENT_NOID_STUDENT_MAZE_H_
#define ECE642RTLE_STUDENT_NOID_STUDENT_MAZE_H_

#include "student.h"

bool moveTurtle(QPointF &pos_, int &nw_or);
QPointF translatePos(QPointF pos_, turtleMove nextMove);
int translateOrnt(int orientation, turtleMove nextMove);

#endif // ECE642RTLE_STUDENT_NOID_STUDENT_MAZE_H_
