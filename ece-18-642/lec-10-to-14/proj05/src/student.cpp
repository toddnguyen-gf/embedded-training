#include "student.h"

#include <stdint.h>

static const char *_FS_TURTLE_MOVE_STR[] = {"NO MOVE", "MOVE", "TURN LEFT", "TURN RIGHT"};

const char *STUDENT_getTurtleMoveDecisionString(turtleMove turtle_move_decision) {
  uint8_t index = static_cast<uint8_t>(turtle_move_decision);
  return _FS_TURTLE_MOVE_STR[index];
}
