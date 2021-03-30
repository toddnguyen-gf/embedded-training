#include "properties.h"

static const char *_FS_ORIENTATION_STR[] = {"WEST", "NORTH", "EAST", "SOUTH"};


const char *PROPERTIES_getOrientationString(Orientation orientation) {
  uint8_t index = static_cast<uint8_t>(orientation);
  return _FS_ORIENTATION_STR[index];
}
