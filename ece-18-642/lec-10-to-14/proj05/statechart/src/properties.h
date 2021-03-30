#ifndef ECE642RTLE_STUDENT_PROPERTIES_H_
#define ECE642RTLE_STUDENT_PROPERTIES_H_

#include <stdint.h>

typedef struct Coordinates {
  uint8_t x;
  uint8_t y;
  // Initialization list!
  Coordinates() : x(0), y(0) {}
  Coordinates(uint8_t x_input, uint8_t y_input): x(x_input), y(y_input) {}
} Coordinates;

typedef enum Orientation {
  kWest = 0,
  kNorth,
  kEast,
  kSouth,
} Orientation;

const char *PROPERTIES_getOrientationString(Orientation orientation);

int16_t STUDENT_TURTLE_getNumberOfVisits();

#endif // ECE642RTLE_STUDENT_PROPERTIES_H_
