## Notes

- For each turtle move:
  - Turtle can either `NO_MOVE`, `MOVE` (base on orietnation), or `TURN (LEFT/RIGHT)`
  - When do we have a NO_MOVE?
    - when `_returnTimeout()` returns `false`
  - `MOVE`?
    - when it is time (timeout finishes)
    - set orientation first
    - when this condition is satisfied.
```cpp
_has_this_orientation_been_traveled =
    _current_state == State::kMove;
  if (_has_this_orientation_been_traveled == true && _is_end_of_maze == false) {
```
  - `TURN`?
    - when the boolean above is false
