#include <cstdint>

struct ShirasuLegID{//シラスのID
  uint32_t upperRightID;//右上
  uint32_t upperLeftID;//左上
  uint32_t lowerLeftID;//左下
  uint32_t lowerRightID;//右下
  uint32_t syoukouID;
};

struct Location{
  float x = 0.0;
  float y = 0.0;
};

struct Velocity{
    float upperRight = 0.0;
    float upperLeft = 0.0;
    float lowerLeft = 0.0;
    float lowerRight = 0.0;
};