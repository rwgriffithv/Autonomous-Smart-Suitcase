#include <Arduino.h>
#include "vector.h"

float vector_normalize(struct vector *raw, struct vector *unit)
{
  float mag = sqrt(pow(raw->x, 2) + pow(raw->y, 2) + pow(raw->z, 2));

  if (mag != 0) {
    unit->x = raw->x / mag;
    unit->y = raw->y / mag;
    unit->z = raw->z / mag;
  }

  return mag;
}

void vector_add(struct vector *v1, struct vector *v2, struct vector *result)
{
  result->x = v1->x + v2->x;
  result->y = v1->y + v2->y;
  result->z = v1->z + v2->z;
}

void vector_multiply(struct vector *v, float c, struct vector *result)
{
  result->x = v->x * c;
  result->y = v->y * c;
  result->z = v->z * c;
}

float vector_xy_magnitude(struct vector *v) {
  return sqrt(pow(v->x, 2) + pow(v->y, 2));
}
