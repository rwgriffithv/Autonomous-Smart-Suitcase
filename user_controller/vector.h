#include <Arduino.h>

struct vector {
  float x;
  float y;
  float z;
};

/**
 * Normalizes a vector.
 *
 * raw: pointer to the vector to be normalized.
 * unit: pointer to where unit vector dimensions should be stored.
 *
 * returns: length of original vector
 */
float vector_normalize(struct vector *raw, struct vector *unit);

/**
 * Adds two vectors together.
 *
 * v1: pointer to the first vector to be added.
 * v2: pointer to the second vector to be added.
 * result: pointer where the sum of v1 and v2 should be stored.
 */
void vector_add(struct vector *v1, struct vector *v2, struct vector *result);

/**
 * Multiples a vector by a constant.
 *
 * v: pointer to the vector to be multiplied.
 * c: scalar constant to multiply vector by.
 * result: pointer where the c * v should be stored.
 */
void vector_multiply(struct vector *v, float c, struct vector *result);

float vector_xy_magnitude(struct vector *v);
