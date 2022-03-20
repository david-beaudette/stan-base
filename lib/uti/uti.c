#include "uti.h"

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float horner(int n, const float *c, float x) {
  
  float f;
  
  f = c[0];
  for ( int i=1; i<n; i++ ) {
    f = f*x + c[i];
  }
  return(f);
}
