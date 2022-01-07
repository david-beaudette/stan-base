#include "horner.h"

float horner(int n, const float *c, float x) {
  
  float f;
  
  f = c[0];
  for ( int i=1; i<n; i++ ) {
    f = f*x + c[i];
  }
  return(f);
}
