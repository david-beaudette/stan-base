/** STAN THE STANDING ROBOT
   Navigation function
   by David Beaudette   
**/

// One step of the pitch complementary filter
void complementary_filter_step(float &pitch, int ax, int ay, int az, int gy);