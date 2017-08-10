#include "mraa.hpp"
#include <unistd.h>
#include "/usr/include/upm/pca9685.h"

int main() {

  upm::PCA9685 *pwm = new upm::PCA9685(0,0x40);
  pwm->ledOffTime(PCA9685_ALL_LED, 1200); // pulse off
  pwm->setModeSleep(true);
  usleep(50000);
  pwm->setModeSleep(false);

  return 0;
}

