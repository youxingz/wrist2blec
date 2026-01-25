#include <zephyr/kernel.h>

#include "inc/ble.h"
#include "inc/task_imu.h"


int main(void)
{
  ble_init();
  task_imu_init();

  task_imu_start();

  for(;;) {
    k_msleep(1000);
    static uint32_t count = 0;
    printk("live! [%d]\n", count++);
  }
  return 0;
}
