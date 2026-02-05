#include <zephyr/kernel.h>

#include "inc/ble.h"
#include "inc/task_imu.h"
#include "inc/pba.h"

int event_after_startup()
{
  ble_init();
  task_imu_init();
  task_imu_start();
  return 0;
}
int event_before_shutdown()
{
  task_imu_stop();
  task_imu_uninit();
  ble_uninit();
  return 0;
}


int main(void)
{
  // 初始化
  pba_init();

  for(;;) {
    k_msleep(1000);
    static uint32_t count = 0;
    printk("live! [%d]\n", count++);
  }
  return 0;
}
