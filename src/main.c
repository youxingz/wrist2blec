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
  //// do nothing.
  // task_imu_stop();
  // task_imu_uninit();
  // ble_uninit();
  return 0;
}


int main(void)
{
  // 初始化
  pba_init();


  k_msleep(4000); // 开机后的 5s 内不允许按钮逻辑，否则容易重复触发开关机操作

  for(;;) {
    k_msleep(1);
    pba_loop();
    static uint32_t count = 0;
    if (count++ % 1000 == 0) {
      printk("live! [%d]\n", count);
    }
  }
  return 0;
}
