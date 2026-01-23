### 常用命令

```shell
west build -t menuconfig
west build -p always -b eigen_blec_nrf52833
west flash -r jlink

nrfjprog --recover --clockspeed 100
nrfjprog --reset --clockspeed 100
nrfjprog --halt --clockspeed 100
```

```shell
# 根据 map 获取 indentify 为 "hb" 的变量的 RAM 地址
grep -n " hb" build/zephyr/zephyr.map | head -n 20

# 读取 4 words 的 0x20000000 地址值
nrfjprog --memrd 0x20000000 --w 32 --n 4 --clockspeed 100
```

### UICR 锁定或无法进入 main 函数

通过测试的配置：
```conf
# 先把所有日志/控制台关掉（很多卡死发生在串口/console init）
CONFIG_LOG=n
CONFIG_CONSOLE=n
CONFIG_UART_CONSOLE=n
CONFIG_PRINTK=n
CONFIG_SERIAL=n

# 先关 flash/存储/文件系统（如果你工程里带了）
CONFIG_FLASH=n
CONFIG_FLASH_MAP=n
CONFIG_NVS=n
CONFIG_SETTINGS=n
CONFIG_FCB=n
CONFIG_FILE_SYSTEM=n

# 先别让任何低频时钟/外设复杂初始化影响（RC 32k，稳定）
CONFIG_CLOCK_CONTROL=y
CONFIG_CLOCK_CONTROL_NRF=y
CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC=y
CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL=n
CONFIG_CLOCK_CONTROL_NRF_K32SRC_SYNTH=n
```

测试代码：
```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
__attribute__((used)) volatile uint32_t hb = 0x12345678;

int main(void)
{
    hb = 0xDEADBEEF;   // 一进 main 就改
    if (!device_is_ready(led.port)) {
        hb = 0xE0010001;  // port not ready
        while (1) { }  // 即使 GPIO 设备不 ready，也让 hb 变化
    }

    int ret =gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        hb = 0xE0010002 | (uint32_t)(ret & 0xFFFF);  // configure error
        while (1) { }  // 即使配置失败，也让 hb 变化
    }

    while (1) {
      hb ^= 0xA5A5A5A5;
      (void)gpio_pin_toggle_dt(&led);
      k_msleep(200);
    }
}

// #include <zephyr/kernel.h>

// __attribute__((used)) volatile uint32_t hb = 0x11111111;

// static int mark_pre_kernel_1(void)
// {
//     hb = 0xAAAA0001;
//     return 0;
// }
// SYS_INIT(mark_pre_kernel_1, PRE_KERNEL_1, 0);

// static int mark_pre_kernel_2(void)
// {
//     hb = 0xAAAA0002;
//     return 0;
// }
// SYS_INIT(mark_pre_kernel_2, PRE_KERNEL_2, 0);

// static int mark_post_kernel(void)
// {
//     hb = 0xAAAA0003;
//     return 0;
// }
// SYS_INIT(mark_post_kernel, POST_KERNEL, 0);

// static int mark_application(void)
// {
//     hb = 0xAAAA0004;
//     return 0;
// }
// SYS_INIT(mark_application, APPLICATION, 0);

// int main(void)
// {
//     hb = 0xDEADBEEF;
//     while (1) {
//         hb ^= 0xA5A5A5A5;
//         k_busy_wait(1000);
//     }
// }
```
