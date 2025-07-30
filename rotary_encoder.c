#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/timer.h> // 커널 타이머 헤더 추가

// --- 장치 이름 및 주 번호 설정 ---
#define DEVICE_NAME "rotary_dev"

// --- GPIO 핀 설정 (BCM GPIO 번호 기준) ---
#define LED0_GPIO   538
#define LED1_GPIO   531
#define LED2_GPIO   525
#define LED3_GPIO   518
#define LED5_GPIO   533
#define LED6_GPIO   532
#define LED7_GPIO   528

#define ROTARY_CLK_GPIO 529
#define ROTARY_DT_GPIO  539
#define KEY_SWITCH_GPIO 534

#define DEBOUNCE_TIME_MS 50 // 디바운싱 시간 (50ms)

// --- 전역 변수 ---
static int rotary_count = 0;
static int led5_state = 0;
static dev_t dev_num;
static struct cdev rotary_cdev;
static struct class *rotary_class;
static struct device *rotary_device;

// 인터럽트 관련
static int rotary_irq_clk;
static int key_switch_irq;
static int last_rotary_clk_state;
static unsigned long last_rotary_irq_time = 0;

// 디바운싱을 위한 커널 타이머
static struct timer_list debounce_timer;

// ... (이전 코드의 함수 선언, 파일 연산 구조체, LED 업데이트 함수 등은 동일) ...
static int rotary_open(struct inode *, struct file *);
static int rotary_release(struct inode *, struct file *);
static ssize_t rotary_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t rotary_write(struct file *, const char __user *, size_t, loff_t *);
static long rotary_ioctl(struct file *, unsigned int, unsigned long);

static const struct file_operations rotary_fops = {
    .owner   = THIS_MODULE,
    .open    = rotary_open,
    .release = rotary_release,
    .read    = rotary_read,
    .write   = rotary_write,
    .unlocked_ioctl = rotary_ioctl,
};

static void update_led_display(int value) {
    value = value % 16;
    if (value < 0) value += 16;
    gpio_set_value(LED0_GPIO, (value & 0x01) ? 1 : 0);
    gpio_set_value(LED1_GPIO, (value & 0x02) ? 1 : 0);
    gpio_set_value(LED2_GPIO, (value & 0x04) ? 1 : 0);
    gpio_set_value(LED3_GPIO, (value & 0x08) ? 1 : 0);
}


// --- Rotary Encoder 인터럽트 핸들러 (이전과 동일) ---
static irqreturn_t rotary_encoder_isr(int irq, void *dev_id) {
    unsigned long current_time = jiffies;
    if (time_after(current_time, last_rotary_irq_time + msecs_to_jiffies(2))) {
        int current_clk_state = gpio_get_value(ROTARY_CLK_GPIO);
        if (current_clk_state != last_rotary_clk_state) {
            int dt_state = gpio_get_value(ROTARY_DT_GPIO);
            if (current_clk_state == 0) {
                if (dt_state == 1) { rotary_count++; gpio_set_value(LED7_GPIO, 1); gpio_set_value(LED6_GPIO, 0); }
                else { rotary_count--; gpio_set_value(LED7_GPIO, 0); gpio_set_value(LED6_GPIO, 1); }
                update_led_display(rotary_count);
            }
        }
        last_rotary_clk_state = current_clk_state;
        last_rotary_irq_time = current_time;
    }
    return IRQ_HANDLED;
}

// --- 디바운싱 타이머 콜백 함수 (NEW) ---
static void debounce_timer_callback(struct timer_list *t) {
    // 타이머가 만료된 시점의 버튼 상태를 다시 확인
    int current_state = gpio_get_value(KEY_SWITCH_GPIO);

    // 여전히 눌려있다면(LOW), 유효한 누름으로 간주하고 토글 실행
    if (current_state == 0) {
        led5_state = !led5_state;
        gpio_set_value(LED5_GPIO, led5_state);
        printk(KERN_INFO "Key Switch: Valid press detected, LED5 toggled to %s\n", led5_state ? "ON" : "OFF");
    }

    // 다음 입력을 받을 수 있도록 인터럽트를 다시 활성화
    enable_irq(key_switch_irq);
}

// --- Key Switch 인터럽트 핸들러 (MODIFIED) ---
static irqreturn_t key_switch_isr(int irq, void *dev_id) {
    // 인터럽트가 발생하면, 추가적인 바운싱을 막기 위해 인터럽트를 즉시 비활성화
    disable_irq_nosync(key_switch_irq);

    // 디바운싱 타이머를 현재로부터 DEBOUNCE_TIME_MS 이후로 설정
    mod_timer(&debounce_timer, jiffies + msecs_to_jiffies(DEBOUNCE_TIME_MS));

    return IRQ_HANDLED;
}


// ... (장치 파일 연산 함수들은 이전과 동일) ...
static int rotary_open(struct inode *inode, struct file *file) { return 0; }
static int rotary_release(struct inode *inode, struct file *file) { return 0; }
static ssize_t rotary_read(struct file *file, char __user *buf, size_t count, loff_t *pos) {
    int len; char kbuf[20];
    if (*pos > 0) return 0;
    len = scnprintf(kbuf, sizeof(kbuf), "%d\n", rotary_count);
    if (copy_to_user(buf, kbuf, len)) return -EFAULT;
    *pos += len; return len;
}
static ssize_t rotary_write(struct file *file, const char __user *buf, size_t count, loff_t *pos) {
    long new_count; int ret; char kbuf[20];
    if (count >= sizeof(kbuf)) return -EINVAL;
    if (copy_from_user(kbuf, buf, count)) return -EFAULT;
    kbuf[count] = '\0';
    ret = kstrtol(kbuf, 10, &new_count);
    if (ret != 0) return ret;
    rotary_count = new_count; update_led_display(rotary_count);
    return count;
}
#define IOCTL_ROTARY_GET_COUNT _IOR('R', 1, int)
#define IOCTL_ROTARY_SET_COUNT _IOW('R', 2, int)
#define IOCTL_ROTARY_TOGGLE_LED5 _IO('R', 3)
#define IOCTL_ROTARY_GET_LED5_STATE _IOR('R', 4, int)
static long rotary_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int ret = 0; int val;
    switch (cmd) {
        case IOCTL_ROTARY_GET_COUNT:
            val = rotary_count; if (copy_to_user((int __user *)arg, &val, sizeof(val))) ret = -EFAULT; break;
        case IOCTL_ROTARY_SET_COUNT:
            if (copy_from_user(&val, (int __user *)arg, sizeof(val))) { ret = -EFAULT; }
            else { rotary_count = val; update_led_display(rotary_count); } break;
        case IOCTL_ROTARY_TOGGLE_LED5: led5_state = !led5_state; gpio_set_value(LED5_GPIO, led5_state); break;
        case IOCTL_ROTARY_GET_LED5_STATE:
            val = led5_state; if (copy_to_user((int __user *)arg, &val, sizeof(val))) ret = -EFAULT; break;
        default: ret = -ENOTTY; break;
    }
    return ret;
}

// --- 모듈 초기화 함수 (MODIFIED) ---
static int __init rotary_init(void) {
    // ... (이전의 장치 등록, GPIO 요청 부분은 동일) ...
    int ret = 0;
    static const struct gpio leds[] = {
        { LED0_GPIO, GPIOF_OUT_INIT_LOW, "led0" }, { LED1_GPIO, GPIOF_OUT_INIT_LOW, "led1" },
        { LED2_GPIO, GPIOF_OUT_INIT_LOW, "led2" }, { LED3_GPIO, GPIOF_OUT_INIT_LOW, "led3" },
        { LED5_GPIO, GPIOF_OUT_INIT_LOW, "led5" }, { LED6_GPIO, GPIOF_OUT_INIT_LOW, "led6" },
        { LED7_GPIO, GPIOF_OUT_INIT_LOW, "led7" },
    };
    static const struct gpio inputs[] = {
        { ROTARY_CLK_GPIO, GPIOF_IN, "r_clk" }, { ROTARY_DT_GPIO,  GPIOF_IN, "r_dt"  },
        { KEY_SWITCH_GPIO, GPIOF_IN, "r_sw"  },
    };
    int i;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;
    cdev_init(&rotary_cdev, &rotary_fops);
    ret = cdev_add(&rotary_cdev, dev_num, 1);
    if (ret < 0) goto unregister_chrdev;
    rotary_class = class_create(DEVICE_NAME);
    if (IS_ERR(rotary_class)) { ret = PTR_ERR(rotary_class); goto cdev_del; }
    rotary_device = device_create(rotary_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(rotary_device)) { ret = PTR_ERR(rotary_device); goto class_destroy; }

    for (i = 0; i < ARRAY_SIZE(leds); i++) {
        ret = gpio_request_one(leds[i].gpio, leds[i].flags, leds[i].label);
        if (ret) { for (i--; i >= 0; i--) gpio_free(leds[i].gpio); goto device_destroy; }
    }
    for (i = 0; i < ARRAY_SIZE(inputs); i++) {
        ret = gpio_request_one(inputs[i].gpio, inputs[i].flags, inputs[i].label);
        if (ret) { for (i--; i >= 0; i--) gpio_free(inputs[i].gpio); goto free_led_gpios; }
    }
    update_led_display(0); last_rotary_clk_state = gpio_get_value(ROTARY_CLK_GPIO);

    // 디바운싱 타이머 초기화 (NEW)
    timer_setup(&debounce_timer, debounce_timer_callback, 0);

    // 인터럽트 등록 (이전과 동일)
    rotary_irq_clk = gpio_to_irq(ROTARY_CLK_GPIO);
    ret = request_irq(rotary_irq_clk, rotary_encoder_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "rotary_clk_irq", NULL);
    if (ret) goto free_input_gpios;

    key_switch_irq = gpio_to_irq(KEY_SWITCH_GPIO);
    ret = request_irq(key_switch_irq, key_switch_isr, IRQF_TRIGGER_FALLING, "key_switch_irq", NULL);
    if (ret) goto free_rotary_irq;

    printk(KERN_INFO "rotary_dev: Driver loaded with debounce timer. Major: %d\n", MAJOR(dev_num));
    return 0;

free_rotary_irq:
    free_irq(rotary_irq_clk, NULL);
free_input_gpios:
    for (i = 0; i < ARRAY_SIZE(inputs); i++) gpio_free(inputs[i].gpio);
free_led_gpios:
    for (i = 0; i < ARRAY_SIZE(leds); i++) gpio_free(leds[i].gpio);
device_destroy:
    device_destroy(rotary_class, dev_num);
class_destroy:
    class_destroy(rotary_class);
cdev_del:
    cdev_del(&rotary_cdev);
unregister_chrdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

// --- 모듈 종료 함수 (MODIFIED) ---
static void __exit rotary_exit(void) {
    int i;
    // 모듈이 제거될 때 타이머가 실행 중일 수 있으므로 안전하게 삭제
    del_timer_sync(&debounce_timer);

    free_irq(key_switch_irq, NULL);
    free_irq(rotary_irq_clk, NULL);

    static const struct gpio leds[] = { { LED0_GPIO }, { LED1_GPIO }, { LED2_GPIO }, { LED3_GPIO }, { LED5_GPIO }, { LED6_GPIO }, { LED7_GPIO }, };
    static const struct gpio inputs[] = { { ROTARY_CLK_GPIO }, { ROTARY_DT_GPIO }, { KEY_SWITCH_GPIO }, };
    for (i = 0; i < ARRAY_SIZE(inputs); i++) gpio_free(inputs[i].gpio);
    for (i = 0; i < ARRAY_SIZE(leds); i++) { gpio_set_value(leds[i].gpio, 0); gpio_free(leds[i].gpio); }

    device_destroy(rotary_class, dev_num);
    class_destroy(rotary_class);
    cdev_del(&rotary_cdev);
    unregister_chrdev_region(dev_num, 1);
    printk(KERN_INFO "rotary_dev: Driver unloaded.\n");
}

module_init(rotary_init);
module_exit(rotary_exit);
MODULE_LICENSE("GPL");
