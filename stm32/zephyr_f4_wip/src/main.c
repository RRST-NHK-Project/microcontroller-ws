#include <microros_transports/serial-usb/microros_transports.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <zephyr/kernel.h>

int main(void) {
    zephyr_transport_open();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "zephyr_node", "", &support);

    while (1) {
        k_msleep(1000);
    }
}
