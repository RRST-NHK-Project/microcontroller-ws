#include <micro_ros_zephyr_transport.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <zephyr/kernel.h>

void main(void) {
    // transport 初期化（UART / Serial / UDP）
    rmw_uros_set_custom_transport(
        true,
        NULL,
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "zephyr_node", "", &support);

    rcl_publisher_t publisher;
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "zephyr_pub");

    std_msgs__msg__Int32 msg;
    msg.data = 0;

    while (1) {
        msg.data++;
        rcl_publish(&publisher, &msg, NULL);
        k_msleep(100);
    }
}
