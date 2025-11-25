#include "main.h"
#include <micro_ros_stm32c_transport.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // micro-ROS transport init
    set_microros_transports(UART_TRANSPORT, &huart2);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "f446_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "number");

    msg.data = 0;

    while (1) {
        msg.data++;
        rcl_publish(&publisher, &msg, NULL);
        HAL_Delay(100);
    }
}
