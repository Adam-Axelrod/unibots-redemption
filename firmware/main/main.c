// Unibots firmware for the Yahboom MicroROS control board (ESP32-S3).
//
// Publishes and subscribes the same topics as Yahboom's firmware, so nothing on the
// Pi has to change, but with mecanum kinematics: linear.y strafes, and /odom_raw
// reports it. See README.md.
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int16.h>

#include "config.h"
#include "imu.h"
#include "lidar.h"
#include "motors.h"
#include "periph.h"
#include "transport.h"

// Anything that fails while setting up means the link is not usable. Restarting is
// safe: the motors stop, and the board goes back to waiting for the agent.
#define MUST(call) do { if ((call) != RCL_RET_OK) esp_restart(); } while (0)

static rcl_publisher_t pub_scan, pub_imu, pub_odom, pub_battery;
static rcl_subscription_t sub_cmd, sub_beep, sub_s1, sub_s2;

static sensor_msgs__msg__LaserScan msg_scan;
static sensor_msgs__msg__Imu msg_imu;
static nav_msgs__msg__Odometry msg_odom;
static std_msgs__msg__UInt16 msg_battery;
static geometry_msgs__msg__Twist msg_cmd;
static std_msgs__msg__UInt16 msg_beep;
static std_msgs__msg__Int32 msg_s1, msg_s2;

static float scan_ranges[MS200_BINS];
static float odom_x, odom_y, odom_heading;
static int64_t last_odom_us;

// A dropped sensor message is harmless: the next one is along shortly.
static void publish(rcl_publisher_t *pub, const void *msg)
{
    rcl_ret_t ret = rcl_publish(pub, msg, NULL);
    (void)ret;
}

static void set_string(rosidl_runtime_c__String *s, const char *text)
{
    s->data = (char *)text;
    s->size = strlen(text);
    s->capacity = s->size + 1;
}

static void stamp(builtin_interfaces__msg__Time *t)
{
    int64_t ns = rmw_uros_epoch_nanos();
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

static void on_cmd(const void *msg)
{
    const geometry_msgs__msg__Twist *m = msg;
    twist_t cmd = {(float)m->linear.x, (float)m->linear.y, (float)m->angular.z};
    motors_command(&cmd);
}

static void on_beep(const void *msg)
{
    buzzer_command(((const std_msgs__msg__UInt16 *)msg)->data);
}

static void on_s1(const void *msg)
{
    servo_set(0, (int)((const std_msgs__msg__Int32 *)msg)->data);
}

static void on_s2(const void *msg)
{
    servo_set(1, (int)((const std_msgs__msg__Int32 *)msg)->data);
}

static void scan_timer(rcl_timer_t *timer, int64_t last)
{
    lidar_copy_ranges(scan_ranges);
    stamp(&msg_scan.header.stamp);
    publish(&pub_scan, &msg_scan);
}

static void imu_timer(rcl_timer_t *timer, int64_t last)
{
    float accel[3], gyro[3];
    if (!imu_read(accel, gyro)) return;
    msg_imu.linear_acceleration.x = accel[0];
    msg_imu.linear_acceleration.y = accel[1];
    msg_imu.linear_acceleration.z = accel[2];
    msg_imu.angular_velocity.x = gyro[0];
    msg_imu.angular_velocity.y = gyro[1];
    msg_imu.angular_velocity.z = gyro[2];
    stamp(&msg_imu.header.stamp);
    publish(&pub_imu, &msg_imu);
}

static void odom_timer(rcl_timer_t *timer, int64_t last)
{
    int64_t now = esp_timer_get_time();
    float dt = (now - last_odom_us) / 1e6f;
    last_odom_us = now;

    twist_t v;
    motors_measured(&v);
    float c = cosf(odom_heading), s = sinf(odom_heading);
    odom_x += (v.vx * c - v.vy * s) * dt;
    odom_y += (v.vx * s + v.vy * c) * dt;
    odom_heading = atan2f(sinf(odom_heading + v.wz * dt), cosf(odom_heading + v.wz * dt));

    msg_odom.pose.pose.position.x = odom_x;
    msg_odom.pose.pose.position.y = odom_y;
    msg_odom.pose.pose.orientation.z = sinf(odom_heading / 2.0f);
    msg_odom.pose.pose.orientation.w = cosf(odom_heading / 2.0f);
    msg_odom.twist.twist.linear.x = v.vx;
    msg_odom.twist.twist.linear.y = v.vy;
    msg_odom.twist.twist.angular.z = v.wz;
    stamp(&msg_odom.header.stamp);
    publish(&pub_odom, &msg_odom);
}

static void battery_timer(rcl_timer_t *timer, int64_t last)
{
    float volts = battery_volts();
    msg_battery.data = (uint16_t)(volts * 10.0f + 0.5f);
    publish(&pub_battery, &msg_battery);

    // Alarm once a second, and also when a wheel has faulted.
    if ((volts > 1.0f && volts < LOW_BATTERY_V) || motors_faulted()) buzzer_command(200);
    led_toggle();
}

static void messages_init(void)
{
    set_string(&msg_scan.header.frame_id, "laser_frame");
    msg_scan.angle_min = -(float)M_PI;
    msg_scan.angle_max = (float)M_PI;
    msg_scan.angle_increment = (float)M_PI / 180.0f;
    msg_scan.range_min = 0.12f;
    msg_scan.range_max = 8.0f;
    msg_scan.ranges.data = scan_ranges;
    msg_scan.ranges.size = MS200_BINS;
    msg_scan.ranges.capacity = MS200_BINS;

    set_string(&msg_imu.header.frame_id, "imu_frame");
    msg_imu.orientation.w = 1.0;   // No orientation: integrate angular_velocity.z

    set_string(&msg_odom.header.frame_id, "odom_frame");
    set_string(&msg_odom.child_frame_id, "base_footprint");
    msg_odom.pose.pose.orientation.w = 1.0;
    msg_odom.pose.covariance[0] = 0.001;
    msg_odom.pose.covariance[7] = 0.001;
    msg_odom.pose.covariance[35] = 0.001;
    msg_odom.twist.covariance[0] = 0.0001;
    msg_odom.twist.covariance[7] = 0.0001;
    msg_odom.twist.covariance[35] = 0.0001;
}

static void ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    MUST(rcl_init_options_init(&options, allocator));
    MUST(rcl_init_options_set_domain_id(&options, ROS_DOMAIN_ID));
    rmw_init_options_t *rmw = rcl_init_options_get_rmw_init_options(&options);
    MUST(rmw_uros_options_set_custom_transport(true, NULL, transport_open, transport_close,
                                               transport_write, transport_read, rmw));

    rclc_support_t support;
    while (rclc_support_init_with_options(&support, 0, NULL, &options, &allocator) != RCL_RET_OK) {
        led_toggle();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    rcl_node_t node;
    MUST(rclc_node_init_default(&node, "unibots_board", "", &support));

    MUST(rclc_publisher_init_default(&pub_scan, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
    MUST(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu"));
    MUST(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom_raw"));
    MUST(rclc_publisher_init_default(&pub_battery, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "battery"));

    MUST(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    MUST(rclc_subscription_init_default(&sub_beep, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "beep"));
    MUST(rclc_subscription_init_default(&sub_s1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_s1"));
    MUST(rclc_subscription_init_default(&sub_s2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_s2"));

    static rcl_timer_t t_scan, t_imu, t_odom, t_battery;
    MUST(rclc_timer_init_default(&t_scan, &support, RCL_MS_TO_NS(80), scan_timer));
    MUST(rclc_timer_init_default(&t_imu, &support, RCL_MS_TO_NS(40), imu_timer));
    MUST(rclc_timer_init_default(&t_odom, &support, RCL_MS_TO_NS(50), odom_timer));
    MUST(rclc_timer_init_default(&t_battery, &support, RCL_MS_TO_NS(1000), battery_timer));

    rclc_executor_t executor;
    MUST(rclc_executor_init(&executor, &support.context, 8, &allocator));
    MUST(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, on_cmd, ON_NEW_DATA));
    MUST(rclc_executor_add_subscription(&executor, &sub_beep, &msg_beep, on_beep, ON_NEW_DATA));
    MUST(rclc_executor_add_subscription(&executor, &sub_s1, &msg_s1, on_s1, ON_NEW_DATA));
    MUST(rclc_executor_add_subscription(&executor, &sub_s2, &msg_s2, on_s2, ON_NEW_DATA));
    MUST(rclc_executor_add_timer(&executor, &t_odom));
    MUST(rclc_executor_add_timer(&executor, &t_imu));
    MUST(rclc_executor_add_timer(&executor, &t_scan));
    MUST(rclc_executor_add_timer(&executor, &t_battery));

    rmw_uros_sync_session(1000);
    last_odom_us = esp_timer_get_time();

    // If the agent goes away, restart and wait for it to come back.
    int64_t next_ping = esp_timer_get_time() + 2000000;
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        if (esp_timer_get_time() >= next_ping) {
            if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) esp_restart();
            next_ping = esp_timer_get_time() + 2000000;
        }
    }
}

void app_main(void)
{
    periph_init();
    motors_init();
    lidar_init();
    imu_init();          // On failure the robot still drives; /imu just stays silent
    messages_init();
    xTaskCreatePinnedToCore(ros_task, "ros", 16 * 1024, NULL, 5, NULL, 0);
}
