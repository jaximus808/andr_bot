#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

// ═══════════════════════════════════════════════════════════════════════════
//  PIN ASSIGNMENTS
//  All pins listed here — wire your hardware to match these.
// ═══════════════════════════════════════════════════════════════════════════

// ── Left motor (L298N channel A) ─────────────────────────────────────────
#define LEFT_PWM_PIN    25   // ENA  — speed via LEDC PWM
#define LEFT_IN1_PIN    26   // IN1  — direction
#define LEFT_IN2_PIN    27   // IN2  — direction

// ── Right motor (L298N channel B) ────────────────────────────────────────
#define RIGHT_PWM_PIN   32   // ENB  — speed via LEDC PWM
#define RIGHT_IN1_PIN   33   // IN3  — direction
#define RIGHT_IN2_PIN   14   // IN4  — direction

// ── Left wheel quadrature encoder ────────────────────────────────────────
#define LEFT_ENC_A_PIN  18   // Channel A (interrupt on CHANGE)
#define LEFT_ENC_B_PIN  19   // Channel B (interrupt on CHANGE)

// ── Right wheel quadrature encoder ───────────────────────────────────────
#define RIGHT_ENC_A_PIN  4   // Channel A (interrupt on CHANGE)
#define RIGHT_ENC_B_PIN  5   // Channel B (interrupt on CHANGE)

// ── I2C — BNO085 IMU (STEMMA QT / Qwiic) ─────────────────────────────────
#define IMU_SDA_PIN     21   // SDA — connect to STEMMA QT blue wire
#define IMU_SCL_PIN     22   // SCL — connect to STEMMA QT yellow wire
// BNO085 I2C address: 0x4A (ADR pin floating/low, factory default)

// ── Status LED ────────────────────────────────────────────────────────────
#define LED_PIN          2   // Built-in LED — on when micro-ROS connected


// ═══════════════════════════════════════════════════════════════════════════
//  ROBOT GEOMETRY  (must match URDF values in robot/description/robot_core.xacro)
// ═══════════════════════════════════════════════════════════════════════════

// Wheel radius (metres) — URDF: wheel_radius = 0.05
#define WHEEL_RADIUS        0.05f

// Centre-to-centre wheel separation (metres)
// URDF: wheel_offset_y = chassis_width/2 + wheel_thickness/2 = 0.17
//       separation     = 2 * wheel_offset_y = 0.34
#define WHEEL_SEPARATION    0.34f

// Encoder counts per wheel revolution (4× quadrature decoding of 16 PPR)
// 16 PPR × 4 = 64 counts/rev
#define ENCODER_CPR         64

// Metres of travel per encoder count
#define METERS_PER_COUNT    ((2.0f * M_PI * WHEEL_RADIUS) / ENCODER_CPR)


// ═══════════════════════════════════════════════════════════════════════════
//  LEDC PWM CONFIG
// ═══════════════════════════════════════════════════════════════════════════

#define LEFT_PWM_CH     0
#define RIGHT_PWM_CH    1
#define PWM_FREQ        1000   // Hz
#define PWM_RESOLUTION  8      // bits → 0–255

// Max physical wheel speed used to normalise cmd_vel → PWM duty
#define MAX_WHEEL_SPEED 0.5f   // m/s


// ═══════════════════════════════════════════════════════════════════════════
//  SAFETY TIMEOUT
// ═══════════════════════════════════════════════════════════════════════════

// Stop motors if no /cmd_vel received within this period
#define CMD_VEL_TIMEOUT_MS  500


// ═══════════════════════════════════════════════════════════════════════════
//  ODOMETRY PUBLISH RATE
// ═══════════════════════════════════════════════════════════════════════════

// Timer fires at 20 Hz → publishes /odom and /imu
#define ODOM_PUBLISH_HZ     20


// ═══════════════════════════════════════════════════════════════════════════
//  ENCODER STATE  (written in ISR — must be volatile)
// ═══════════════════════════════════════════════════════════════════════════

volatile int32_t left_ticks  = 0;
volatile int32_t right_ticks = 0;

// 4× quadrature ISRs:
//   On channel-A edge: A == B → forward, A != B → backward
//   On channel-B edge: A != B → forward, A == B → backward

void IRAM_ATTR left_enc_a_isr() {
    if (digitalRead(LEFT_ENC_A_PIN) == digitalRead(LEFT_ENC_B_PIN))
        left_ticks++;
    else
        left_ticks--;
}

void IRAM_ATTR left_enc_b_isr() {
    if (digitalRead(LEFT_ENC_A_PIN) != digitalRead(LEFT_ENC_B_PIN))
        left_ticks++;
    else
        left_ticks--;
}

void IRAM_ATTR right_enc_a_isr() {
    if (digitalRead(RIGHT_ENC_A_PIN) == digitalRead(RIGHT_ENC_B_PIN))
        right_ticks++;
    else
        right_ticks--;
}

void IRAM_ATTR right_enc_b_isr() {
    if (digitalRead(RIGHT_ENC_A_PIN) != digitalRead(RIGHT_ENC_B_PIN))
        right_ticks++;
    else
        right_ticks--;
}


// ═══════════════════════════════════════════════════════════════════════════
//  ODOMETRY STATE
// ═══════════════════════════════════════════════════════════════════════════

float odom_x   = 0.0f;
float odom_y   = 0.0f;
float odom_th  = 0.0f;   // heading (radians)

int32_t prev_left_ticks  = 0;
int32_t prev_right_ticks = 0;


// ═══════════════════════════════════════════════════════════════════════════
//  IMU STATE  (updated by polling BNO085 in main loop, read in timer cb)
// ═══════════════════════════════════════════════════════════════════════════

Adafruit_BNO08x  bno085;
sh2_SensorValue_t imu_event;

struct ImuCache {
    float qx = 0, qy = 0, qz = 0, qw = 1;   // orientation quaternion
    float gx = 0, gy = 0, gz = 0;             // gyroscope (rad/s)
    float ax = 0, ay = 0, az = 0;             // linear acceleration (m/s²)
    bool  valid = false;
} imu_cache;

bool imu_ok = false;   // false if BNO085 did not initialise


// ═══════════════════════════════════════════════════════════════════════════
//  micro-ROS ENTITIES
// ═══════════════════════════════════════════════════════════════════════════

rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t    odom_pub;
rcl_publisher_t    imu_pub;
rcl_timer_t        odom_timer;

geometry_msgs__msg__Twist   cmd_vel_msg;
nav_msgs__msg__Odometry     odom_msg;
sensor_msgs__msg__Imu       imu_msg;

unsigned long last_cmd_vel_ms = 0;

// Static string storage for message frame IDs
static char odom_frame_id[]       = "odom";
static char odom_child_frame_id[] = "base_link";
static char imu_frame_id[]        = "imu_link";


// ═══════════════════════════════════════════════════════════════════════════
//  ERROR LOOP — flashes LED rapidly, halts execution
// ═══════════════════════════════════════════════════════════════════════════

#define RCCHECK(fn) \
    { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { error_loop(); } }

void error_loop() {
    while (true) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  MOTOR HELPERS
// ═══════════════════════════════════════════════════════════════════════════

// speed: -1.0 (full reverse) → +1.0 (full forward)
void set_motor(uint8_t pwm_ch, uint8_t in1, uint8_t in2, float speed) {
    speed = constrain(speed, -1.0f, 1.0f);
    if (speed > 0.01f) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < -0.01f) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);   // brake
        digitalWrite(in2, LOW);
    }
    ledcWrite(pwm_ch, (uint8_t)(fabs(speed) * 255.0f));
}

void stop_motors() {
    set_motor(LEFT_PWM_CH,  LEFT_IN1_PIN,  LEFT_IN2_PIN,  0.0f);
    set_motor(RIGHT_PWM_CH, RIGHT_IN1_PIN, RIGHT_IN2_PIN, 0.0f);
}

// Differential drive kinematics: cmd_vel → individual wheel speeds
void drive(float linear_x, float angular_z) {
    float v_left  = linear_x - (angular_z * WHEEL_SEPARATION / 2.0f);
    float v_right = linear_x + (angular_z * WHEEL_SEPARATION / 2.0f);
    set_motor(LEFT_PWM_CH,  LEFT_IN1_PIN,  LEFT_IN2_PIN,  v_left  / MAX_WHEEL_SPEED);
    set_motor(RIGHT_PWM_CH, RIGHT_IN1_PIN, RIGHT_IN2_PIN, v_right / MAX_WHEEL_SPEED);
}


// ═══════════════════════════════════════════════════════════════════════════
//  cmd_vel CALLBACK
// ═══════════════════════════════════════════════════════════════════════════

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;
    last_cmd_vel_ms = millis();
    drive((float)msg->linear.x, (float)msg->angular.z);
}


// ═══════════════════════════════════════════════════════════════════════════
//  ODOMETRY TIMER CALLBACK  (fires at ODOM_PUBLISH_HZ)
// ═══════════════════════════════════════════════════════════════════════════

void odom_timer_callback(rcl_timer_t *timer, int64_t /*last_call_time*/) {
    if (timer == nullptr) return;

    // ── Read encoder ticks atomically ────────────────────────────────────
    noInterrupts();
    int32_t lt = left_ticks;
    int32_t rt = right_ticks;
    interrupts();

    int32_t dl = lt - prev_left_ticks;
    int32_t dr = rt - prev_right_ticks;
    prev_left_ticks  = lt;
    prev_right_ticks = rt;

    // ── Dead-reckoning odometry ──────────────────────────────────────────
    float dist_left   = dl * METERS_PER_COUNT;
    float dist_right  = dr * METERS_PER_COUNT;
    float dist_centre = (dist_left + dist_right) / 2.0f;
    float d_theta     = (dist_right - dist_left) / WHEEL_SEPARATION;

    odom_th += d_theta;
    odom_x  += dist_centre * cosf(odom_th);
    odom_y  += dist_centre * sinf(odom_th);

    // Heading → quaternion (2D: only yaw component)
    float qz = sinf(odom_th / 2.0f);
    float qw = cosf(odom_th / 2.0f);

    // Velocity estimates over this period
    float dt = 1.0f / ODOM_PUBLISH_HZ;
    float vx = dist_centre / dt;
    float wz = d_theta     / dt;

    // ── Publish /odom ────────────────────────────────────────────────────
    int64_t now_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    odom_msg.pose.pose.position.x    = odom_x;
    odom_msg.pose.pose.position.y    = odom_y;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.pose.pose.orientation.w = qw;

    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.angular.z = wz;

    rcl_publish(&odom_pub, &odom_msg, nullptr);

    // ── Publish /imu ─────────────────────────────────────────────────────
    if (imu_ok && imu_cache.valid) {
        imu_msg.header.stamp.sec     = odom_msg.header.stamp.sec;
        imu_msg.header.stamp.nanosec = odom_msg.header.stamp.nanosec;

        imu_msg.orientation.x = imu_cache.qx;
        imu_msg.orientation.y = imu_cache.qy;
        imu_msg.orientation.z = imu_cache.qz;
        imu_msg.orientation.w = imu_cache.qw;

        imu_msg.angular_velocity.x = imu_cache.gx;
        imu_msg.angular_velocity.y = imu_cache.gy;
        imu_msg.angular_velocity.z = imu_cache.gz;

        imu_msg.linear_acceleration.x = imu_cache.ax;
        imu_msg.linear_acceleration.y = imu_cache.ay;
        imu_msg.linear_acceleration.z = imu_cache.az;

        rcl_publish(&imu_pub, &imu_msg, nullptr);
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  BNO085 POLLING  (call every loop — non-blocking)
// ═══════════════════════════════════════════════════════════════════════════

void poll_imu() {
    if (!imu_ok) return;
    if (!bno085.getSensorEvent(&imu_event)) return;

    switch (imu_event.sensorId) {
        case SH2_ROTATION_VECTOR:
            // Fused orientation quaternion (onboard sensor fusion)
            imu_cache.qx    = imu_event.un.rotationVector.i;
            imu_cache.qy    = imu_event.un.rotationVector.j;
            imu_cache.qz    = imu_event.un.rotationVector.k;
            imu_cache.qw    = imu_event.un.rotationVector.real;
            imu_cache.valid = true;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            // Angular velocity in rad/s
            imu_cache.gx = imu_event.un.gyroscope.x;
            imu_cache.gy = imu_event.un.gyroscope.y;
            imu_cache.gz = imu_event.un.gyroscope.z;
            break;

        case SH2_LINEAR_ACCELERATION:
            // Linear acceleration with gravity removed (BNO085 onboard)
            imu_cache.ax = imu_event.un.linearAcceleration.x;
            imu_cache.ay = imu_event.un.linearAcceleration.y;
            imu_cache.az = imu_event.un.linearAcceleration.z;
            break;
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  micro-ROS LIFECYCLE
// ═══════════════════════════════════════════════════════════════════════════

bool create_entities() {
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));

    // Subscription: /cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Publisher: /odom
    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // Publisher: /imu
    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));

    // Timer: publish /odom + /imu at ODOM_PUBLISH_HZ
    RCCHECK(rclc_timer_init_default(
        &odom_timer, &support,
        RCL_MS_TO_NS(1000 / ODOM_PUBLISH_HZ),
        odom_timer_callback));

    // Executor: 2 handles — 1 subscription + 1 timer
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

    // Sync clock with host so timestamps on /odom and /imu are correct
    rmw_uros_sync_session(1000);

    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_stream(rmw_ctx);

    rclc_executor_fini(&executor);
    rcl_timer_fini(&odom_timer);
    rcl_publisher_fini(&odom_pub, &node);
    rcl_publisher_fini(&imu_pub,  &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}


// ═══════════════════════════════════════════════════════════════════════════
//  CONNECTION STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════

enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
State state = WAITING_AGENT;


// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Motor direction pins
    pinMode(LEFT_IN1_PIN,  OUTPUT);
    pinMode(LEFT_IN2_PIN,  OUTPUT);
    pinMode(RIGHT_IN1_PIN, OUTPUT);
    pinMode(RIGHT_IN2_PIN, OUTPUT);

    // Motor PWM (LEDC)
    ledcSetup(LEFT_PWM_CH,  PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LEFT_PWM_PIN,  LEFT_PWM_CH);
    ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CH);
    stop_motors();

    // Encoder pins — pull-up, interrupt on any edge for 4× decoding
    pinMode(LEFT_ENC_A_PIN,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B_PIN,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN),  left_enc_a_isr,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN),  left_enc_b_isr,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), right_enc_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), right_enc_b_isr, CHANGE);

    // I2C + BNO085
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    imu_ok = bno085.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire);
    if (imu_ok) {
        bno085.enableReport(SH2_ROTATION_VECTOR,      10000);  // 100 Hz
        bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);  // 100 Hz
        bno085.enableReport(SH2_LINEAR_ACCELERATION,  10000);  // 100 Hz
    }

    // Static message fields — frame IDs and covariance diagonals
    odom_msg.header.frame_id.data     = odom_frame_id;
    odom_msg.header.frame_id.size     = strlen(odom_frame_id);
    odom_msg.header.frame_id.capacity = sizeof(odom_frame_id);
    odom_msg.child_frame_id.data      = odom_child_frame_id;
    odom_msg.child_frame_id.size      = strlen(odom_child_frame_id);
    odom_msg.child_frame_id.capacity  = sizeof(odom_child_frame_id);

    imu_msg.header.frame_id.data     = imu_frame_id;
    imu_msg.header.frame_id.size     = strlen(imu_frame_id);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

    // Covariance diagonals — robot_localization reads these for sensor weighting.
    // Values are variance (std_dev²).  BNO085 datasheet typical noise figures:
    //   Rotation vector:      ~0.01 deg  → 0.00017 rad → variance ~0.00003
    //   Gyro calibrated:      ~0.002 rad/s std dev   → variance ~0.000004
    //   Linear acceleration:  ~0.017 m/s² std dev    → variance ~0.000289
    imu_msg.orientation_covariance[0]         = 0.00003;
    imu_msg.orientation_covariance[4]         = 0.00003;
    imu_msg.orientation_covariance[8]         = 0.00003;
    imu_msg.angular_velocity_covariance[0]    = 0.000004;
    imu_msg.angular_velocity_covariance[4]    = 0.000004;
    imu_msg.angular_velocity_covariance[8]    = 0.000004;
    imu_msg.linear_acceleration_covariance[0] = 0.000289;
    imu_msg.linear_acceleration_covariance[4] = 0.000289;
    imu_msg.linear_acceleration_covariance[8] = 0.000289;

    delay(2000);
}


// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    // Poll IMU every iteration regardless of micro-ROS state — keeps cache fresh
    poll_imu();

    switch (state) {
        case WAITING_AGENT:
            state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                        ? AGENT_AVAILABLE : WAITING_AGENT;
            break;

        case AGENT_AVAILABLE:
            state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == AGENT_CONNECTED) digitalWrite(LED_PIN, HIGH);
            break;

        case AGENT_CONNECTED:
            if (millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS)
                stop_motors();

            if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                state = AGENT_DISCONNECTED;
                break;
            }

            // Spin executor — fires cmd_vel_callback + odom_timer_callback
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            break;

        case AGENT_DISCONNECTED:
            stop_motors();
            destroy_entities();
            digitalWrite(LED_PIN, LOW);
            state = WAITING_AGENT;
            break;
    }

    delay(10);
}
