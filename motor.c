#include "main.h"
#include "can.h"
#include "gpio.h"
#include "robstride_example.h"

extern CAN_HandleTypeDef hcan1;

static RobstrideAppContext g_app;
static const uint8_t g_motor_ids[4] = {1, 2, 3, 4};

static uint32_t g_ctrl_tick_ms = 0U;

static void Robot_MotorInit(void)
{
    if (RobstrideApp_Init(&g_app, &hcan1, 0, CAN_RX_FIFO0, g_motor_ids, 4, 0x00FD) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Robot_ControlStep_2ms(void)
{
    float tau_cmd[4] = {0};

    const RobstrideMotor *m0 = RobstrideApp_GetMotor(&g_app, 0);
    const RobstrideMotor *m1 = RobstrideApp_GetMotor(&g_app, 1);
    const RobstrideMotor *m2 = RobstrideApp_GetMotor(&g_app, 2);
    const RobstrideMotor *m3 = RobstrideApp_GetMotor(&g_app, 3);

    float q[4] = {0}, dq[4] = {0};

    if (m0) { q[0] = m0->state.position_rad; dq[0] = m0->state.velocity_rad_s; }
    if (m1) { q[1] = m1->state.position_rad; dq[1] = m1->state.velocity_rad_s; }
    if (m2) { q[2] = m2->state.position_rad; dq[2] = m2->state.velocity_rad_s; }
    if (m3) { q[3] = m3->state.position_rad; dq[3] = m3->state.velocity_rad_s; }

    /* TODO: 在这里写你的状态估计、LQR、VMC、虚拟腿反算 */
    tau_cmd[0] = 0.0f;
    tau_cmd[1] = 0.0f;
    tau_cmd[2] = 0.0f;
    tau_cmd[3] = 0.0f;

    RobstrideApp_SetTorque(&g_app, 0, tau_cmd[0]);
    RobstrideApp_SetTorque(&g_app, 1, tau_cmd[1]);
    RobstrideApp_SetTorque(&g_app, 2, tau_cmd[2]);
    RobstrideApp_SetTorque(&g_app, 3, tau_cmd[3]);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();

    Robot_MotorInit();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        RobstrideApp_Process(&g_app, now);

        if ((now - g_ctrl_tick_ms) >= 2U)
        {
            g_ctrl_tick_ms = now;
            Robot_ControlStep_2ms();
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    Robstride_CAN_RxFifo0MsgPendingCallback(hcan);
}
