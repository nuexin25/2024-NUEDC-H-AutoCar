#include "Tracking.h"
#include "Motor.h"

/* 传感器全黑检测 — 到达场地顶点(A/B/C/D)，需要去抖 */
uint8_t Tracking_All_Black(void)
{
    static uint16_t blackCnt = 0;
    if (READ_L1_OUT == 1 && READ_L2_OUT == 1 && READ_L3_OUT == 1 &&
        READ_R1_OUT == 1 && READ_R2_OUT == 1 && READ_R3_OUT == 1)
    {
        blackCnt++;
        if (blackCnt > 30) { blackCnt = 0; return 1; }
    }
    else
    {
        blackCnt = 0;
    }
    return 0;
}

/* 传感器全部在白线上（完全脱线） */
uint8_t Tracking_All_White(void)
{
    return (READ_L1_OUT == 0 && READ_L2_OUT == 0 && READ_L3_OUT == 0 &&
            READ_R1_OUT == 0 && READ_R2_OUT == 0 && READ_R3_OUT == 0);
}

/* 获取传感器位置偏差: -1.0(极左) ~ +1.0(极右), 0为居中 */
float Tracking_GetError(void)
{
    float error = 0;
    int8_t count = 0;

    if (READ_L3_OUT) { error -= 3.0f; count++; }
    if (READ_L2_OUT) { error -= 2.0f; count++; }
    if (READ_L1_OUT) { error -= 1.0f; count++; }
    if (READ_R1_OUT) { error += 1.0f; count++; }
    if (READ_R2_OUT) { error += 2.0f; count++; }
    if (READ_R3_OUT) { error += 3.0f; count++; }

    if (count > 0)
        error /= (float)count;
    return error / 3.0f;
}

/* 巡线跟随：根据灰度传感器计算左右轮差速 */
void Tracking_LineFollow(float baseSpeed, float *leftSpeed, float *rightSpeed)
{
    /* 居中直行 */
    if (!READ_L3_OUT && !READ_L2_OUT && READ_L1_OUT && READ_R1_OUT && !READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed;
        *rightSpeed = baseSpeed;
    }
    /* 轻微偏右 → 右转修正 */
    else if (!READ_L3_OUT && !READ_L2_OUT && READ_L1_OUT && !READ_R1_OUT && !READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed + 0.2f;
        *rightSpeed = baseSpeed - 0.2f;
    }
    /* 轻微偏左 → 左转修正 */
    else if (!READ_L3_OUT && !READ_L2_OUT && !READ_L1_OUT && READ_R1_OUT && !READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed - 0.2f;
        *rightSpeed = baseSpeed + 0.2f;
    }
    /* 中等偏右 */
    else if (!READ_L3_OUT && READ_L2_OUT && READ_L1_OUT && !READ_R1_OUT && !READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed + 0.4f;
        *rightSpeed = baseSpeed - 0.4f;
    }
    /* 中等偏左 */
    else if (!READ_L3_OUT && !READ_L2_OUT && !READ_L1_OUT && READ_R1_OUT && READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed - 0.4f;
        *rightSpeed = baseSpeed + 0.4f;
    }
    /* 大幅偏右 */
    else if (!READ_L3_OUT && READ_L2_OUT && !READ_L1_OUT && !READ_R1_OUT && !READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed + 0.6f;
        *rightSpeed = baseSpeed - 0.6f;
    }
    /* 大幅偏左 */
    else if (!READ_L3_OUT && !READ_L2_OUT && !READ_L1_OUT && !READ_R1_OUT && READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed - 0.6f;
        *rightSpeed = baseSpeed + 0.6f;
    }
    /* 极右 — 只有R3 */
    else if (!READ_L3_OUT && !READ_L2_OUT && !READ_L1_OUT && !READ_R1_OUT && !READ_R2_OUT && READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed + 0.8f;
        *rightSpeed = baseSpeed - 0.8f;
    }
    /* 极左 — 只有L3 */
    else if (READ_L3_OUT && !READ_L2_OUT && !READ_L1_OUT && !READ_R1_OUT && !READ_R2_OUT && !READ_R3_OUT)
    {
        *leftSpeed  = baseSpeed - 0.8f;
        *rightSpeed = baseSpeed + 0.8f;
    }
    /* 全白 — 脱线，直走 */
    else
    {
        *leftSpeed  = baseSpeed;
        *rightSpeed = baseSpeed;
    }
}

/* 弧线跟随：结合陀螺仪Z轴角速度辅助巡线 */
void Tracking_ArcFollow(float baseSpeed, float *leftSpeed, float *rightSpeed, float targetYawRate)
{
    extern float Jy61_Z_Angle;
    float yawError = Jy61_Z_Angle - targetYawRate;

    /* 基本巡线差速 */
    Tracking_LineFollow(baseSpeed, leftSpeed, rightSpeed);

    /* 陀螺仪修正：偏离目标偏航角时调整差速 */
    float correction = yawError * 0.15f;
    *leftSpeed  += correction;
    *rightSpeed -= correction;
}
