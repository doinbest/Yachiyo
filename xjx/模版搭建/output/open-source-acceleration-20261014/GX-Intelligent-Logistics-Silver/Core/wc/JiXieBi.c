/**
  ******************************************************************************
  * @file    JiXieBi.c
  * @brief   3轴码垛机械臂驱动实现文件-----------------------------工创赛------------------------------
  * @details 实现了3轴码垛机械臂的各项功能，包括初始化、
  *          逆运动学计算、位置控制、丢步检测与校准等
  ******************************************************************************
  */

#include "JiXieBi.h"

/**
  * @brief  读取碰撞开关状态
  * @param  port: GPIO端口
  * @param  pin: GPIO引脚
  * @retval 碰撞开关状态（1-已触发/0-未触发）
  */
static uint8_t JiXieBi_DuQuPengZhuang(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
    return (state == GPIO_PIN_RESET) ? 1 : 0;
}

/**
  * @brief  机械臂延时函数
  * @param  ms: 延时毫秒数
  */
static void JiXieBi_YanShi(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief  机械臂初始化
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @retval 状态码
  * @note   初始化机械臂参数，设置电机模式，使能电机
  */
JiXieBiStatus_t JiXieBi_ChuShiHua(JiXieBi_t *jixiebi)
{
    /* 检查输入参数 */
    if (jixiebi == NULL)
        return JIXIEBI_ERROR;
    
   
    /* 设置当前位置为零点 */
    Emm_V5_Reset_CurPos_To_Zero(jixiebi->dizuoAddr);
    JiXieBi_YanShi(10);

    Emm_V5_Reset_CurPos_To_Zero(jixiebi->dabiAddr);
    JiXieBi_YanShi(10);

    Emm_V5_Reset_CurPos_To_Zero(jixiebi->xiaobiAddr);
    JiXieBi_YanShi(10);
    
    /* 设置机械臂当前位置为初始位置 */
    jixiebi->danqianAlpha = jixiebi->alpha0;
    jixiebi->danqianBeta = jixiebi->beta0;
    jixiebi->danqianGamma = jixiebi->gamma0;
    
      /* 计算初始位置的笛卡尔坐标 */
    /* 坐标系定义：
     * 原点O(0,0,0)：底座旋转中心
     * X轴：水平方向，alpha=0°时的方向
     * Y轴：水平方向，垂直于X轴  
     * Z轴：垂直向上方向
     */
float betaRad  = jixiebi->beta0  * DEG_TO_RAD;
float gammaRad = jixiebi->gamma0 * DEG_TO_RAD;
float thetaRad = jixiebi->theta  * DEG_TO_RAD;


jixiebi->danqianX = jixiebi->OA*cosf(thetaRad) - jixiebi->AB * cosf(betaRad)+jixiebi->BC * cosf(gammaRad-betaRad);
jixiebi->danqianY = 0;
jixiebi->danqianZ = jixiebi->OA*sinf(thetaRad)+ jixiebi->AB * sinf(betaRad)+jixiebi->BC * sinf(gammaRad-betaRad);
    
    return JIXIEBI_OK;
}
/**
  * @brief  逆运动学计算
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @param  Cj,Cx,Cy: 目标末段的角度和末端位置的平面投影坐标
  * @param  alpha beta gamma: 计算得到的角度(度)
  *         - alpha: 底座旋转角 [0, 360]
  *         - beta: 大臂角度，对应beta0 [32.55, 180]
  *         - gamma: 小臂角度，对应gamma0 [23.22, 180]
  * @retval 状态码
  * @note   公式：C_x=OA*cos(alpha)-AB*cos(beta)+BC*cos(gamma-beta)
  *               C_y=OA*sin(alpha)+AB*sin(beta)+BC*sin(gamma-beta)
  */
JiXieBiStatus_t JiXieBi_NiYunDongXue(JiXieBi_t *jixiebi, float Cj, float Cx, float Cy, 
                                     float *alpha, float *beta, float *gamma)
{
    if (!jixiebi || !alpha || !beta || !gamma) return JIXIEBI_ERROR;

    // 机械臂参数
    float OA = jixiebi->OA;
    float AB = jixiebi->AB;
    float BC = jixiebi->BC;

    // 1. 计算alpha角（底座旋转角）- 确保在[-180, 180]范围内
     *alpha = Cj * RAD_TO_DEG;
     if(*alpha <0) *alpha=360.0+*alpha;

    float alphaRad = (jixiebi->theta) * DEG_TO_RAD;
    // 2. 严格按推导公式计算投影x
    float X = Cx - OA * cosf(alphaRad);
    float Y = Cy - OA * sinf(alphaRad)+JIXIEBI_zz_y; // 调整Y坐标以考虑重力误差
    float Rp = sqrtf(X*X + Y*Y);
    float phi = atan2f(Y, X);

    // 3. 计算K
    float K = (BC*BC - AB*AB - X*X - Y*Y) / (2.0f * AB * Rp);
    if (K > 1.0f) K = 1.0f;
    if (K < -1.0f) K = -1.0f;                    

    // 4. 求解gamma（肘上构型）
    float gammaRad = -phi + acosf(K);
    *gamma = gammaRad * RAD_TO_DEG;

    // 5. 求解beta
    float betaRad = gammaRad + atan2f(Y - AB * sinf(gammaRad), X + AB * cosf(gammaRad));
    *beta = betaRad * RAD_TO_DEG;

   // 显示计算结果
    OLED_Showdecimal(0, 4, *alpha, 3, 2, 16, 1);    
    OLED_Showdecimal(50, 4, *beta, 3, 2, 16, 1);
    OLED_Showdecimal(0, 6, *gamma, 3, 2, 16, 1);
    return JIXIEBI_OK;
}
/**
  * @brief  移动机械臂到指定坐标
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @param  x,y,z: 目标位置坐标(mm)
  * @param  jiaodu: 是否进行角度校准 (true-校准,false-不校准)
  * @retval 状态码
  * @note   先进行逆运动学计算，然后控制电机移动到对应角度
  */
JiXieBiStatus_t JiXieBi_YiDongDaoWeiZhi(JiXieBi_t *jixiebi, float x, float y, float z, bool jiaodu)
{
    float alpha, beta, gamma;// 计算得到的角度(角度)
    
    /* 计算逆运动学 */
    JiXieBi_NiYunDongXue(jixiebi, x, y, z, &alpha, &beta, &gamma);// 计算得到的角度(角度)
    /* 移动到计算出的角度 */
    return JiXieBi_YiDongDaoJiaoDu(jixiebi, alpha, beta, gamma);// 控制电机移动到对应角度
}

/**
    * @brief  移动机械臂到指定角度
    * @param  jixiebi: 指向机械臂配置结构体的指针
    * @param  alpha,beta,gamma: 目标角度(角度)
    *         - alpha: 底座旋转角，以X轴正方向为0°基准
    *         - beta: 大臂与XY平面的夹角，XY平面为0°，向上为正角度
    *         - gamma: 大臂与小臂的夹角
    * @retval 状态码
    * @note   控制三个电机同步运动到指定角度
    */
JiXieBiStatus_t JiXieBi_YiDongDaoJiaoDu(JiXieBi_t *jixiebi, float alpha, float beta, float gamma)
{
        /* 计算角度差值（角度） */
        float deltaAlpha = alpha - jixiebi->danqianAlpha;
        float deltaBeta = beta - jixiebi->danqianBeta;
        float deltaGamma = gamma - jixiebi->danqianGamma;
        
        /* 大臂运动会同时改变大臂和小臂的角度，计算小臂实际需要的运动量 */
        float xiaobiActualDelta = deltaGamma - deltaBeta;  // 小臂相对于大臂的运动
        
        /* 转换为步进电机脉冲数 */
        uint32_t pulseAlpha = (uint32_t)fabsf(deltaAlpha * jixiebi->dizuoMaiChong);
        uint32_t pulseBeta = (uint32_t)fabsf(deltaBeta * jixiebi->dabiMaiChong);
        uint32_t pulseGamma = (uint32_t)fabsf(xiaobiActualDelta * jixiebi->xiaobiMaiChong);
        
        /* 确定方向 *///---------------------------------------------------------------根据实际电机安装位置修改-------------------------------------------------------------------
        uint8_t dirAlpha = (deltaAlpha >= 0) ? 1 : 0;  /* 0为CW（顺时针），1为CCW（逆时针） */
        uint8_t dirBeta = (deltaBeta >= 0) ? 1 : 0;  /* 0为CW（顺时针），1为CCW（逆时针） */
        /* 注意：小臂的方向取决于大臂的运动方向 */
        uint8_t dirGamma = (xiaobiActualDelta >= 0) ? 0 : 1;// 小臂的方向取决于大臂的运动方向
        
        /* 发送位置控制命令，使用多机同步运动 */
        Emm_V5_Pos_Control(jixiebi->dizuoAddr, dirAlpha, 15, 3, pulseAlpha, 0, 1);//底座旋转电机
        JiXieBi_YanShi(5); 
        
        Emm_V5_Pos_Control(jixiebi->dabiAddr, dirBeta, 200, 100, pulseBeta, 0, 1);//大臂电机
        JiXieBi_YanShi(5);
        
        Emm_V5_Pos_Control(jixiebi->xiaobiAddr, dirGamma, 160,80, pulseGamma, 0, 1);//小臂电机
        JiXieBi_YanShi(5);
        
        /* 同时触发运行 */
        Emm_V5_Synchronous_motion(0);
        
        /* 计算运动时间并等待 */
        uint32_t maxPulse = (pulseAlpha > pulseBeta) ? 
                                             ((pulseAlpha > pulseGamma) ? pulseAlpha : pulseGamma) : 
                                             ((pulseBeta > pulseGamma) ? pulseBeta : pulseGamma);
                                             
        /* 根据脉冲数和速度估算运动时间 */
        uint32_t moveTimeMs = (maxPulse * 3 ) /(JIXIEBI_DEF_SUDU *16) + 500;
        if(pulseAlpha!=0)
        {
         JiXieBi_YanShi(moveTimeMs*1.9+10900);// 等待运动完成
        }
        else
				{
					JiXieBi_YanShi(moveTimeMs*0.45);// 等待运动完成
        }
        /* 更新当前角度 */
        jixiebi->danqianAlpha = alpha;
        jixiebi->danqianBeta = beta;
        jixiebi->danqianGamma = gamma;
        
        /* 更新当前位置 */
        float alphaRad = alpha * DEG_TO_RAD;
        float betaRad = beta * DEG_TO_RAD;
        float gammaRad = gamma * DEG_TO_RAD;
        float thetaRad = jixiebi->theta * DEG_TO_RAD;
        
        /* 计算A点坐标（肩关节） */
        float Ax = jixiebi->OA * cosf(thetaRad) * cosf(alphaRad);
        float Ay = jixiebi->OA * cosf(thetaRad) * sinf(alphaRad);
        float Az = jixiebi->OA * sinf(thetaRad);
        
        /* 计算B点坐标（肘关节） - beta是大臂与XY平面的夹角 */
        float AB_xy = jixiebi->AB * cosf(betaRad);
        float AB_z = jixiebi->AB * sinf(betaRad);
        
        float Bx = Ax + AB_xy * cosf(alphaRad);
        float By = Ay + AB_xy * sinf(alphaRad);
        float Bz = Az + AB_z;
        
        /* 计算C点坐标（末端） */
        float BC_xy = jixiebi->BC * cosf(betaRad + gammaRad);
        float BC_z = jixiebi->BC * sinf(betaRad + gammaRad);
        
        jixiebi->danqianX = Bx + BC_xy * cosf(alphaRad);
        jixiebi->danqianY = By + BC_xy * sinf(alphaRad);
        jixiebi->danqianZ = Bz + BC_z;
        
        
        return JIXIEBI_OK;
}

/**
    * @brief  校准机械臂位置
    * @param  jixiebi: 指向机械臂配置结构体的指针
    * @retval 状态码
    * @note   使用碰撞开关校准机械臂位置，纠正丢步问题
    */
JiXieBiStatus_t JiXieBi_JiaoZhunWeiZhi(JiXieBi_t *jixiebi)
{
        /* 如果未启用碰撞开关校准，直接返回 */
        if (!jixiebi->pengzhuangJiaoZhun) {
                return JIXIEBI_OK;
        }
        
        /* 逐一校准各轴位置 */
        
        /* 1. 底座轴校准 */
        /* 先向负方向运动直到触发碰撞开关 */
        Emm_V5_Vel_Control(jixiebi->dizuoAddr, 1, JIXIEBI_DEF_SUDU/8, 10, false);
        
        /* 等待碰撞开关触发 */
        while (JiXieBi_DuQuPengZhuang(PENGZHUANG_DIZUO_PORT, PENGZHUANG_DIZUO_PIN) == 0) {
            JiXieBi_YanShi(1); // 添加1ms延时避免CPU占用过高
        }// 等待碰撞开关触发
        
        /* 停止电机 */
        Emm_V5_Stop_Now(jixiebi->dizuoAddr, false);
        JiXieBi_YanShi(10);
        
        /* 设置当前位置为零点 */
        Emm_V5_Reset_CurPos_To_Zero(jixiebi->dizuoAddr);
        JiXieBi_YanShi(100);
        
        /* 2. 大臂校准 */
        /* 向负方向运动直到触发碰撞开关 */
        Emm_V5_Vel_Control(jixiebi->dabiAddr, 1, JIXIEBI_DEF_SUDU/8, 10, false);
        
        /* 等待碰撞开关触发 */
        while (JiXieBi_DuQuPengZhuang(PENGZHUANG_DABI_PORT, PENGZHUANG_DABI_PIN) == 0) {
            JiXieBi_YanShi(1); // 添加1ms延时避免CPU占用过高
        }// 等待碰撞开关触发
        
        /* 停止电机 */
        Emm_V5_Stop_Now(jixiebi->dabiAddr, false);
        JiXieBi_YanShi(10);
        
        /* 设置当前位置为零点 */
        Emm_V5_Reset_CurPos_To_Zero(jixiebi->dabiAddr);
        JiXieBi_YanShi(10);
        
        /* 3. 小臂校准 */
        /* 向负方向运动直到触发碰撞开关 */
        Emm_V5_Vel_Control(jixiebi->xiaobiAddr, 1, JIXIEBI_DEF_SUDU/8, 10, false);
        
        /* 等待碰撞开关触发 */
        while (JiXieBi_DuQuPengZhuang(PENGZHUANG_XIAOBI_PORT, PENGZHUANG_XIAOBI_PIN) == 0) {
            JiXieBi_YanShi(1); // 添加1ms延时避免CPU占用过高
        }// 等待碰撞开关触发
        
        /* 停止电机 */
        Emm_V5_Stop_Now(jixiebi->xiaobiAddr, false);
        JiXieBi_YanShi(10);

        /* 设置当前位置为零点 */
        Emm_V5_Reset_CurPos_To_Zero(jixiebi->xiaobiAddr);
        JiXieBi_YanShi(10);
        
        /* 校准点就是初始位置，重新设置机械臂当前角度为初始角度 */
        jixiebi->danqianAlpha = jixiebi->alpha0;
        jixiebi->danqianBeta = jixiebi->beta0;
        jixiebi->danqianGamma = jixiebi->gamma0;
        
        /* 更新当前位置为初始位置 */
        float alphaRad = jixiebi->alpha0 * DEG_TO_RAD;
        float betaRad = jixiebi->beta0 * DEG_TO_RAD;
        float gammaRad = jixiebi->gamma0 * DEG_TO_RAD;
        float thetaRad = jixiebi->theta * DEG_TO_RAD;
        
        /* 计算A点坐标（肩关节） */
        float Ax = jixiebi->OA * cosf(thetaRad) * cosf(alphaRad);
        float Ay = jixiebi->OA * cosf(thetaRad) * sinf(alphaRad);
        float Az = jixiebi->OA * sinf(thetaRad);
        
        /* 计算B点坐标（肘关节） - beta是大臂与XY平面的夹角 */
        float AB_xy = jixiebi->AB * cosf(betaRad);
        float AB_z = jixiebi->AB * sinf(betaRad);
        
        float Bx = Ax + AB_xy * cosf(alphaRad);
        float By = Ay + AB_xy * sinf(alphaRad);
        float Bz = Az + AB_z;
        
        /* 计算C点坐标（末端） */
        float BC_xy = jixiebi->BC * cosf(betaRad + gammaRad);
        float BC_z = jixiebi->BC * sinf(betaRad + gammaRad);
        
        jixiebi->danqianX = Bx + BC_xy * cosf(alphaRad);
        jixiebi->danqianY = By + BC_xy * sinf(alphaRad);
        jixiebi->danqianZ = Bz + BC_z;
        
        return JIXIEBI_OK;
}

/**
  * @brief  线性插补运动函数
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @param  mubiaoX,mubiaoY,mubiaoZ: 目标位置(mm)
  * @param  sudu: 运动速度(mm/s)
  * @retval 状态码
  * @note   实现从当前位置到目标位置的线性插补运动
  */
JiXieBiStatus_t JiXieBi_XianXingYiDong(JiXieBi_t *jixiebi, float mubiaoX, float mubiaoY, float mubiaoZ, float sudu)
{
    /* 计算起点和终点之间的距离 */
    float dx = mubiaoX - jixiebi->danqianX;
    float dy = mubiaoY - jixiebi->danqianY;
    float dz = mubiaoZ - jixiebi->danqianZ;
    float distance = sqrtf(dx*dx + dy*dy + dz*dz);
    
    /* 计算总移动时间(s) */
    float totalTime = distance / sudu;
    
    /* 计算采样点数量 (假设每50ms采样一次) */
    int segments = (int)(totalTime * 20.0f); // 20Hz采样率
    segments = (segments < 5) ? 5 : segments; // 至少分成5段
    
    /* 逐点插补运动 */
    float stepX = dx / segments;
    float stepY = dy / segments;
    float stepZ = dz / segments;
    
    JiXieBiStatus_t status = JIXIEBI_OK;
    
    /* 沿直线路径逐点移动 */
    for (int i = 1; i <= segments; i++) {
        float x = jixiebi->danqianX + stepX * i;
        float y = jixiebi->danqianY + stepY * i;
        float z = jixiebi->danqianZ + stepZ * i;
        
        status = JiXieBi_YiDongDaoWeiZhi(jixiebi, x, y, z, false);
        if (status != JIXIEBI_OK) {
            return JiXieBi_XianXingYiDong(jixiebi, mubiaoX, mubiaoY, mubiaoZ, sudu);// 如果运动失败，尝试重新执行
        }
    }
    
    return status;
}

/**
  * @brief  将机械臂移动到初始位置（回零）
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @note   使用内置回零功能将机械臂恢复到初始位置
  */
void JiXieBi_HuiLing(JiXieBi_t *jixiebi)
{
    /* 使用内置回零功能 - 单圈就近回零模式 */
    Emm_V5_Origin_Trigger_Return(jixiebi->dizuoAddr, 0, true);
    JiXieBi_YanShi(10);
    
    Emm_V5_Origin_Trigger_Return(jixiebi->dabiAddr, 0, true);
    JiXieBi_YanShi(10);
    
    Emm_V5_Origin_Trigger_Return(jixiebi->xiaobiAddr, 0, true);
    JiXieBi_YanShi(10);
    
    /* 触发同步运行 */
    Emm_V5_Synchronous_motion(0);
    
    /* 等待足够长的时间确保回零完成 */
    JiXieBi_YanShi(5000);
    
    /* 如果回零后需要校准位置 */
    if (jixiebi->pengzhuangJiaoZhun) 
    {
        JiXieBi_JiaoZhunWeiZhi(jixiebi);
    } 
    else 
    {
        /* 重置当前角度为初始角度 */
        jixiebi->danqianAlpha = jixiebi->alpha0;
        jixiebi->danqianBeta = jixiebi->beta0;
        jixiebi->danqianGamma = jixiebi->gamma0;
        
        /* 更新当前位置为初始位置 */
        float alphaRad = jixiebi->alpha0 * DEG_TO_RAD;
        float betaRad = jixiebi->beta0 * DEG_TO_RAD;
        float gammaRad = jixiebi->gamma0 * DEG_TO_RAD;
        float thetaRad = jixiebi->theta * DEG_TO_RAD;
        
        float Ax = jixiebi->OA * cosf(thetaRad) * cosf(alphaRad);
        float Ay = jixiebi->OA * cosf(thetaRad) * sinf(alphaRad);
        float Az = jixiebi->OA * sinf(thetaRad);
        
        /* 计算B点坐标（肘关节） - beta是大臂与XY平面的夹角 */
        float AB_xy = jixiebi->AB * cosf(betaRad);
        float AB_z = jixiebi->AB * sinf(betaRad);
        
        float Bx = Ax + AB_xy * cosf(alphaRad);
        float By = Ay + AB_xy * sinf(alphaRad);
        float Bz = Az + AB_z;
        
        /* 计算C点坐标（末端） */
        float BC_xy = jixiebi->BC * cosf(betaRad + gammaRad);
        float BC_z = jixiebi->BC * sinf(betaRad + gammaRad);
        
        jixiebi->danqianX = Bx + BC_xy * cosf(alphaRad);
        jixiebi->danqianY = By + BC_xy * sinf(alphaRad);
        jixiebi->danqianZ = Bz + BC_z;
    }
}

/**
  * @brief  停止机械臂运动
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @retval 状态码
  * @note   紧急停止所有电机运动
  */
JiXieBiStatus_t JiXieBi_TingZhi(JiXieBi_t *jixiebi)
{
    /* 立即停止所有电机 */
    Emm_V5_Stop_Now(5, true);// 停止所有电机
    HAL_Delay(10); // 等待10ms确保停止完成
    Emm_V5_Stop_Now(6, true);// 停止所有电机
    HAL_Delay(10); // 等待10ms确保停止完成
    Emm_V5_Stop_Now(7, true);// 停止所有电机
    HAL_Delay(10); // 等待10ms确保停止完成
    Emm_V5_Synchronous_motion(0);

    return JIXIEBI_OK;
}

/**
  * @brief  启用/禁用机械臂电机
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @param  shiNeng: true启用电机，false禁用电机
  * @retval 状态码
  * @note   控制所有电机的使能状态
  */
JiXieBiStatus_t JiXieBi_ShiNeng(JiXieBi_t *jixiebi, bool shiNeng)
{
    /* 控制所有电机使能 */
    Emm_V5_En_Control(0, shiNeng, false);
    return JIXIEBI_OK;
}

/**
  * @brief  码垛功能
  * @param  jixiebi: 指向机械臂配置结构体的指针
  * @param  dianWei: 码垛点位数组，每个点包含x,y,z三个坐标
  * @param  hangShu: 行数
  * @param  lieShu: 列数
  * @param  gao: 抬升高度
  * @retval 状态码
  * @note   执行码垛任务，在指定点位进行放置和取料
  */
JiXieBiStatus_t JiXieBi_MaDuo(JiXieBi_t *jixiebi, float dianWei[][3], int hangShu, int lieShu, float gao)
{
    /* 定义取料点位置 - 假设是原点附近 */
    float quLiaoDian[3] = {150.0f, 0.0f, 0.0f};
    JiXieBiStatus_t status = JIXIEBI_OK;
    
    /* 执行码垛任务 */
    for (int hang = 0; hang < hangShu; hang++) {
        for (int lie = 0; lie < lieShu; lie++) {
            int index = hang * lieShu + lie;
            
            /* 移动到取料点上方 */
            status = JiXieBi_XianXingYiDong(jixiebi, quLiaoDian[0], quLiaoDian[1], quLiaoDian[2] + gao, 50.0f);
            if (status != JIXIEBI_OK) return status;
            
            /* 下降到取料点 */
            status = JiXieBi_XianXingYiDong(jixiebi, quLiaoDian[0], quLiaoDian[1], quLiaoDian[2], 20.0f);
            if (status != JIXIEBI_OK) return status;
            
            /* 模拟抓取动作 - 这里可以添加电磁铁或气动控制 */
            JiXieBi_YanShi(500);
            
            /* 抬升 */
            status = JiXieBi_XianXingYiDong(jixiebi, quLiaoDian[0], quLiaoDian[1], quLiaoDian[2] + gao, 30.0f);
            if (status != JIXIEBI_OK) return status;
            
            /* 移动到码垛点上方 */
            status = JiXieBi_XianXingYiDong(jixiebi, 
                                         dianWei[index][0], 
                                         dianWei[index][1], 
                                         dianWei[index][2] + gao,
                                         40.0f);
            if (status != JIXIEBI_OK) return status;
            
            /* 下降到码垛点 */
            status = JiXieBi_XianXingYiDong(jixiebi, 
                                         dianWei[index][0], 
                                         dianWei[index][1], 
                                         dianWei[index][2],
                                         20.0f);
            if (status != JIXIEBI_OK) return status;
            
            /* 模拟放置动作 */
            JiXieBi_YanShi(500);
            
            /* 抬升 */
            status = JiXieBi_XianXingYiDong(jixiebi, 
                                         dianWei[index][0], 
                                         dianWei[index][1], 
                                         dianWei[index][2] + gao,
                                         30.0f);
            if (status != JIXIEBI_OK) return status;
            
            
        }
    }
    
    /* 回到初始位置 */
    return JIXIEBI_OK;
}

