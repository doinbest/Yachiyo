/**
  ******************************************************************************
  * @file    JiXieBi.h
  * @brief   3轴码垛机械臂驱动头文件
  * @details 本文件定义了3轴码垛机械臂所需的参数和函数接口
  ******************************************************************************
  */

#ifndef __JIXIEBI_H
#define __JIXIEBI_H

#include "main.h"
#include "stdio.h"
#include "bjdj.h"
#include <math.h>
#include <stdbool.h>
#include "oled.h"
/* 数学常量定义 */
#define M_PI_JIXIEBI 3.1415926         // 圆周率
#define RAD_TO_DEG (180.0f / M_PI_JIXIEBI)  // 弧度转角度
#define DEG_TO_RAD (M_PI_JIXIEBI / 180.0f)  // 角度转弧度

/* 碰撞开关引脚定义 */
#define PENGZHUANG_DIZUO_PORT    GPIOB      // 底座旋转轴碰撞开关端口
#define PENGZHUANG_DIZUO_PIN     GPIO_PIN_0 // 底座旋转轴碰撞开关引脚
#define PENGZHUANG_DABI_PORT     GPIOB      // 大臂碰撞开关端口
#define PENGZHUANG_DABI_PIN      GPIO_PIN_1 // 大臂碰撞开关引脚
#define PENGZHUANG_XIAOBI_PORT   GPIOB      // 小臂碰撞开关端口
#define PENGZHUANG_XIAOBI_PIN    GPIO_PIN_2 // 小臂碰撞开关引脚

/* 电机默认参数 */
#define JIXIEBI_DEF_SUDU   5     // 默认速度(RPM)
#define JIXIEBI_DEF_JIASU  0     // 默认加速度(0-255)

/*机械臂重力误差*/
#define JIXIEBI_G_mm  20.0f  // 机械臂重力误差(mm)

/* 机械臂爪子的部分 */
#define JIXIEBI_zz_x  128.28f  // 爪子x轴坐标(mm)
#define JIXIEBI_zz_y  50.06f   // 爪子y轴坐标(mm)

/**
  * @brief  机械臂参数结构体
  * @note   包含了机械臂的物理参数、电机参数和当前状态
  */
typedef struct {
    /* 机械臂物理参数 */
    float OA;        /* 底座到第一关节的长度(mm) *///-------34mm
    float AB;        /* 大臂长度(mm) *///-------------110mm
    float BC;        /* 小 m臂长度(mm) *///-------------150mm
    float theta;     /* OA与XY平面的夹角(角度) *///------------
    
    float alpha0;    /* 初始alpha角度(角度) - 底座旋转角 */
    float beta0;     /* 初始beta角度(角度) - 大臂与XY平面夹角 */
    float gamma0;    /* 初始gamma角度(角度) - 大臂与小臂夹角 */
    
    /* 步进电机参数 */
    uint8_t dizuoAddr;  /* 底座旋转电机地址 */
    uint8_t dabiAddr;   /* 大臂电机地址 */
    uint8_t xiaobiAddr; /* 小臂电机地址 */
    
    /* 电机脉冲与角度换算 */
    float dizuoMaiChong;  /* 底座旋转轴每度对应的脉冲数 */
    float dabiMaiChong;   /* 大臂每度对应的脉冲数 */
    float xiaobiMaiChong; /* 小臂每度对应的脉冲数 */
    
    /* 当前位置 */
    float danqianX;      /* 当前X坐标(mm) */
    float danqianY;      /* 当前Y坐标(mm) */
    float danqianZ;      /* 当前Z坐标(mm) */
    
    /* 当前角度 */
    float danqianAlpha;  /* 当前底座旋转角(角度) */
    float danqianBeta;   /* 当前大臂与XY平面夹角(角度) */
    float danqianGamma;  /* 当前小臂角度(角度) */
    
    /* 丢步检测与校准参数 */
    uint8_t diubuJianCe;      /* 是否启用丢步检测，1-启用，0-禁用 */
    uint8_t pengzhuangJiaoZhun; /* 是否启用碰撞开关校准，1-启用，0-禁用 */
    float diubuYuZhi;         /* 丢步检测阈值(度) */
    
} JiXieBi_t;

/**
  * @brief  错误码定义
  */
typedef enum {
    JIXIEBI_OK = 0,       /* 操作成功 */
    JIXIEBI_ERROR,        /* 一般错误 */
    JIXIEBI_BUDADA,       /* 目标点不可达 */
    JIXIEBI_GUANJIE,      /* 关节角度超出限制 */
    JIXIEBI_DIUBU         /* 检测到丢步 */
} JiXieBiStatus_t;

/* 公共函数声明 */
JiXieBiStatus_t JiXieBi_ChuShiHua(JiXieBi_t *jixiebi);
JiXieBiStatus_t JiXieBi_NiYunDongXue(JiXieBi_t *jixiebi, float x, float y, float z, 
                                    float *alpha, float *beta, float *gamma);
JiXieBiStatus_t JiXieBi_YiDongDaoWeiZhi(JiXieBi_t *jixiebi, float x, float y, float z, bool jiaodu);
JiXieBiStatus_t JiXieBi_YiDongDaoJiaoDu(JiXieBi_t *jixiebi, float alpha, float beta, float gamma);
void JiXieBi_HuiLing(JiXieBi_t *jixiebi);
JiXieBiStatus_t JiXieBi_XianXingYiDong(JiXieBi_t *jixiebi, float mubiaoX, float mubiaoY, float mubiaoZ, float sudu);
JiXieBiStatus_t JiXieBi_TingZhi(JiXieBi_t *jixiebi);
JiXieBiStatus_t JiXieBi_ShiNeng(JiXieBi_t *jixiebi, bool shiNeng);
JiXieBiStatus_t JiXieBi_JiaoZhunWeiZhi(JiXieBi_t *jixiebi);
JiXieBiStatus_t JiXieBi_MaDuo(JiXieBi_t *jixiebi, float dianWei[][3], int hangShu, int lieShu, float gao);

#endif /* __JIXIEBI_H */

