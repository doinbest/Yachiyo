#ifndef HWT101_I2C_H
#define HWT101_I2C_H

#include "main.h"
#include <stdbool.h>

#define HWT101_I2C_ADDRESS 0x50U
#define HWT101_ANGLE_REGISTER 0x3FU
#define HWT101_DATA_FRESH_MS 300U

/** HWT101单Z轴角度，单位为度；roll/pitch仅为旧模板兼容字段，恒为0。 */
typedef struct
{
  float roll;
  float pitch;
  float yaw;
  uint32_t update_count;
  uint32_t last_update_ms;
} HWT101_Angle_t;

/** HWT101 I2C 通信统计。 */
typedef struct
{
  uint32_t valid_read_count;
  uint32_t i2c_error_count;
  uint32_t last_update_ms;
} HWT101_Status_t;

/**
  * @brief 初始化并探测 HWT101 I2C 从机。
  * @param hi2c 已完成初始化的 I2C2 句柄。
  * @retval true 探测到默认地址0x50；false 参数为空或设备无应答。
  * @note 探测只验证从机地址，不读取角度数据。
  */
bool HWT101_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief 读取 HWT101 的一个 16 位寄存器，按低字节在前解码。
  * @param reg 8 位寄存器地址。
  * @param value 成功时保存寄存器值；失败时保持原值，不能为 NULL。
  * @return I2C 事务的 HAL 状态；句柄未初始化或参数为空时返回 HAL_ERROR。
  * @pre 已调用 HWT101_Init 配置有效的 I2C 句柄；仅在主循环调用。
  * @note 单次阻塞事务超时 5ms。总线失败计入通信错误并触发后续重新探测。
  */
HAL_StatusTypeDef HWT101_ReadRegister(uint8_t reg, uint16_t *value);

/**
  * @brief 写入 HWT101 的一个 16 位寄存器，按低字节在前发送。
  * @param reg 8 位寄存器地址。
  * @param value 要写入的寄存器值。
  * @return I2C 事务的 HAL 状态；句柄未初始化时返回 HAL_ERROR。
  * @pre 已调用 HWT101_Init 配置有效的 I2C 句柄；仅在主循环调用。
  * @note 单次阻塞事务超时 5ms。解锁、等待、保存等协议步骤由调用者安排。
  */
HAL_StatusTypeDef HWT101_WriteRegister(uint8_t reg, uint16_t value);

/**
  * @brief 在主循环轮询HWT101；负责探测、重试和角度I2C读取。
  * @note 最小读取间隔10ms，未就绪每1000ms探测，I2C超时5ms。
  */
void HWT101_Process(void);

/**
  * @brief 非消费式复制最近一次有效的 HWT101 Z 轴角度。
  * @param angle 用于保存角度数据的结构体地址。
  * @retval true 至少已有一次有效采样。
  * @retval false 参数为空或尚无有效采样。
  * @note 本函数不访问I2C，也不消费缓存；调用HWT101_Process更新缓存。
  */
bool HWT101_Angle_Get(HWT101_Angle_t *angle);

/**
  * @brief 判断 HWT101 角度数据是否仍然新鲜。
  * @param angle 最近一次角度数据。
  * @param max_age_ms 允许的最大数据年龄，单位 ms。
  * @retval true 已成功读取过数据且未超时。
  * @retval false 参数为空、尚未成功读取或数据已超时。
  */
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *angle,
                           uint32_t max_age_ms);

/**
  * @brief 非消费式读取 HWT101 的 I2C 通信统计，不访问I2C。
  * @param status 用于保存统计的结构体地址。
  * @retval true 参数有效。
  * @retval false 参数为空。
  */
bool HWT101_Status_Get(HWT101_Status_t *status);

/**
  * @brief 返回最近一次 HWT101 地址探测结果。
  */
bool HWT101_Is_Ready(void);

#endif /* HWT101_I2C_H */
