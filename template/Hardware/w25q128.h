#ifndef W25Q128_H
#define W25Q128_H

#include "main.h"

/**
 * @brief 读取天空星核心板 W25Q128 的三字节 JEDEC ID。
 * @param hspi 已初始化的 SPI1 句柄，使用 8 位全双工、软件片选。
 * @param id 至少三字节的输出缓冲区，依次为厂商、类型和容量编号。
 * @return HAL_OK 表示传输完成；空指针返回 HAL_ERROR，否则返回 HAL 状态。
 * @pre PA4 已配置为推挽输出且为高电平；SPI1 已初始化，当前无其他传输。
 * @note 阻塞调用，超时 20ms。失败时不修改 id，传输结束总是释放片选。
 *       HAL_OK 不代表型号匹配，调用者需检查 ID，尤其全 FF / 全 00。
 * @code
 * uint8_t id[3];
 * HAL_StatusTypeDef status = W25Q128_ReadJedecId(&hspi1, id);
 * @endcode
 */
HAL_StatusTypeDef W25Q128_ReadJedecId(SPI_HandleTypeDef *hspi, uint8_t id[3]);

/**
 * @brief 使用 0x90 命令读取厂商和设备 ID，用于与 JEDEC ID 对照。
 * @param hspi 已初始化的 SPI1 句柄，使用 8 位全双工、软件片选。
 * @param id 至少两字节的输出缓冲区，依次为厂商和设备编号。
 * @return HAL_OK 表示传输完成；空指针返回 HAL_ERROR，否则返回 HAL 状态。
 * @pre PA4 已配置为推挽输出且为高电平；SPI1 已初始化，当前无其他传输。
 * @note 超时 20ms。失败不修改 id，传输结束总是释放片选。
 *       HAL_OK 不代表型号匹配，全 FF / 全 00 由调用者判断。
 */
HAL_StatusTypeDef W25Q128_ReadDeviceId(SPI_HandleTypeDef *hspi, uint8_t id[2]);

/** @brief 读SR1；成功更新输出，失败保留输出。BUSY=bit0，WEL=bit1。 */
HAL_StatusTypeDef W25Q128_ReadStatus(SPI_HandleTypeDef *hspi, uint8_t *status);
/** @brief 读取1..256字节；地址必须在16MiB范围内，失败保留输出。 */
HAL_StatusTypeDef W25Q128_ReadData(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint16_t length);
/** @brief 启动4KiB扇区擦除，地址须4KiB对齐；内部检查BUSY和WEL。
 * @return HAL_OK仅表示命令发出，须轮询SR1并回读确认；不等待擦除完成。 */
HAL_StatusTypeDef W25Q128_EraseSectorStart(SPI_HandleTypeDef *hspi, uint32_t address);
/** @brief 启动页编程，1..256字节且不可跨页；先擦除目标区域。
 * @return HAL_OK仅表示命令发出，须轮询SR1并回读确认；不等待编程完成。 */
HAL_StatusTypeDef W25Q128_ProgramStart(SPI_HandleTypeDef *hspi, uint32_t address, const uint8_t *data, uint16_t length);

#endif
