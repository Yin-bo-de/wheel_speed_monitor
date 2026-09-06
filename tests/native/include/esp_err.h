/*
 * 宿主机测试专用的 esp_err 极简定义/宏（签名与 IDF esp_check.h 对齐，
 * 使组件源码在双环境用同一调用形式）。IDF 构建命中 esp_common 的真实定义
 * （include 路径先前优先），不会命中此文件。
 * 错误码取值与 ESP-IDF esp_err.h 保持一致。
 */
#ifndef ESP_ERR_SHIM_H
#define ESP_ERR_SHIM_H

#include <stdint.h>

typedef int esp_err_t;

#define ESP_OK                    0
#define ESP_FAIL                  -1
#define ESP_ERR_NO_MEM            0x101
#define ESP_ERR_INVALID_ARG       0x102
#define ESP_ERR_INVALID_STATE     0x103
#define ESP_ERR_INVALID_SIZE      0x104
#define ESP_ERR_NOT_FOUND         0x105
#define ESP_ERR_NOT_SUPPORTED     0x106
#define ESP_ERR_TIMEOUT           0x107
#define ESP_ERR_INVALID_RESPONSE  0x108
#define ESP_ERR_INVALID_CRC       0x109
#define ESP_ERR_INVALID_VERSION   0x10A

/* 与 IDF esp_check.h 签名对齐（log_tag/format 双环境兼容的调用形式） */
#define ESP_RETURN_ON_ERROR(x, log_tag, format, ...) \
    do { \
        esp_err_t err_rc_ = (x); \
        if (err_rc_ != ESP_OK) { \
            return err_rc_; \
        } \
    } while (0)

#define ESP_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) \
    do { \
        esp_err_t err_rc_ = (x); \
        if (err_rc_ != ESP_OK) { \
            goto goto_tag; \
        } \
    } while (0)

#endif /* ESP_ERR_SHIM_H */
