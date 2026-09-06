/* config_store 实现：NVS 薄封装。 */
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "config_store.h"

static const char *TAG = "cfgstore";

#define CFG_NS "wheelcfg"
#define CFG_KEY "params"

esp_err_t config_store_load(cfg_params_t *out)
{
    cfg_params_default(out);

    nvs_handle_t h;
    esp_err_t e = nvs_open(CFG_NS, NVS_READONLY, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        /* 命名空间首次不存在（新分区/首次上电）→ 默认值并回写 */
        ESP_LOGI(TAG, "first boot: writing default config to NVS");
        return config_store_save(out);
    }
    if (e != ESP_OK) {
        return e;
    }
    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    size_t len = sizeof(blob);
    e = nvs_get_blob(h, CFG_KEY, blob, &len);
    nvs_close(h);
    if (e == ESP_OK) {
        return cfg_params_unpack(blob, len, out);
    }
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        /* 键不存在（namespace 存在但没写过）→ 默认值并回写 */
        return config_store_save(out);
    }
    return e;
}

esp_err_t config_store_save(const cfg_params_t *cfg)
{
    uint8_t blob[CFG_PARAMS_BLOB_SIZE];
    ESP_RETURN_ON_ERROR(cfg_params_pack(cfg, blob, sizeof(blob)), TAG, "err");

    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(CFG_NS, NVS_READWRITE, &h), TAG, "err");
    esp_err_t e = nvs_set_blob(h, CFG_KEY, blob, sizeof(blob));
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    return e;
}
