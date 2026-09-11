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
        esp_err_t ue = cfg_params_unpack(blob, len, out);
        if (ue != ESP_OK) {
            return ue;
        }
        /* 读到旧版 blob（一期固件写下的 v1）：解出的新字段是默认值，
         * 立刻回写 v2，之后 NVS 恒为最新布局，不必每次上电都迁移一次。 */
        if (blob[0] != CFG_PARAMS_BLOB_VERSION) {
            ESP_LOGW(TAG, "config blob v%u migrated to v%d", (unsigned)blob[0],
                     CFG_PARAMS_BLOB_VERSION);
            /* 回写失败不影响本次使用（RAM 里的配置是有效的），记日志即可 */
            esp_err_t se = config_store_save(out);
            if (se != ESP_OK) {
                ESP_LOGE(TAG, "migrated config save failed: %d", se);
            }
        }
        return ESP_OK;
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
