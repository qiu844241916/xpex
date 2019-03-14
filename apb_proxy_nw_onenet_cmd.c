
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "syslog.h"

#include "apb_proxy.h"
#include "apb_proxy_nw_onenet_cmd.h"
#include "apb_proxy_nw_cmd_gprot.h"
#ifdef MTK_ONENET_SUPPORT
#include "cis_def.h"
#include "cis_api.h"
#include "cis_if_sys.h"
#include "cis_internals.h"
#endif
#include "memory_attribute.h"
#include "hal_rtc_external.h"
#include "hal_rtc_internal.h"
#include "syslog.h"
#include "timers.h"
#include "hal_sleep_manager.h"

log_create_module(onenet_at, PRINT_LEVEL_INFO);

#if !defined(MTK_DEBUG_LEVEL_NONE)
#define ONENET_AT_LOGI(fmt, args...)    LOG_I(onenet_at, "[ONENET_AT] "fmt, ##args)
#define ONENET_AT_LOGW(fmt, args...)    LOG_W(onenet_at, "[ONENET_AT] "fmt, ##args)
#define ONENET_AT_LOGE(fmt, args...)    LOG_E(onenet_at, "[ONENET_AT] "fmt, ##args)
#else
#define ONENET_AT_LOGI(fmt, args...)    printf("[ONENET_AT] "fmt"\r\n", ##args)
#define ONENET_AT_LOGW(fmt, args...)    printf("[ONENET_AT] "fmt"\r\n", ##args)
#define ONENET_AT_LOGE(fmt, args...)    printf("[ONENET_AT] "fmt"\r\n", ##args)
#endif

#if defined(MTK_ONENET_SUPPORT)

#define ONENET_AT_TASK_NAME             "ONENET_AT"
#define ONENET_AT_TASK_STACK_SIZE       (1024 * 2)
#define ONENET_AT_TASK_PRIORITY         (TASK_PRIORITY_NORMAL)

#define ONENET_AT_INSTANCE_NUM          (1)
#define ONENET_AT_CONFIG_MAX_BUFFER_LEN (56)
#define ONENET_AT_CONFIG_BUFFER         "130030F10003F200220400110005434D494F5400000000000D3138332E3233302E34302E33390000F30008E200C80000"
#define ONENET_AT_OBJECT_MAX_COUNT      (2)
#define ONENET_AT_OBJECT_INSTANCE_MAX_COUNT (8)
#define ONENET_AT_RESPONSE_DATA_LEN     (100)
#define ONENET_AT_CMD_STRING_LEN        (1024)
#define ONENET_AT_CMD_PARAM_NUM         (20)
#define ONENET_AT_CMD_FILED_NUM         (100)
#define ONENET_AT_LIFE_TIME             (300)
#define ONENET_AT_LIFE_TIME_UPDATE      (270)

#define ONENET_AT_FLAG_FIRST            (1)
#define ONENET_AT_FLAG_MIDDLE           (2)
#define ONENET_AT_FLAG_LAST             (0)

#define ONENET_AT_ERRID_UNKNOWN_ERROR           (0)
#define ONENET_AT_ERRID_SYSTEM_ERROR            (1)
#define ONENET_AT_ERRID_NETWORK_ERROR           (2)
#define ONENET_AT_ERRID_REGISTRATION_FAILURE    (3)
#define ONENET_AT_ERRID_CONTINUE                (1000)
#define ONENET_AT_ERRID_OK                      (650)
#define ONENET_AT_ERRID_MEMORY_ERROR            (ONENET_AT_ERRID_OK + 1)
#define ONENET_AT_ERRID_PARAMETER_ERROR         (ONENET_AT_ERRID_OK + 2)
#define ONENET_AT_ERRID_NOT_SUPPORT             (ONENET_AT_ERRID_OK + 3)
#define ONENET_AT_ERRID_SDK_ERROR               (ONENET_AT_ERRID_OK + 4)
#define ONENET_AT_ERRID_NOT_FOUND               (ONENET_AT_ERRID_OK + 5)

#define ONENET_AT_CMDID_CREATE                  (0)
#define ONENET_AT_CMDID_DELETE                  (1)
#define ONENET_AT_CMDID_OPEN                    (2)
#define ONENET_AT_CMDID_CLOSE                   (3)
#define ONENET_AT_CMDID_ADD_OBJECT              (4)
#define ONENET_AT_CMDID_DELETE_OBJECT           (5)
#define ONENET_AT_CMDID_UPDATE                  (6)
#define ONENET_AT_CMDID_NOTIFY                  (7)
#define ONENET_AT_CMDID_READ_RESPONSE           (8)
#define ONENET_AT_CMDID_WRITE_RESPONSE          (9)
#define ONENET_AT_CMDID_EXECUTE_RESPONSE        (10)
#define ONENET_AT_CMDID_OBSERVE_RESPONSE        (11)
#define ONENET_AT_CMDID_DISCOVER_RESPONSE       (12)
#define ONENET_AT_CMDID_PARAMETER_RESPONSE      (13)
#define ONENET_AT_CMDID_VERSION                 (14)

typedef enum {
    ONENET_AT_RESULT_205_CONTENT = 1,
    ONENET_AT_RESULT_204_CHANGED = 2,
    ONENET_AT_RESULT_400_BAD_REQUEST = 11,
    ONENET_AT_RESULT_401_UNAUTHORIZED = 12,
    ONENET_AT_RESULT_404_NOT_FOUND = 13,
    ONENET_AT_RESULT_405_METHOD_NOT_ALLOWED = 14,
    ONENET_AT_RESULT_406_NOT_ACCEPTABLE = 15
} onenet_at_result_t;

typedef struct {
    bool is_used;
    cis_oid_t object_id;
    cis_instcount_t instance_count;
    uint8_t instance_bitmap[(ONENET_AT_OBJECT_INSTANCE_MAX_COUNT + 7) / 8];
    cis_attrcount_t attribute_count;
    cis_actcount_t action_count;
} onenet_at_object_info_t;

typedef struct {
    bool is_used;
    bool is_connected;
    uint32_t onenet_id;
    void *cis_context;
    cis_time_t life_time;
    bool use_default_config;
    uint8_t config_bin[ONENET_AT_CONFIG_MAX_BUFFER_LEN];
    uint16_t config_len;
    onenet_at_object_info_t object_info[ONENET_AT_OBJECT_MAX_COUNT];
    uint32_t rtc_handle;
    bool rtc_enable;
    et_client_state_t state;
} onenet_at_context_t;

static ATTR_ZIDATA_IN_RETSRAM onenet_at_context_t g_onenet_at_context[ONENET_AT_INSTANCE_NUM];
static bool g_onenet_at_task_running;
static bool g_onenet_at_need_update;
static bool g_onenet_at_need_deep_sleep_handling;
static TimerHandle_t g_onenet_at_ping_timer;
static uint8_t g_onenet_at_sleep_handle;
static bool g_onenet_at_is_locking;

static onenet_at_context_t *onenet_at_create_instance(void)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    for (i = 0; i < ONENET_AT_INSTANCE_NUM; i++) {
        if (g_onenet_at_context[i].is_used == false) {
            memset(&g_onenet_at_context[i], 0, sizeof(onenet_at_context_t));
            g_onenet_at_context[i].is_used = true;
            g_onenet_at_context[i].onenet_id = i;
            return &g_onenet_at_context[i];
        }
    }

    return NULL;
}

static onenet_at_context_t *onenet_at_search_instance(uint32_t onenet_id)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    for (i = 0; i < ONENET_AT_INSTANCE_NUM; i++) {
        if (g_onenet_at_context[i].is_used == true && g_onenet_at_context[i].onenet_id == onenet_id) {
            return &g_onenet_at_context[i];
        }
    }

    return NULL;
}

static onenet_at_context_t *onenet_at_search_instance_ex(void *cis_context)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    for (i = 0; i < ONENET_AT_INSTANCE_NUM; i++) {
        if (g_onenet_at_context[i].is_used == true && g_onenet_at_context[i].cis_context == cis_context) {
            return &g_onenet_at_context[i];
        }
    }

    return NULL;
}

static void onenet_at_delete_instance(onenet_at_context_t *onenet)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (onenet->is_used == true) {
        onenet->is_used = false;
    }
}

static void onenet_at_retention_save_object(onenet_at_context_t *onenet, cis_oid_t objectid, const cis_inst_bitmap_t *bitmap, const cis_res_count_t *resource)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    onenet_at_object_info_t *object_info;
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    for (i = 0; i < ONENET_AT_OBJECT_MAX_COUNT; i++) {
        object_info = &onenet->object_info[i];
        if (object_info->is_used == 0) {
            object_info->is_used = 1;
            object_info->object_id = objectid;
            object_info->instance_count = bitmap->instanceCount;
            memcpy(object_info->instance_bitmap, bitmap->instanceBitmap, bitmap->instanceBytes);
            object_info->attribute_count = resource->attrCount;
            object_info->action_count = resource->actCount;
            return;
        }
    }
}

static void onenet_at_retention_delete_object(onenet_at_context_t *onenet, cis_oid_t objectid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    onenet_at_object_info_t *object_info;
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    for (i = 0; i < ONENET_AT_OBJECT_MAX_COUNT; i++) {
        object_info = &onenet->object_info[i];
        if (object_info->is_used == 1 && object_info->object_id == objectid) {
            object_info->is_used = 0;
            return;
        }
    }
}

static void onenet_at_retention_delete_all_objects(onenet_at_context_t *onenet)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    onenet_at_object_info_t *object_info;
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    for (i = 0; i < ONENET_AT_OBJECT_MAX_COUNT; i++) {
        object_info = &onenet->object_info[i];
        if (object_info->is_used == 1) {
            object_info->is_used = 0;
        }
    }
}

static void onenet_at_rtc_timer_callback(void *user_data)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    g_onenet_at_need_update = true;
    xTimerStartFromISR(g_onenet_at_ping_timer, &xHigherPriorityTaskWoken);
    hal_sleep_manager_acquire_sleeplock(g_onenet_at_sleep_handle, HAL_SLEEP_LOCK_DEEP);
    g_onenet_at_is_locking = true;
}

static void onenet_at_start_rtc_timer(onenet_at_context_t *onenet)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (onenet->rtc_enable == false) {
        rtc_sw_timer_status_t status = rtc_sw_timer_create(&onenet->rtc_handle, onenet->life_time * 9, true, onenet_at_rtc_timer_callback);
        ONENET_AT_LOGI("rtc_sw_timer_create = %d", (int)status);
        status = rtc_sw_timer_start(onenet->rtc_handle);
        ONENET_AT_LOGI("rtc_sw_timer_start = %d", (int)status);
        onenet->rtc_enable = true;
    } else {
        ONENET_AT_LOGI("onenet_at_start_rtc_timer already");
    }
}

static void onenet_at_stop_rtc_timer(onenet_at_context_t *onenet)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (onenet->rtc_enable == true) {
        rtc_sw_timer_status_t status = rtc_sw_timer_stop(onenet->rtc_handle);
        ONENET_AT_LOGI("rtc_sw_timer_stop = %d", (int)status);
        status = rtc_sw_timer_delete(onenet->rtc_handle);
        ONENET_AT_LOGI("rtc_sw_timer_delete = %d", (int)status);
        onenet->rtc_enable = false;
    } else {
        ONENET_AT_LOGI("onenet_at_stop_rtc_timer already");
    }
}

static int32_t onenet_at_create(uint32_t currentsize, const char *config, uint32_t *onenet_id)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (currentsize % 2 != 0) {
        ONENET_AT_LOGE("length of <currentsize> is odd");
        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
    } else if (config == NULL || strlen(config) == currentsize) {
        onenet_at_context_t *onenet = onenet_at_create_instance();
        if (onenet == NULL) {
            ONENET_AT_LOGE("max instance reached");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
        } else {
            if (currentsize / 2 > ONENET_AT_CONFIG_MAX_BUFFER_LEN) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
                onenet_at_delete_instance(onenet);
            } else {
                if (config != NULL) {
                    onenet->use_default_config = false;
                    onenet->config_len = (uint16_t)onenet_at_hex_to_bin(onenet->config_bin, config, currentsize / 2);
                } else {
                    onenet->use_default_config = true;
                    onenet->config_len = 0;
                }
                ONENET_AT_LOGI("cis_init start: 0x%x, 0x%x, %d", (unsigned int)onenet->cis_context, (unsigned int)onenet->config_bin, (int)onenet->config_len);
                cis_ret_t cis_ret;
                if (onenet->use_default_config) {
                    cis_ret = cis_init(&onenet->cis_context, 0, NULL, NULL, NULL);
                } else {
                    cis_ret = cis_init(&onenet->cis_context, onenet->config_bin, onenet->config_len, NULL, NULL);
                }
                ONENET_AT_LOGI("cis_init end: %d", (int)cis_ret);
                if (cis_ret != CIS_RET_OK) {
                    cis_error = ONENET_AT_ERRID_SDK_ERROR;
                    onenet_at_delete_instance(onenet);
                } else {
                    *onenet_id = onenet->onenet_id;
                }
            }
        }
    } else {
        ONENET_AT_LOGE("length of <config> and <currentsize> are not equal");
        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
    }

    return cis_error;
}

static int32_t onenet_at_delete(uint32_t onenet_id)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        ONENET_AT_LOGI("cis_deinit start: 0x%x", (unsigned int)onenet->cis_context);
        cis_ret_t cis_ret = cis_deinit(&onenet->cis_context);
        ONENET_AT_LOGI("cis_deinit end: %d", (int)cis_ret);
        if (cis_ret != CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        } else {
            onenet_at_stop_rtc_timer(onenet);
            onenet_at_retention_delete_all_objects(onenet);
            onenet_at_delete_instance(onenet);
        }
    }

    return cis_error;
}

static cis_coapret_t onenet_at_read_callback(void *context, cis_uri_t *uri, cis_mid_t mid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return CIS_CALLBACK_BAD_REQUEST;
    } else {
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLREAD: <ref>, <msgid>, <objectid>, <instanceid>, <resourceid>
        if (CIS_URI_IS_SET_RESOURCE(uri)) {
            sprintf(response_data, "+MIPLREAD: %d, %d, %d, %d, %d",
                    (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        } else if (CIS_URI_IS_SET_INSTANCE(uri)) {
            sprintf(response_data, "+MIPLREAD: %d, %d, %d, %d, -1",
                    (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId);
        } else {
            sprintf(response_data, "+MIPLREAD: %d, %d, %d, -1, -1",
                    (int)onenet->onenet_id, (int)mid, (int)uri->objectId);
        }
        ONENET_AT_LOGI("objectId: %d, instanceId: %d, resourceId: %d", (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        response.pdata = response_data;
        response.length = strlen(response.pdata);

        response.cmd_id = APB_PROXY_INVALID_CMD_ID;
        apb_proxy_send_at_cmd_result(&response);
    }

    return CIS_CALLBACK_CONFORM;
}

static cis_coapret_t onenet_at_write_callback(void *context, cis_uri_t *uri, const cis_data_t *value, cis_attrcount_t attrcount, cis_mid_t mid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char *response_data = NULL;
    char *string = NULL;
    uint32_t length;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return CIS_CALLBACK_BAD_REQUEST;
    } else {
        uint32_t i, flag = ONENET_AT_FLAG_LAST;
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLWRITE: <ref>, <msgid>, <objectid>, <instanceid>, <resourceid>, <valuetype>, <len>, <value>, <flag>, <index>
        for (i = 0; i < attrcount; i++) {
            switch (value[i].type) {
                case cis_data_type_string:
                    response_data = (char *)pvPortMalloc(ONENET_AT_RESPONSE_DATA_LEN + value[i].asBuffer.length);
                    if (response_data == NULL) return CIS_CALLBACK_BAD_REQUEST;
                    sprintf(response_data, "+MIPLWRITE: %d, %d, %d, %d, %d, %d, %d, \"%s\", %d, %d",
                            (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId,
                            (int)value[i].type, (int)value[i].asBuffer.length, (char *)value[i].asBuffer.buffer,
                            (int)flag, (int)(attrcount - 1 - i));
                    break;
                case cis_data_type_opaque:
                    length = value[i].asBuffer.length;
                    ONENET_AT_LOGI("length = %d", length);
                    string = (char *)pvPortMalloc(length * 2 + 1);
                    if (string == NULL) return CIS_CALLBACK_BAD_REQUEST;
                    onenet_at_bin_to_hex(string, value[i].asBuffer.buffer, length * 2);
                    response_data = (char *)pvPortMalloc(ONENET_AT_RESPONSE_DATA_LEN + length * 2);
                    if (response_data == NULL) {
                        vPortFree(string);
                        return CIS_CALLBACK_BAD_REQUEST;
                    }
                    sprintf(response_data, "+MIPLWRITE: %d, %d, %d, %d, %d, %d, %d, %s, %d, %d",
                            (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)value[i].id,
                            (int)value[i].type, (int)length, string,
                            (int)flag, (int)(attrcount - 1 - i));
                    break;
                case cis_data_type_integer:
                    response_data = (char *)pvPortMalloc(ONENET_AT_RESPONSE_DATA_LEN);
                    if (response_data == NULL) return CIS_CALLBACK_BAD_REQUEST;
                    sprintf(response_data, "+MIPLWRITE: %d, %d, %d, %d, %d, %d, , %d, %d, %d",
                            (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)value[i].id,
                            (int)value[i].type, (int)value[i].value.asInteger,
                            (int)flag, (int)(attrcount - 1 - i));
                    break;
                case cis_data_type_float:
                    response_data = (char *)pvPortMalloc(ONENET_AT_RESPONSE_DATA_LEN);
                    if (response_data == NULL) return CIS_CALLBACK_BAD_REQUEST;
                    sprintf(response_data, "+MIPLWRITE: %d, %d, %d, %d, %d, %d, , %lf, %d, %d",
                            (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)value[i].id,
                            (int)value[i].type, value[i].value.asFloat,
                            (int)flag, (int)(attrcount - 1 - i));
                    break;
                case cis_data_type_bool:
                    response_data = (char *)pvPortMalloc(ONENET_AT_RESPONSE_DATA_LEN);
                    if (response_data == NULL) return CIS_CALLBACK_BAD_REQUEST;
                    sprintf(response_data, "+MIPLWRITE: %d, %d, %d, %d, %d, %d, , %d, %d, %d",
                            (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)value[i].id,
                            (int)value[i].type, (int)value[i].value.asBoolean,
                            (int)flag, (int)(attrcount - 1 - i));
                    break;
                default:
                    ONENET_AT_LOGE("not support");
                    configASSERT(0);
                    break;
            }
            ONENET_AT_LOGI("objectId: %d, instanceId: %d, resourceId: %d, attrcount: %d", (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId, (int)attrcount);
            response.pdata = response_data;
            response.length = strlen(response.pdata);

            response.cmd_id = APB_PROXY_INVALID_CMD_ID;
            apb_proxy_send_at_cmd_result(&response);

            if (string != NULL) {
                vPortFree(string);
                string = NULL;
            }
            vPortFree(response_data);
        }
    }

    return CIS_CALLBACK_CONFORM;
}

static cis_coapret_t onenet_at_execute_callback(void *context, cis_uri_t *uri, const uint8_t *buffer, uint32_t length, cis_mid_t mid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char *response_data;
    char *display_buffer;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return CIS_CALLBACK_BAD_REQUEST;
    } else {
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLEXECUTE: <ref>, <msgid>, <objectid>, <instanceid>, <resourceid>[, <len>, <arguments>]
        display_buffer = (char *)pvPortMalloc(length + 1);
        if (display_buffer == NULL) return CIS_CALLBACK_BAD_REQUEST;
        memcpy(display_buffer, buffer, length);
        display_buffer[length] = '\0';
        response_data = (char *)pvPortMalloc(ONENET_AT_RESPONSE_DATA_LEN + length);
        if (response_data == NULL) {
            vPortFree(display_buffer);
            return CIS_CALLBACK_BAD_REQUEST;
        }
        sprintf(response_data, "+MIPLEXECUTE: %d, %d, %d, %d, %d, %d, \"%s\"",
                (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId, (int)length, display_buffer);
        ONENET_AT_LOGI("objectId: %d, instanceId: %d, resourceId: %d", (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        response.pdata = response_data;
        response.length = strlen(response.pdata);

        response.cmd_id = APB_PROXY_INVALID_CMD_ID;
        apb_proxy_send_at_cmd_result(&response);

        vPortFree(display_buffer);
        vPortFree(response_data);
    }

    return CIS_CALLBACK_CONFORM;
}
int msgid = 0 ;
static cis_coapret_t onenet_at_observe_callback(void *context, cis_uri_t *uri, bool flag, cis_mid_t mid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return CIS_CALLBACK_BAD_REQUEST;
    } else {
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLOBSERVE: <ref>, <msgid>, <flag>, <objectid>, <instanceid>, <resourceid>
         msgid = (int)mid;
        if (CIS_URI_IS_SET_RESOURCE(uri)) {
            sprintf(response_data, "+MIPLOBSERVE: %d, %d, %d, %d, %d, %d",
                    (int)onenet->onenet_id, (int)mid, (int)flag, (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        } else if (CIS_URI_IS_SET_INSTANCE(uri)) {
            sprintf(response_data, "+MIPLOBSERVE: %d, %d, %d, %d, %d, -1",
                    (int)onenet->onenet_id, (int)mid, (int)flag, (int)uri->objectId, (int)uri->instanceId);
        } else {
            sprintf(response_data, "+MIPLOBSERVE: %d, %d, %d, %d, -1, -1",
                    (int)onenet->onenet_id, (int)mid, (int)flag, (int)uri->objectId);
        }

        ONENET_AT_LOGI("objectId: %d, instanceId: %d, resourceId: %d", (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        response.pdata = response_data;
        response.length = strlen(response.pdata);

        response.cmd_id = APB_PROXY_INVALID_CMD_ID;
        apb_proxy_send_at_cmd_result(&response);
    }

    return CIS_CALLBACK_CONFORM;
}
int msgid1=0;
static cis_coapret_t onenet_at_discover_callback(void *context, cis_uri_t *uri, cis_mid_t mid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return CIS_CALLBACK_BAD_REQUEST;
    } else {
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLDISCOVER: <ref>, <msgid>, <objectid>
        msgid1=(int)mid;
        if (CIS_URI_IS_SET_RESOURCE(uri)) {
            ONENET_AT_LOGE("CIS_URI_IS_SET_RESOURCE");
            return CIS_COAP_405_METHOD_NOT_ALLOWED;
        } else if (CIS_URI_IS_SET_INSTANCE(uri)) {
            ONENET_AT_LOGE("CIS_URI_IS_SET_INSTANCE");
            return CIS_COAP_405_METHOD_NOT_ALLOWED;
        } else {
            sprintf(response_data, "+MIPLDISCOVER: %d, %d, %d", (int)onenet->onenet_id, (int)mid, (int)uri->objectId);
        }
        ONENET_AT_LOGI("objectId: %d, instanceId: %d, resourceId: %d", (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        response.pdata = response_data;
        response.length = strlen(response.pdata);

        response.cmd_id = APB_PROXY_INVALID_CMD_ID;
        apb_proxy_send_at_cmd_result(&response);
    }

    return CIS_CALLBACK_CONFORM;
}

static cis_coapret_t onenet_at_set_parameter_callback(void *context, cis_uri_t *uri, cis_observe_attr_t parameters, cis_mid_t mid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char response_data[2 * ONENET_AT_RESPONSE_DATA_LEN];
    char parameter[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return CIS_CALLBACK_BAD_REQUEST;
    } else {
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLPARAMETER: <ref>, <msgid>, <objectid>, <instanceid>, <resourceid>, <len>, <parameter>
        sprintf(parameter, "pmin=%d; pmax=%d; gt=%lf; lt=%lf; st=%lf",
                (int)parameters.minPeriod, (int)parameters.maxPeriod, (double)parameters.greaterThan, (double)parameters.lessThan, (double)parameters.step);
        sprintf(response_data, "+MIPLPARAMETER: %d, %d, %d, %d, %d, %d, \"%s\"",
                (int)onenet->onenet_id, (int)mid, (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId,
                (int)strlen(parameter), parameter);
        ONENET_AT_LOGI("objectId: %d, instanceId: %d, resourceId: %d", (int)uri->objectId, (int)uri->instanceId, (int)uri->resourceId);
        response.pdata = response_data;
        response.length = strlen(response.pdata);

        response.cmd_id = APB_PROXY_INVALID_CMD_ID;
        apb_proxy_send_at_cmd_result(&response);
    }

    return CIS_CALLBACK_CONFORM;
}
int MIPLEVENT_id = 0;
static void onenet_at_event_callback(void *context, cis_evt_t id, void* param)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (id == CIS_EVENT_UPDATE_SUCCESS && g_onenet_at_is_locking) {
        xTimerStop(g_onenet_at_ping_timer, 0);
        hal_sleep_manager_release_sleeplock(g_onenet_at_sleep_handle, HAL_SLEEP_LOCK_DEEP);
        ONENET_AT_LOGI("hal_sleep_manager_release_sleeplock");
        g_onenet_at_is_locking = false;
    }

    onenet_at_context_t *onenet = onenet_at_search_instance_ex(context);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<context> not found");
        return;
    } else {
        response.result_code = APB_PROXY_RESULT_UNSOLICITED;
        // +MIPLEVENT: <ref>, <evtid>[, <extend>]
        switch (id)
        {
            case CIS_EVENT_RESPONSE_FAILED:
            case CIS_EVENT_NOTIFY_FAILED:
            case CIS_EVENT_UPDATE_NEED:
            case CIS_EVENT_NOTIFY_SUCCESS:
                sprintf(response_data, "+MIPLEVENT: %d, %d, %d", (int)onenet->onenet_id, (int)id, (int)param);
                break;

            default:
                MIPLEVENT_id = (int)id;
                sprintf(response_data, "+MIPLEVENT: %d, %d", (int)onenet->onenet_id, (int)id);
                break;
        }
        ONENET_AT_LOGI("id: %d, param: %d", (int)id, (int)param);
        response.pdata = response_data;
        response.length = strlen(response.pdata);

        response.cmd_id = APB_PROXY_INVALID_CMD_ID;
        apb_proxy_send_at_cmd_result(&response);
    }

    if (id == CIS_EVENT_REG_SUCCESS) {
        onenet_at_start_rtc_timer(onenet);
    } else if (id == CIS_EVENT_UNREG_DONE) {
        onenet_at_stop_rtc_timer(onenet);
    } else if ((id == CIS_EVENT_CONNECT_SUCCESS || id == CIS_EVENT_UPDATE_NEED) && onenet->is_connected && g_onenet_at_need_update) {
        g_onenet_at_need_update = false;
        ONENET_AT_LOGI("cis_update_reg start: 0x%x", (unsigned int)onenet->cis_context);
        cis_ret_t cis_ret = cis_update_reg(onenet->cis_context, LIFETIME_INVALID, false);
        ONENET_AT_LOGI("cis_update_reg: %d", (int)cis_ret);
    }
}

static int32_t onenet_at_open(uint32_t onenet_id, cis_time_t lifetime)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;
    cis_callback_t callback;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        ONENET_AT_LOGI("cis_register start: 0x%x", (unsigned int)onenet->cis_context);
        callback.onRead = onenet_at_read_callback;
        callback.onWrite = onenet_at_write_callback;
        callback.onExec = onenet_at_execute_callback;
        callback.onObserve = onenet_at_observe_callback;
        callback.onDiscover = onenet_at_discover_callback;
        callback.onSetParams = onenet_at_set_parameter_callback;
        callback.onEvent = onenet_at_event_callback;
        cis_ret_t cis_ret = cis_register(onenet->cis_context, lifetime, &callback);
        ONENET_AT_LOGI("cis_register end: %d", (int)cis_ret);
        if (cis_ret != CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        } else {
            onenet->is_connected = true;
            onenet->life_time = lifetime;
            onenet_at_start_rtc_timer(onenet);
        }
    }

    return cis_error;
}

static int32_t onenet_at_close(uint32_t onenet_id)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        ONENET_AT_LOGI("cis_unregister start: 0x%x", (unsigned int)onenet->cis_context);
        cis_ret_t cis_ret = cis_unregister(onenet->cis_context);
        ONENET_AT_LOGI("cis_unregister end: %d", (int)cis_ret);
        if (cis_ret != CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        } else {
            onenet->is_connected = false;
            onenet_at_stop_rtc_timer(onenet);
        }
    }

    return cis_error;
}

static int32_t onenet_at_add_object(uint32_t onenet_id, uint32_t objectid, uint32_t instancecount, const char *instancebitmap, uint32_t attributecount, uint32_t actioncount)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        uint32_t i;
        uint8_t *instancebitmap_bytes;
        cis_inst_bitmap_t bitmap;
        cis_res_count_t resource;
        if (instancebitmap == NULL || instancecount == 0 || strlen(instancebitmap) != instancecount) {
            return ONENET_AT_ERRID_PARAMETER_ERROR;
        }
        bitmap.instanceCount = instancecount;
        bitmap.instanceBytes = (bitmap.instanceCount + 7) / 8;
        instancebitmap_bytes = (uint8_t *)pvPortMalloc(bitmap.instanceBytes + 1);
        if (instancebitmap_bytes == NULL) {
            ONENET_AT_LOGE("memory error");
            cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
        } else {
            memset(instancebitmap_bytes, 0, bitmap.instanceBytes);
            for (i = 0; i < bitmap.instanceCount; i++) {
                uint8_t pos = i / 8;
                uint8_t offset = 7 - (i % 8);
                if (instancebitmap[bitmap.instanceCount - i - 1] == '1') {
                    instancebitmap_bytes[pos] += 0x01 << offset;
                }
            }
            bitmap.instanceBitmap = instancebitmap_bytes;
            resource.attrCount = attributecount;
            resource.actCount = actioncount;
            ONENET_AT_LOGI("cis_addobject start: 0x%x, %d, (%d-%d-0x%x), (%d-%d)",
                           (unsigned int)onenet->cis_context, (int)objectid,
                           (int)bitmap.instanceCount, (int)bitmap.instanceBytes, (unsigned int)bitmap.instanceBitmap,
                           (int)resource.attrCount, (int)resource.actCount);
            cis_ret_t cis_ret = cis_addobject(onenet->cis_context, (cis_oid_t)objectid, &bitmap, &resource);
            ONENET_AT_LOGI("cis_addobject end: %d", (int)cis_ret);
            if (cis_ret != CIS_RET_OK) {
                cis_error = ONENET_AT_ERRID_SDK_ERROR;
            } else {
                onenet_at_retention_save_object(onenet, (cis_oid_t)objectid, &bitmap, &resource);
            }
        }

        if (instancebitmap_bytes != NULL) {
            vPortFree(instancebitmap_bytes);
        }
    }

    return cis_error;
}

static int32_t onenet_at_delete_object(uint32_t onenet_id, uint32_t objectid)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        ONENET_AT_LOGI("cis_delobject start: 0x%x, %d", (unsigned int)onenet->cis_context, (int)objectid);
        cis_ret_t cis_ret = cis_delobject(onenet->cis_context, (cis_oid_t)objectid);
        ONENET_AT_LOGI("cis_delobject end: %d", (int)cis_ret);
        if (cis_ret != CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        } else {
            onenet_at_retention_delete_object(onenet, (cis_oid_t)objectid);
        }
    }

    return cis_error;
}

static int32_t onenet_at_update(uint32_t onenet_id, uint32_t lifetime, bool withObjectFlag)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        ONENET_AT_LOGI("cis_update_reg start: 0x%x, %d, %d",
                       (unsigned int)onenet->cis_context, (int)lifetime, (int)withObjectFlag);
        cis_ret_t cis_ret = cis_update_reg(onenet->cis_context, lifetime, withObjectFlag);
        ONENET_AT_LOGI("cis_update_reg end: %d", (int)cis_ret);
        if (cis_ret != CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        } else {
            onenet->life_time = lifetime;
            onenet_at_stop_rtc_timer(onenet);
            onenet_at_start_rtc_timer(onenet);
        }
    }

    return cis_error;
}

static void onenet_at_set_cisdata(cis_data_t *cis_data, uint32_t resourceid, uint32_t valuetype, uint32_t len, const char *value)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    cis_data->id = (uint16_t)resourceid;
    cis_data->type = (cis_datatype_t)valuetype;
    switch (cis_data->type) {
        case cis_data_type_string:
            cis_data->asBuffer.length = len;
            cis_data->asBuffer.buffer = (uint8_t *)pvPortMalloc(cis_data->asBuffer.length);
            configASSERT(cis_data->asBuffer.buffer != NULL);
            memcpy(cis_data->asBuffer.buffer, value, cis_data->asBuffer.length);
            break;
        case cis_data_type_opaque:
            cis_data->asBuffer.buffer = (uint8_t *)pvPortMalloc(len + 1);
            configASSERT(cis_data->asBuffer.buffer != NULL);
            cis_data->asBuffer.length = onenet_at_hex_to_bin(cis_data->asBuffer.buffer, value, len);
            break;
        case cis_data_type_integer:
            cis_data->value.asInteger = atoi(value);
            break;
        case cis_data_type_float:
            cis_data->value.asFloat = atof(value);
            break;
        case cis_data_type_bool:
            cis_data->value.asBoolean = atoi(value);
            break;
        default:
            ONENET_AT_LOGE("not support");
            configASSERT(0);
            break;
    }
}

static void onenet_at_clear_cisdata(cis_data_t *cis_data)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    switch (cis_data->type) {
        case cis_data_type_string:
        case cis_data_type_opaque:
            vPortFree(cis_data->asBuffer.buffer);
            break;
        default:
            break;
    }
}

static int32_t onenet_at_notify(uint32_t onenet_id, uint32_t msgid, uint32_t objectid, uint32_t instanceid, uint32_t resourceid, uint32_t valuetype, uint32_t len, const char *value, uint32_t ackid, cis_coapret_t result)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;
    cis_data_t cis_data;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        cis_uri_t uri;
        uri.objectId = objectid;
        uri.instanceId = instanceid;
        uri.resourceId = resourceid;
        cis_uri_update(&uri);
        memset(&cis_data, 0, sizeof(cis_data_t));
        onenet_at_set_cisdata(&cis_data, resourceid, valuetype, len, value);
        ONENET_AT_LOGI("cis_notify start: 0x%x, 0x%x, 0x%x, %d, %d",
                       (unsigned int)onenet->cis_context, (unsigned int)&uri, (unsigned int)&cis_data, (int)msgid, (int)result);
        cis_ret_t cis_ret;
        if (ackid == (uint32_t)-1) {
            cis_ret = cis_notify(onenet->cis_context, &uri, &cis_data, (cis_mid_t)msgid, result, false);
        } else {
            cis_ret = cis_notify_ex(onenet->cis_context, &uri, &cis_data, (cis_mid_t)msgid, result, (cis_mid_t)ackid);
        }
        ONENET_AT_LOGI("cis_notify end: %d", (int)cis_ret);
        onenet_at_clear_cisdata(&cis_data);
        if (cis_ret < CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        }
    }

    return cis_error;
}

static int32_t onenet_at_response_notify(uint32_t onenet_id, uint32_t msgid, uint32_t objectid, uint32_t instanceid, uint32_t resourceid, uint32_t valuetype, uint32_t len, const char *value, cis_coapret_t result)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;
    cis_data_t cis_data;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        cis_uri_t uri;
        uri.objectId = objectid;
        uri.instanceId = instanceid;
        uri.resourceId = resourceid;
        cis_uri_update(&uri);
        memset(&cis_data, 0, sizeof(cis_data_t));
        onenet_at_set_cisdata(&cis_data, resourceid, valuetype, len, value);
        ONENET_AT_LOGI("cis_response start: 0x%x, 0x%x, 0x%x, %d, %d",
                       (unsigned int)onenet->cis_context, (unsigned int)&uri, (unsigned int)&cis_data, (int)msgid, (int)result);
        cis_ret_t cis_ret = cis_response(onenet->cis_context, &uri, &cis_data, (cis_mid_t)msgid, result);
        ONENET_AT_LOGI("cis_response end: %d", (int)cis_ret);
        onenet_at_clear_cisdata(&cis_data);
        if (cis_ret < CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        }
    }

    return cis_error;
}

static int32_t onenet_at_response(uint32_t onenet_id, uint32_t msgid, const cis_uri_t *uri, const cis_data_t *cis_data, cis_coapret_t result)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t cis_error = ONENET_AT_ERRID_OK;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet = onenet_at_search_instance(onenet_id);
    if (onenet == NULL) {
        ONENET_AT_LOGE("<ref> not found");
        cis_error = ONENET_AT_ERRID_NOT_FOUND;
    } else {
        ONENET_AT_LOGI("cis_response start: 0x%x, 0x%x, 0x%x, %d, %d",
                       (unsigned int)onenet->cis_context, (unsigned int)uri, (unsigned int)cis_data, (int)msgid, (int)result);
        cis_ret_t cis_ret = cis_response(onenet->cis_context, uri, cis_data, (cis_mid_t)msgid, result);
        ONENET_AT_LOGI("cis_response end: %d", (int)cis_ret);
        if (cis_ret < CIS_RET_OK) {
            cis_error = ONENET_AT_ERRID_SDK_ERROR;
        }
    }

    return cis_error;
}

static void onenet_at_task_processing(void *arg)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet;
    onenet_at_object_info_t *object_info;
    uint32_t i, j;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (g_onenet_at_need_deep_sleep_handling) {
        g_onenet_at_need_deep_sleep_handling = false;
        for (i = 0; i < ONENET_AT_INSTANCE_NUM; i++) {
            onenet = &g_onenet_at_context[i];
            if (onenet->is_connected && (onenet->state == PUMP_STATE_READY || onenet->state == PUMP_STATE_CONNECTING)) {
                onenet->cis_context = NULL;
                ONENET_AT_LOGI("cis_init start: 0x%x, 0x%x, %d", (unsigned int)onenet->cis_context, (unsigned int)onenet->config_bin, (int)onenet->config_len);
                cis_ret_t cis_ret;
                if (onenet->use_default_config) {
                    cis_ret = cis_init(&onenet->cis_context, 0, NULL, NULL, NULL);
                } else {
                    cis_ret = cis_init(&onenet->cis_context, onenet->config_bin, onenet->config_len, NULL, NULL);
                }
                ONENET_AT_LOGI("cis_init end: %d", (int)cis_ret);
                configASSERT(cis_ret == CIS_RET_OK);
                ((st_context_t*)onenet->cis_context)->stateStep = PUMP_STATE_CONNECTING;
                ((st_context_t*)onenet->cis_context)->ignoreRegistration = true;
                for (j = 0; j < ONENET_AT_OBJECT_MAX_COUNT; j++) {
                    object_info = &onenet->object_info[j];
                    if (object_info->is_used) {
                        cis_inst_bitmap_t bitmap;
                        cis_res_count_t resource;
                        bitmap.instanceCount = object_info->instance_count;
                        bitmap.instanceBytes = (bitmap.instanceCount + 7) / 8;
                        bitmap.instanceBitmap = object_info->instance_bitmap;
                        resource.attrCount = object_info->attribute_count;
                        resource.actCount = object_info->action_count;
                        ONENET_AT_LOGI("cis_addobject start: 0x%x, %d, (%d-%d-0x%x), (%d-%d)",
                                       (unsigned int)onenet->cis_context, (int)object_info->object_id,
                                       (int)bitmap.instanceCount, (int)bitmap.instanceBytes, (unsigned int)bitmap.instanceBitmap,
                                       (int)resource.attrCount, (int)resource.actCount);
                        cis_ret = cis_addobject(onenet->cis_context, object_info->object_id, &bitmap, &resource);
                        ONENET_AT_LOGI("cis_addobject end: %d", (int)cis_ret);
                        configASSERT(cis_ret == CIS_RET_OK);
                    }
                }
                int32_t cis_error = onenet_at_open(onenet->onenet_id, onenet->life_time);
                configASSERT(cis_error == ONENET_AT_ERRID_OK);
                observe_read_retention_data((st_context_t*)onenet->cis_context);
            }
        }
    }

    while (g_onenet_at_task_running) {
        for (i = 0; i < ONENET_AT_INSTANCE_NUM; i++) {
            onenet = &g_onenet_at_context[i];
            if (onenet->is_used) {
                cis_pump(onenet->cis_context);
                onenet->state = ((st_context_t*)onenet->cis_context)->stateStep;
            }
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

static void onenet_at_timeout_callback(TimerHandle_t xTimer)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (g_onenet_at_is_locking) {
        hal_sleep_manager_release_sleeplock(g_onenet_at_sleep_handle, HAL_SLEEP_LOCK_DEEP);
        ONENET_AT_LOGI("hal_sleep_manager_release_sleeplock");
        g_onenet_at_is_locking = false;
    }
}

void onenet_at_init(void)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    onenet_at_context_t *onenet;
    uint32_t i;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    g_onenet_at_ping_timer = xTimerCreate("onenet_at_timer",
                                      1000 * COAP_MAX_TRANSMIT_WAIT / portTICK_PERIOD_MS, 
                                      pdFALSE,
                                      (void *)0,
                                      onenet_at_timeout_callback);
    g_onenet_at_sleep_handle = hal_sleep_manager_set_sleep_handle("onenet_at");
    if (rtc_power_on_result_external() != DEEP_SLEEP && rtc_power_on_result_external() != DEEPER_SLEEP) {
        /* COLD-BOOT case: normal init */
    } else {
        /* DEEP-SLEEP case: data retention process */
        ONENET_AT_LOGI("deep sleep handling");
        for (i = 0; i < ONENET_AT_INSTANCE_NUM; i++) {
            onenet = &g_onenet_at_context[i];
            ONENET_AT_LOGI("is_connected: %d, state: %d", onenet->is_connected, onenet->state);
            if (onenet->is_connected && (onenet->state == PUMP_STATE_READY || onenet->state == PUMP_STATE_CONNECTING)) {
                g_onenet_at_need_deep_sleep_handling = true;
                if (g_onenet_at_task_running == false) {
                    g_onenet_at_task_running = true;
                    xTaskCreate(onenet_at_task_processing,
                                 ONENET_AT_TASK_NAME,
                                 ONENET_AT_TASK_STACK_SIZE / sizeof(portSTACK_TYPE),
                                 NULL,
                                 ONENET_AT_TASK_PRIORITY,
                                 NULL);
                    ONENET_AT_LOGI("create onenet task");
                }
            }
        }
    }
}

bool onenet_at_convert_result(uint32_t cmdid, onenet_at_result_t result, cis_coapret_t *coapret)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    if (coapret == NULL) {
        return false;
    }

    switch (result) {
        case ONENET_AT_RESULT_205_CONTENT:
            if (cmdid == ONENET_AT_CMDID_READ_RESPONSE || cmdid == ONENET_AT_CMDID_OBSERVE_RESPONSE || cmdid == ONENET_AT_CMDID_DISCOVER_RESPONSE) {
                *coapret = CIS_COAP_205_CONTENT;
                return true;
            }
            break;

        case ONENET_AT_RESULT_204_CHANGED:
            if (cmdid == ONENET_AT_CMDID_WRITE_RESPONSE || cmdid == ONENET_AT_CMDID_EXECUTE_RESPONSE || cmdid == ONENET_AT_CMDID_PARAMETER_RESPONSE) {
                *coapret = CIS_COAP_204_CHANGED;
                return true;
            }
            break;

        case ONENET_AT_RESULT_400_BAD_REQUEST:
            *coapret = CIS_COAP_400_BAD_REQUEST;
            return true;

        case ONENET_AT_RESULT_401_UNAUTHORIZED:
            *coapret = CIS_COAP_401_UNAUTHORIZED;
            return true;

        case ONENET_AT_RESULT_404_NOT_FOUND:
            *coapret = CIS_COAP_404_NOT_FOUND;
            return true;

        case ONENET_AT_RESULT_405_METHOD_NOT_ALLOWED:
            *coapret = CIS_COAP_405_METHOD_NOT_ALLOWED;
            return true;

        case ONENET_AT_RESULT_406_NOT_ACCEPTABLE:
            if (cmdid == ONENET_AT_CMDID_READ_RESPONSE || cmdid == ONENET_AT_CMDID_OBSERVE_RESPONSE || cmdid == ONENET_AT_CMDID_DISCOVER_RESPONSE) {
                *coapret = CIS_COAP_406_NOT_ACCEPTABLE;
                return true;
            }
            break;

        default:
            return false;
    }

    return false;
}
bool error_oennet = FALSE;
apb_proxy_status_t apb_proxy_hdlr_onenet_create_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer = NULL;
    int32_t cis_error;
    uint32_t ref;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    debug_print("%s\n",parse_cmd->string_ptr);
    debug_print("%d\n",parse_cmd->string_len);
    debug_print("%d\n",parse_cmd->cmd_id);
    debug_print("%d\n",parse_cmd->mode);
    debug_print("%d\n",parse_cmd->name_len);
    debug_print("%d\n",parse_cmd->parse_pos);
    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);
   
    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLCREATE=<totalsize>, <config>, <index>, <currentsize>, <flag>
                    if (param_num < 5) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t totalsize = 2 * atoi(param_list[0]);
                        char *config = param_list[1];
                        int32_t index = atoi(param_list[2]);
                        uint32_t currentsize = 2 * atoi(param_list[3]);
                        if (totalsize < currentsize) {
                            ONENET_AT_LOGE("<totalsize> is smaller than <currentsize>");
                            cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                        } else if (totalsize == currentsize) {
                            // <totalsize> equals <currentsize>
                            cis_error = onenet_at_create(currentsize, config, &ref);
                        } else {
                            // <totalsize> is larger than <currentsize>
                            static uint32_t currentsize_last = 0;
                            static char *config_last = NULL;
                            if (index > 0 && config_last == NULL) {
                                currentsize_last = currentsize;
                                config_last = (char *)pvPortMalloc(totalsize + 1);
                                if (config_last == NULL) {
                                    ONENET_AT_LOGE("memory error");
                                    cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
                                } else {
                                    strcpy(config_last, config);
                                    cis_error = ONENET_AT_ERRID_CONTINUE;
                                    goto create_exit;
                                }
                            } else if (index > 0 && config_last != NULL && currentsize_last + currentsize <= totalsize) {
                                currentsize_last += currentsize;
                                strcat(config_last, config);
                                cis_error = ONENET_AT_ERRID_CONTINUE;
                                goto create_exit;
                            } else if (index == 0 && config_last != NULL && currentsize_last + currentsize <= totalsize) {
                                currentsize_last += currentsize;
                                strcat(config_last, config);
                                cis_error = onenet_at_create(currentsize_last, config_last, &ref);
                            } else {
                                ONENET_AT_LOGE("parameter error");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                            // free it when "OK" or "ERROR" is sent
                            currentsize_last = 0;
                            if (config_last != NULL) {
                                vPortFree(config_last);
                            }
                            config_last = NULL;
                        }
                    }
                }
            }

create_exit:
            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_CONTINUE || cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
                if (g_onenet_at_task_running == false) {
                    g_onenet_at_task_running = true;
                    xTaskCreate(onenet_at_task_processing,
                                 ONENET_AT_TASK_NAME,
                                 ONENET_AT_TASK_STACK_SIZE / sizeof(portSTACK_TYPE),
                                 NULL,
                                 ONENET_AT_TASK_PRIORITY,
                                 NULL);
                    ONENET_AT_LOGI("create onenet task");
                }
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        case APB_PROXY_CMD_MODE_ACTIVE:
            cis_error = onenet_at_create(0, NULL, &ref);
            goto create_exit;
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (cis_error == ONENET_AT_ERRID_OK) {
        sprintf(response_data, "\r\n+MIPLCREATE: %d", (int)ref);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
        error_oennet = TRUE;
    } else if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_delete_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLDELETE=<ref>
                    if (param_num < 1) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        cis_error = onenet_at_delete(ref);
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                g_onenet_at_task_running = false;
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_open_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLOPEN=<ref>, <lifetime>
                    if (param_num < 2) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t lifetime = atoi(param_list[1]);
                        if (lifetime == 0) {
                            lifetime = 3600;
                        }
                        cis_error = onenet_at_open(ref, (cis_time_t)lifetime);
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_close_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLCLOSE=<ref>
                    if (param_num < 1) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        cis_error = onenet_at_close(ref);
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_add_object_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLADDOBJ=<ref>, <objectid>, <instancecount>, <instancebitmap>, <attributecount>, <actioncount>
                    if (param_num < 6) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t objectid = atoi(param_list[1]);
                        uint32_t instancecount = atoi(param_list[2]);
                        char *instancebitmap = param_list[3];
                        uint32_t attributecount = atoi(param_list[4]);
                        uint32_t actioncount = atoi(param_list[5]);
                        cis_error = onenet_at_add_object(ref, objectid, instancecount, instancebitmap, attributecount, actioncount);
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_delete_object_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLDELOBJ=<ref>, <objectid>
                    if (param_num < 2) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t objectid = atoi(param_list[1]);
                        cis_error = onenet_at_delete_object(ref, objectid);
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_update_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLUPDATE=<ref>, <lifetime>, <withObjectFlag>
                    if (param_num < 3) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t lifetime = atoi(param_list[1]);
                        uint32_t withObjectFlag = atoi(param_list[2]);
                        if (withObjectFlag == 0 || withObjectFlag == 1) {
                            if (lifetime == 0) {
                                lifetime = 3600;
                            }
                            cis_error = onenet_at_update(ref, lifetime, withObjectFlag);
                        } else {
                            ONENET_AT_LOGE("parameter error");
                            cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_notify_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLNOTIFY=<ref>, <msgid>, <objectid>, <instanceid>, <resourceid>, <valuetype>, <len>, <value>, <index>, <flag>[, <ackid>]
                    if (param_num < 10) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t objectid = atoi(param_list[2]);
                        uint32_t instanceid = atoi(param_list[3]);
                        uint32_t resourceid = atoi(param_list[4]);
                        uint32_t valuetype = atoi(param_list[5]);
                        uint32_t len = atoi(param_list[6]);
                        char *value = param_list[7];
                        int32_t index = atoi(param_list[8]);
                        uint32_t ackid = (uint32_t)-1;
                        if (param_num > 10) {
                            ackid = atoi(param_list[10]);
                        }
                        if (index > 0) {
                            cis_error = onenet_at_notify(ref, msgid, objectid, instanceid, resourceid, valuetype, len, value, ackid, CIS_NOTIFY_CONTINUE);
                        } else if (index == 0) {
                            cis_error = onenet_at_notify(ref, msgid, objectid, instanceid, resourceid, valuetype, len, value, ackid, CIS_NOTIFY_CONTENT);
                        } else {
                            ONENET_AT_LOGE("parameter error");
                            cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_read_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    cis_coapret_t coapret;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLREADRSP=<ref>, <msgid>, <result>[, <objectid>, <instanceid>, <resourceid>, <valuetype>, <len>, <value>, <index>, <flag>]
                    if (param_num < 3) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t result = atoi(param_list[2]);
                        if (result == ONENET_AT_RESULT_205_CONTENT) {
                            if (param_num < 11) {
                                ONENET_AT_LOGE("parameter too short");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            } else {
                                uint32_t objectid = atoi(param_list[3]);
                                uint32_t instanceid = atoi(param_list[4]);
                                uint32_t resourceid = atoi(param_list[5]);
                                uint32_t valuetype = atoi(param_list[6]);
                                uint32_t len = atoi(param_list[7]);
                                char *value = param_list[8];
                                int32_t index = atoi(param_list[9]);
                                if (index > 0) {
                                    cis_error = onenet_at_response_notify(ref, msgid, objectid, instanceid, resourceid, valuetype, len, value, CIS_NOTIFY_CONTINUE);
                                } else if (index == 0) {
                                    cis_error = onenet_at_response_notify(ref, msgid, objectid, instanceid, resourceid, valuetype, len, value, CIS_RESPONSE_READ);
                                } else {
                                    ONENET_AT_LOGE("parameter error");
                                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                                }
                            }
                        } else {
                            if (onenet_at_convert_result(ONENET_AT_CMDID_READ_RESPONSE, (onenet_at_result_t)result, &coapret) == true) {
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, coapret);
                            } else {
                                ONENET_AT_LOGE("<result> out of range");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_write_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    cis_coapret_t coapret;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLWRITERSP=<ref>, <msgid>, <result>
                    if (param_num < 3) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t result = atoi(param_list[2]);
                        if (result == ONENET_AT_RESULT_204_CHANGED) {
                            cis_error = onenet_at_response(ref, msgid, NULL, NULL, CIS_RESPONSE_WRITE);
                        } else {
                            if (onenet_at_convert_result(ONENET_AT_CMDID_WRITE_RESPONSE, (onenet_at_result_t)result, &coapret) == true) {
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, coapret);
                            } else {
                                ONENET_AT_LOGE("<result> out of range");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_execute_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    cis_coapret_t coapret;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLEXECUTERSP=<ref>, <msgid>, <result>
                    if (param_num < 3) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t result = atoi(param_list[2]);
                        if (result == ONENET_AT_RESULT_204_CHANGED) {
                            cis_error = onenet_at_response(ref, msgid, NULL, NULL, CIS_RESPONSE_EXECUTE);
                        } else {
                            if (onenet_at_convert_result(ONENET_AT_CMDID_EXECUTE_RESPONSE, (onenet_at_result_t)result, &coapret) == true) {
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, coapret);
                            } else {
                                ONENET_AT_LOGE("<result> out of range");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_observe_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    cis_coapret_t coapret;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLOBSERVERSP=<ref>, <msgid>, <result>
                    if (param_num < 3) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t result = atoi(param_list[2]);
                        if (result == ONENET_AT_RESULT_205_CONTENT) {
                            cis_error = onenet_at_response(ref, msgid, NULL, NULL, CIS_RESPONSE_OBSERVE);
                        } else {
                            if (onenet_at_convert_result(ONENET_AT_CMDID_OBSERVE_RESPONSE, (onenet_at_result_t)result, &coapret) == true) {
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, coapret);
                            } else {
                                ONENET_AT_LOGE("<result> out of range");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_discover_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    cis_coapret_t coapret;
    debug_print("%s\n", parse_cmd->string_ptr);
    debug_print("%d\n", parse_cmd->mode);
    debug_print("%d\n", parse_cmd->string_len);
    debug_print("%d\n", parse_cmd->cmd_id);
    debug_print("%d\n", parse_cmd->parse_pos);

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/

    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLDISCOVERRSP=<ref>, <msgid>, <result>, <length>, <valuestring>
                    if (param_num < 4) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t result = atoi(param_list[2]);
                        uint32_t length = atoi(param_list[3]);
                        char *valuestring = param_list[4];
                        if (length == 0 || length != strlen(valuestring)) {
                            ONENET_AT_LOGE("parameter error");
                            cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                        } else if (result == ONENET_AT_RESULT_205_CONTENT) {
                            uint32_t i;
                            cis_uri_t uri;
                            char *field_list[ONENET_AT_CMD_FILED_NUM + 1];
                            uint32_t filed_num = onenet_at_parse_param(valuestring, param_buffer, NULL, field_list, ONENET_AT_CMD_FILED_NUM + 1, ";");
                            if (filed_num > ONENET_AT_CMD_FILED_NUM) {
                                ONENET_AT_LOGE("parameter error");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            } else {
                                for (i = 0; i < filed_num; i++) {
                                    uri.objectId = URI_INVALID;
                                    uri.instanceId = URI_INVALID;
                                    uri.resourceId = (uint16_t)atoi(field_list[i]);
                                    cis_uri_update(&uri);
                                    cis_error = onenet_at_response(ref, msgid, &uri, NULL, CIS_RESPONSE_CONTINUE); 
                                }
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, CIS_RESPONSE_DISCOVER);
                            }
                        } else {
                            if (onenet_at_convert_result(ONENET_AT_CMDID_DISCOVER_RESPONSE, (onenet_at_result_t)result, &coapret) == true) {
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, coapret);
                            } else {
                                ONENET_AT_LOGE("<result> out of range");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_parameter_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    char *param_buffer;
    int32_t cis_error;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];
    cis_coapret_t coapret;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_EXECUTION:
            param_buffer = (char *)pvPortMalloc(parse_cmd->string_len);
            if (param_buffer == NULL) {
                ONENET_AT_LOGE("memory error");
                cis_error = ONENET_AT_ERRID_MEMORY_ERROR;
            } else {
                char *cmd_string = strchr(parse_cmd->string_ptr, '=');
                if (!cmd_string || *(cmd_string + 1) == '\0') {
                    ONENET_AT_LOGE("no parameter");
                    cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                } else {
                    char *param_list[ONENET_AT_CMD_PARAM_NUM];
                    uint32_t param_num = onenet_at_parse_cmd(++cmd_string, param_buffer, param_list, ONENET_AT_CMD_PARAM_NUM);

                    // AT+MIPLPAMAMETERRSP=<ref>, <msgid>, <result>
                    if (param_num < 3) {
                        ONENET_AT_LOGE("parameter too short");
                        cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                    } else {
                        uint32_t ref = atoi(param_list[0]);
                        uint32_t msgid = atoi(param_list[1]);
                        uint32_t result = atoi(param_list[2]);
                        if (result == ONENET_AT_RESULT_204_CHANGED) {
                            cis_error = onenet_at_response(ref, msgid, NULL, NULL, CIS_RESPONSE_OBSERVE_PARAMS);
                        } else {
                            if (onenet_at_convert_result(ONENET_AT_CMDID_PARAMETER_RESPONSE, (onenet_at_result_t)result, &coapret) == true) {
                                cis_error = onenet_at_response(ref, msgid, NULL, NULL, coapret);
                            } else {
                                ONENET_AT_LOGE("<result> out of range");
                                cis_error = ONENET_AT_ERRID_PARAMETER_ERROR;
                            }
                        }
                    }
                }
            }

            if (param_buffer != NULL) {
                vPortFree(param_buffer);
            }

            if (cis_error == ONENET_AT_ERRID_OK) {
                response.result_code = APB_PROXY_RESULT_OK;
            } else {
                response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            }
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_version_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};
    int32_t cis_error;
    cis_version_t version;
    char response_data[ONENET_AT_RESPONSE_DATA_LEN];

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    ONENET_AT_LOGI("%s", parse_cmd->string_ptr);

    switch (parse_cmd->mode) {
        case APB_PROXY_CMD_MODE_READ:
            // AT+MIPLVER?
            cis_version(&version);
            cis_error = ONENET_AT_ERRID_OK;
            response.result_code = APB_PROXY_RESULT_OK;
            break;

        default:
            ONENET_AT_LOGE("not support");
            cis_error = ONENET_AT_ERRID_NOT_SUPPORT;
            response.result_code = APB_PROXY_RESULT_CUSTOM_ERROR;
            break;
    }

    if (cis_error == ONENET_AT_ERRID_OK) {
        sprintf(response_data, "\r\n+MIPLVER: %d.%d.%d", (int)version.major, (int)version.minor, (int)version.micro);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    } else if (response.result_code == APB_PROXY_RESULT_CUSTOM_ERROR) {
        sprintf(response_data, "+CIS ERROR: %d", (int)cis_error);
        response.pdata = response_data;
        response.length = strlen(response.pdata);
    }

    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

#else /* MTK_ONENET_SUPPORT */

void onenet_at_init(void)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
}

apb_proxy_status_t apb_proxy_hdlr_onenet_create_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_delete_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_open_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_close_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_add_object_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_delete_object_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_update_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_notify_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_read_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_write_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_execute_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_observe_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_discover_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_parameter_response_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

apb_proxy_status_t apb_proxy_hdlr_onenet_version_cmd(apb_proxy_parse_cmd_param_t *parse_cmd)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    apb_proxy_at_cmd_result_t response = {0};

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    response.result_code = APB_PROXY_RESULT_ERROR;
    response.cmd_id = parse_cmd->cmd_id;
    apb_proxy_send_at_cmd_result(&response);

    return APB_PROXY_STATUS_OK;
}

#endif /* MTK_ONENET_SUPPORT */

uint32_t onenet_at_hex_to_bin(uint8_t *dest, const char *source, uint32_t max_dest_len)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    uint32_t i = 0;
    uint8_t lower_byte, upper_byte;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    while (source[i] != '\0') {
        if (source[i] >= '0' && source[i] <= '9') {
            lower_byte = source[i] - '0';
        } else if (source[i] >= 'a' && source[i] <= 'f') {
            lower_byte = source[i] - 'a' + 10;
        } else if (source[i] >= 'A' && source[i] <= 'F') {
            lower_byte = source[i] - 'A' + 10;
        } else {
            return 0;
        }

        if (source[i + 1] >= '0' && source[i + 1] <= '9') {
            upper_byte = source[i + 1] - '0';
        } else if (source[i + 1] >= 'a' && source[i + 1] <= 'f') {
            upper_byte = source[i + 1] - 'a' + 10;
        } else if (source[i + 1] >= 'A' && source[i + 1] <= 'F') {
            upper_byte = source[i + 1] - 'A' + 10;
        } else {
            return 0;
        }

        if ((i >> 1) >= max_dest_len) {
            return (i >> 1);
        }

        *(dest + (i >> 1)) = (lower_byte << 4) + upper_byte;
        i += 2;
    }

    return (i >> 1);
}

uint32_t onenet_at_bin_to_hex(char *dest, const uint8_t *source, uint32_t max_dest_len)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    uint32_t i = 0, j = 0;
    uint8_t ch1, ch2;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    while (j + 1 < max_dest_len)
    {
        ch1 = (source[i] & 0xF0) >> 4;
        ch2 = source[i] & 0x0F;

        if (ch1 <= 9) {
            *(dest + j) = ch1 + '0';
        } else {
            *(dest + j) = ch1 + 'A' - 10;
        }

        if (ch2 <= 9) {
            *(dest + j + 1) = ch2 + '0';
        } else {
            *(dest + j + 1) = ch2 + 'A' - 10;
        }

        i++;
        j += 2;
    }

    *(dest + j) = '\0';
    return j;
}

uint32_t onenet_at_parse_cmd(const char *cmd_string, const char *param_buffer, char *param_list[], uint32_t param_max_num)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t i;
    uint32_t param_num = 0;
    char *p = (char *)cmd_string, *p2, *p3;
    char *param = (char *)param_buffer;
    uint32_t param_len, len;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    p = strtok(p, "\r\n");
    while (p != NULL && *p != '\0' && param_num < param_max_num) {
        // remove leading blanks & tabs
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        // remove double quotation
        if (*p == '\"') {
            p2 = strchr(++p, '"');
            if (p2 == NULL) {
                break;
            }
            len = p2 - p;
            strncpy(param, p, len);
            *(param + len) = '\0';
            p = p2;
            strsep(&p, ",");
        } else {
            p3 = strsep(&p, ",");
            if (!p3)
            {
                break;
            }
            strcpy(param, p3);
            // remove post blanks & tabs
            param_len = strlen(param);
            for (i = param_len - 1; i >= 0; i--) {
                if (*(param + i) == ' ' || *(param + i) == '\t') {
                    *(param + i) = '\0';
                } else {
                    break;
                }
            }
        }
        // add to param_list
        param_list[param_num++] = param;

        param = param + strlen(param) + 1;
    }

    for (i = 0; i < param_num; i++) {
        ONENET_AT_LOGI("param_list[%d]: %s", (int)i, param_list[i]);
    }

    return param_num;
}

uint32_t onenet_at_parse_param(const char *param_string, const char *field_buffer, const char *field_key[], char *field_list[], uint32_t field_max_num, const char *delim)
{
    /*----------------------------------------------------------------*/
    /* Local Variables                                                */
    /*----------------------------------------------------------------*/
    int32_t i;
    uint32_t field_num = 0;
    char *p = strtok((char *)param_string, delim);
    char *field = (char *)field_buffer;
    uint32_t field_len;
    char *field_value = NULL;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    while (p != NULL && *p != '\0' && field_num < field_max_num) {
        strcpy(field, p);
        // remove leading blanks
        while (*field == 0x20) {
            field++;
        }
        // remove post blanks
        field_len = strlen(field);
        for (i = field_len - 1; i >= 0; i--) {
            if (*(field + i) == 0x20) {
                *(field + i) = '\0';
            } else {
                break;
            }
        }
        if (field_key != NULL) {
            // search '='
            field_value = strchr(field, '=');
            if (field_value == NULL || strnicmp(field, field_key[field_num], strlen(field_key[field_num])) != 0) {
                break;
            }
            field_value++;
            // remove leading blanks
            while (*field_value == 0x20) {
                field_value++;
            }
        } else {
            field_value = field;
        }
        // add to field_list
        field_list[field_num++] = field_value;

        field = field + strlen(field) + 1;
        p = strtok(NULL, delim);
    }

    for (i = 0; i < field_num; i++) {
        ONENET_AT_LOGI("field_list[%d]: %s", (int)i, field_list[i]);
    }

    return field_num;
}


