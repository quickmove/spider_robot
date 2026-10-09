#include <stdlib.h>

#include "spider_kf_parse.h"
#include "spider_web.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "spider_motion.h"
#include "spider_params.h"
#include "spider_servo.h"
#include "web_priv.h"

static const char *TAG = "web";

/* target_add_binary_data 生成的符号（MAKE_C_IDENTIFIER 作用于文件名） */
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[]   asm("_binary_index_html_gz_end");

#define AP_SSID_PREFIX "SpiderRobot-"
#define AP_CHANNEL     1
#define AP_MAX_CONN    4

#define TELEMETRY_PERIOD_MS 100   /* 10Hz，与 docs/protocol.md §2.3 一致 */
#define WS_MAX_MSG         256

/*
 * WS 客户端句柄。
 *
 * IDF 5.4 的 httpd_req_t 不含 fd，也没有 httpd_ws_get_fd_socket，
 * 因此连接时保存 req 指针 + httpd_req_to_sockfd() 得到的 fd。
 * 二者都在该 socket 存活期间有效；关闭时清空。
 */
static httpd_handle_t s_httpd;
static httpd_req_t  *s_ws_req = NULL;
static volatile int  s_ws_fd = -1;
static volatile bool s_client_connected;
static char             s_ssid[24];

int web_ws_fd(void)
{
    return s_ws_fd;
}

esp_err_t web_send_text(const char *payload, size_t len)
{
    if (s_ws_fd < 0 || s_ws_req == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * 异步发送：遥测与 ack 都发生在 httpd 服务任务之外，
     * 阻塞版 httpd_ws_send_frame() 会在 httpd 的锁上互相等待。
     */
    httpd_ws_frame_t f = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)payload,
        .len = len,
    };
    return httpd_ws_send_frame_async(s_httpd, s_ws_fd, &f);
}

/* ------------------------------------------------------------------ WiFi */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP 已启动: %s", s_ssid);
        /* 默认 AP 网段固定为 192.168.4.1，直接打印省一次取值 */
        ESP_LOGI(TAG, "打开浏览器访问  http://192.168.4.1");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *ev = data;
        ESP_LOGI(TAG, "客户端接入: %02x:%02x:%02x:%02x:%02x:%02x",
                 ev->mac[0], ev->mac[1], ev->mac[2],
                 ev->mac[3], ev->mac[4], ev->mac[5]);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGW(TAG, "客户端断开");
        /* 断连后主动清零运动意图：不等心跳超时，机器人立刻站住 */
        motion_drive(0.0f, 0.0f, 0.0f);
        motion_stop();
    }
}

static esp_err_t wifi_ap_start(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ssid, sizeof(s_ssid), AP_SSID_PREFIX "%02X%02X",
             mac[4], mac[5]);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), "web", "esp_wifi_init 失败");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL),
        "web", "注册 WIFI_EVENT 失败");

    wifi_config_t wc = {0};
    memcpy(wc.ap.ssid, s_ssid, sizeof(wc.ap.ssid));   /* char[32]，非指针 */
    wc.ap.ssid_len = (uint8_t)strlen(s_ssid);
    wc.ap.channel = AP_CHANNEL;
    wc.ap.max_connection = AP_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_OPEN;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), "web", "设 AP 模式失败");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wc), "web", "AP 配置失败");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), "web", "esp_wifi_start 失败");

    ESP_LOGI(TAG, "AP 配置完成 ssid=%s channel=%d", s_ssid, AP_CHANNEL);
    return ESP_OK;
}

/* ------------------------------------------------------------------ HTTP */

static esp_err_t root_handler(httpd_req_t *req)
{
    const size_t len = (size_t)(index_html_gz_end - index_html_gz_start);
    /* 内容是 gzip 后的原始字节，须声明编码，否则浏览器会当乱码 */
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    ESP_LOGI(TAG, "下发控制页 %u 字节(gzip)", (unsigned)len);
    return httpd_resp_send(req, (const char *)index_html_gz_start, len);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[512];
    web_build_status(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t params_handler(httpd_req_t *req)
{
    char buf[640];
    int n = snprintf(buf, sizeof(buf),
        "{\"geom\":{\"L1\":%.1f,\"L2\":%.1f,\"alpha_deg\":%.1f,"
        "\"beta_cruise_deg\":%.1f},"
        "\"crawl\":{\"stride_mm\":%.1f,\"period_s\":%.2f,\"lift_mm\":%.1f},"
        "\"trot\":{\"stride_mm\":%.1f,\"period_s\":%.2f,\"lift_mm\":%.1f},"
        "\"limits\":{\"hip_deg\":%.0f,\"beta_min\":%.1f,\"beta_max\":%.1f,"
        "\"max_rate_deg_s\":%.0f},"
        "\"safety\":{\"heartbeat_ms\":%d,\"estop_ms\":%d},"
        "\"legs\":[",
        SPIDER_L1_MM, SPIDER_L2_MM, SPIDER_ALPHA_DEG, SPIDER_BETA_CRUISE_DEG,
        SPIDER_CRAWL_STRIDE_MM, SPIDER_CRAWL_PERIOD_S, SPIDER_CRAWL_LIFT_MM,
        SPIDER_TROT_STRIDE_MM, SPIDER_TROT_PERIOD_S, SPIDER_TROT_LIFT_MM,
        SPIDER_HIP_LIMIT_DEG, SPIDER_BETA_MIN_DEG, SPIDER_BETA_MAX_DEG,
        SPIDER_MAX_RATE_DEG_S, SPIDER_HEARTBEAT_TIMEOUT_MS,
        SPIDER_ESTOP_RESPONSE_MS);
    for (int i = 0; i < SPIDER_LEG_COUNT; i++) {
        const spider_leg_t *L = &SPIDER_LEGS[i];
        n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                      "%s{\"id\":\"%s\",\"psi_deg\":%.0f,\"r_mm\":%.1f,"
                      "\"ch_hip\":%u,\"ch_knee\":%u}",
                      (i ? "," : ""), L->id, L->psi_deg, L->r_mm,
                      L->ch_hip, L->ch_knee);
    }
    n += snprintf(buf + n, sizeof(buf) - (size_t)n, "]}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t cal_get_handler(httpd_req_t *req)
{
    char buf[768];
    int n = snprintf(buf, sizeof(buf), "{\"calibrated\":%s,\"ch\":[",
                     servo_cal_is_ready() ? "true" : "false");
    for (uint8_t ch = 0; ch < SPIDER_SERVO_COUNT; ch++) {
        servo_cal_t c;
        servo_cal_get(ch, &c);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                      "%s{\"ch\":%u,\"min\":%u,\"max\":%u,\"neutral\":%u,"
                      "\"trim\":%d,\"reverse\":%s}",
                      (ch ? "," : ""), ch, c.min_us, c.max_us,
                      c.neutral_us, (int)c.trim_us,
                      c.reverse ? "true" : "false");
    }
    snprintf(buf + n, sizeof(buf) - (size_t)n, "]}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t cal_post_handler(httpd_req_t *req)
{
    char body[WS_MAX_MSG * 4];
    int total = 0;
    while (total < (int)sizeof(body) - 1) {
        int got = httpd_req_recv(req, body + total, sizeof(body) - 1 - (size_t)total);
        if (got <= 0) {
            break;
        }
        total += got;
    }
    body[total] = '\0';

    /* 支持单条 {"ch":0,...} 或整表 {"cal":[{...},{...}]} */
    cJSON *root = cJSON_Parse(body);
    if (root == NULL) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad_json\"}");
    }

    cJSON *arr = cJSON_GetObjectItem(root, "cal");
    cJSON *src = arr ? arr : root;

    int applied = 0, rejected = 0;
    cJSON *it = cJSON_IsArray(src) ? src->child : src;
    while (it != NULL) {
        const cJSON *jc = cJSON_GetObjectItem(it, "ch");
        if (jc == NULL) {
            it = it->next;
            continue;
        }
        int ch = jc->valueint;
        cJSON *jmin = cJSON_GetObjectItem(it, "min");
        cJSON *jmax = cJSON_GetObjectItem(it, "max");
        cJSON *jneu = cJSON_GetObjectItem(it, "neutral");
        cJSON *jtri = cJSON_GetObjectItem(it, "trim");
        cJSON *jrev = cJSON_GetObjectItem(it, "reverse");

        if (ch >= 0 && ch < (int)SPIDER_SERVO_COUNT
            && jmin && jmax && jneu) {
            servo_cal_t c = {
                .min_us = (uint16_t)jmin->valueint,
                .max_us = (uint16_t)jmax->valueint,
                .neutral_us = (uint16_t)jneu->valueint,
                .trim_us = (int16_t)(jtri ? jtri->valueint : 0),
                .reverse = cJSON_IsTrue(jrev),
            };
            /* 标定模式下先切进标定，避免保存瞬间机器人在动 */
            if (servo_cal_set((uint8_t)ch, &c) == ESP_OK) {
                applied++;
            } else {
                rejected++;
            }
        }
        it = it->next;
    }
    cJSON_Delete(root);

    esp_err_t save_err = servo_cal_save();
    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"ok\":%s,\"applied\":%d,\"rejected\":%d,\"error\":%s%s}",
             (rejected == 0 && save_err == ESP_OK) ? "true" : "false",
             applied, rejected,
             save_err == ESP_OK ? "null" : "\"save_failed\"",
             (rejected > 0) ? "" : "");

    httpd_resp_set_type(req, "application/json");
    if (rejected > 0) {
        httpd_resp_set_status(req, "400 Bad Request");
    }
    return httpd_resp_sendstr(req, resp);
}

/* ------------------------------------------------------------------ WebSocket */

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        s_ws_req = req;
        s_ws_fd = httpd_req_to_sockfd(req);
        s_client_connected = true;
        ESP_LOGI(TAG, "WebSocket 已连接");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {0};
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGW(TAG, "WebSocket 关闭");
        s_client_connected = false;
        s_ws_fd = -1;
        s_ws_req = NULL;
        motion_drive(0.0f, 0.0f, 0.0f);
        motion_stop();
        return ESP_OK;
    }
    if (ws_pkt.type != HTTPD_WS_TYPE_TEXT || ws_pkt.len == 0) {
        return ESP_OK;
    }
    /*
     * 接收缓冲必须放堆。
     *
     * keyframe_upload 报文可达 4KB（见 spider_kf_parse.h 的
     * KF_MAX_UPLOAD_BYTES），而 httpd 任务栈只有 8KB（cfg.stack_size），
     * 其中还要跑 cJSON_ParseWithLength——放栈上会直接爆栈。
     * WS_MAX_MSG 因此只保留给发送侧的 ack/遥测组包。
     */
    if (ws_pkt.len == 0 || ws_pkt.len > KF_MAX_UPLOAD_BYTES) {
        web_send_text("{\"t\":\"err\",\"msg\":\"too_long\"}",
                      sizeof("{\"t\":\"err\",\"msg\":\"too_long\"}") - 1);
        return ESP_OK;
    }

    size_t rx_len = ws_pkt.len + 1;
    char *msg = (char *)malloc(rx_len);
    if (msg == NULL) {
        web_send_text("{\"t\":\"err\",\"msg\":\"no_mem\"}",
                      sizeof("{\"t\":\"err\",\"msg\":\"no_mem\"}") - 1);
        return ESP_OK;
    }
    memset(msg, 0, rx_len);
    httpd_ws_recv_frame(req, &ws_pkt, rx_len);

    /* 任何有效指令都刷新心跳（安全不变量 3 的前提） */
    motion_heartbeat((uint32_t)(esp_timer_get_time() / 1000));

    esp_err_t rc = web_ws_handle_cmd(msg, ws_pkt.len);
    free(msg);
    return rc;
}

/* ------------------------------------------------------------------ 遥测 */

void web_build_status(char *buf, size_t buf_len)
{
    motion_state_t st;
    motion_get_state(&st);

    int n = snprintf(buf, buf_len,
        "{\"t\":\"st\",\"mode\":\"%s\",\"gait\":\"%s\","
        "\"speed\":%.0f,\"heading\":%.1f,\"height\":%.1f,"
        "\"margin\":%.0f,\"estop\":%s,\"calibrated\":%s,"
        "\"cal_mode\":%s,"
        "\"legs\":[",
        motion_mode_name(st.mode),
        (st.gait == GAIT_TROT) ? "TROT" : "CRAWL",
        st.speed_pct, st.heading_deg, st.height_mm,
        st.support_margin_mm,
        st.estop ? "true" : "false",
        servo_cal_is_ready() ? "true" : "false",
        (servo_get_mode() == SERVO_MODE_CALIBRATE) ? "true" : "false");

    for (uint8_t leg = 0; leg < SERVO_LEGS; leg++) {
        n += snprintf(buf + n, buf_len - (size_t)n, "%s[%.1f,%.1f]",
                      leg ? "," : "",
                      servo_get_current(leg, SERVO_JOINT_HIP),
                      servo_get_current(leg, SERVO_JOINT_KNEE));
    }
    snprintf(buf + n, buf_len - (size_t)n, "],\"temp\":%d}",
             (int)(esp_get_free_heap_size() / 1024));
}

static void telemetry_task(void *arg)
{
    (void)arg;
    char buf[640];
    for (;;) {
        if (s_client_connected && s_ws_fd >= 0) {
            web_build_status(buf, sizeof(buf));
            if (web_send_text(buf, strlen(buf)) != ESP_OK) {
                ESP_LOGW(TAG, "遥测发送失败，标记为断开");
                s_client_connected = false;
                s_ws_fd = -1;
                s_ws_req = NULL;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------ 启动 */

esp_err_t web_init(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), "web", "esp_netif_init 失败");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), "web",
                        "创建默认事件循环失败");
    ESP_RETURN_ON_ERROR(wifi_ap_start(), "web", "AP 启动失败");

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;        /* JSON 解析 + 遥测，需要余量 */
    cfg.max_open_sockets = AP_MAX_CONN;
    cfg.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(&s_httpd, &cfg), "web",
                        "httpd_start 失败");

    static const httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET, .handler = root_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t status = {
        .uri = "/api/status", .method = HTTP_GET, .handler = status_handler,
    };
    static const httpd_uri_t params = {
        .uri = "/api/params", .method = HTTP_GET, .handler = params_handler,
    };
    static const httpd_uri_t cal_get = {
        .uri = "/api/cal", .method = HTTP_GET, .handler = cal_get_handler,
    };
    static const httpd_uri_t cal_post = {
        .uri = "/api/cal", .method = HTTP_POST, .handler = cal_post_handler,
    };
    static const httpd_uri_t ws = {
        .uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
        .is_websocket = true,
    };

    httpd_register_uri_handler(s_httpd, &root);
    httpd_register_uri_handler(s_httpd, &status);
    httpd_register_uri_handler(s_httpd, &params);
    httpd_register_uri_handler(s_httpd, &cal_get);
    httpd_register_uri_handler(s_httpd, &cal_post);
    httpd_register_uri_handler(s_httpd, &ws);

    if (xTaskCreate(telemetry_task, "telemetry", 4096, NULL,
                    2, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HTTP/WS 就绪 ssid=%s", s_ssid);
    return ESP_OK;
}

bool web_client_connected(void)
{
    return s_client_connected;
}

const char *web_ap_ssid(void)
{
    return s_ssid;
}