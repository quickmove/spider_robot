/* web_ctrl 内部接口（不暴露给其他组件） */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

/* 解析一条 WebSocket 文本帧（JSON） */
esp_err_t web_ws_handle_cmd(const char *json, size_t len);

/* 组装状态 JSON（供遥测与 /api/status 共用） */
void web_build_status(char *buf, size_t buf_len);

/* 主动向当前客户端发送一帧文本（供遥测任务用） */
esp_err_t web_send_text(const char *payload, size_t len);

/* 当前客户端的 socket fd，无连接时为 -1 */
int web_ws_fd(void);
