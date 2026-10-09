/*
 * web_ctrl — WiFi AP + HTTP Server + WebSocket 控制通道
 *
 * 实现 docs/protocol.md §2~§4 的协议。本组件只做「协议解析 + 转发」，
 * 所有运动决策都在 motion 层——与 CLI 完全平级（M3 是第三个入口）。
 *
 * 端点:
 *   GET  /           控制页（内嵌固件，零外部依赖，离线可用）
 *   GET  /ws         控制协议（JSON 帧）
 *   GET  /api/status 一次性状态快照
 *   GET  /api/cal    读取 8 路标定
 *   POST /api/cal    保存标定
 *   GET  /api/params 当前几何与步态参数
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 AP + HTTP Server + 遥测任务 */
esp_err_t web_init(void);

/* 当前是否有 Web 客户端连着（用于 UI 提示，不参与安全逻辑） */
bool web_client_connected(void);

/* AP 的 SSID（形如 SpiderRobot-XXXX） */
const char *web_ap_ssid(void);

#ifdef __cplusplus
}
#endif