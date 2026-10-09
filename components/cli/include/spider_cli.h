/*
 * 串口 CLI（应用层）
 *
 * 与 Web 控制协议共用同一套命令名（docs/protocol.md §5），
 * 只是输出为人类可读文本而非 JSON。放在独立组件而非 servo 层，
 * 以避免 servo <-> motion 的循环依赖。
 */

#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 CLI 任务（CONFIG_SPIDER_DEBUG_CLI 关闭时为空实现） */
esp_err_t spider_cli_start(void);

#ifdef __cplusplus
}
#endif
