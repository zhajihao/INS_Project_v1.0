#ifndef __VERSION_H
#define __VERSION_H

#include <stdint.h>

/* 固件版本号 (当前版本)
 * 版本变更历史见仓库根目录 CHANGELOG.md; 逐条细节可用 git log / git show 查阅,
 * 本文件只保留版本宏, 不再重复记录历史。 */
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 5
#define FW_VERSION_PATCH 1
#define FW_VERSION_STR "IMU+NAV v1.5.1"

/* 编译时间由编译器自动填入。 */
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__
#define FW_BUILD_DATETIME __DATE__ " " __TIME__

/* 注: v1.0.6 引入的版本查询协议 (55 AA 01 01 02 0D 0A) 已于 v1.2.0 移除,
 *     当前 USART0 固定用于 GNSS1 数据接收 + 导航结果帧发送。 */

void version_print(void);
const char *version_get_str(void);
const char *version_get_build_date(void);
const char *version_get_build_time(void);

#endif /* __VERSION_H */
