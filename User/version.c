#include "version.h"
#include <stdio.h>

/*
 * v1.1.0: USART0 已专用于 GNSS1 接收 (RX) + 导航结果发送 (TX),
 *         版本查询命令解析功能已移除.
 *         保留 version_get_str / version_get_build_date / version_get_build_time
 *         供其他模块查询版本信息.
 */

/**
 * @brief  上电打印固件版本和编译时间
 */
void version_print(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  Firmware Version : %s\r\n", FW_VERSION_STR);
    printf("  Build Date       : %s\r\n", FW_BUILD_DATE);
    printf("  Build Time       : %s\r\n", FW_BUILD_TIME);
    printf("  Build DateTime   : %s\r\n", FW_BUILD_DATETIME);
    printf("========================================\r\n");
    printf("\r\n");
}

/**
 * @brief  获取版本号字符串
 */
const char *version_get_str(void)
{
    return FW_VERSION_STR;
}

/**
 * @brief  获取编译日期
 */
const char *version_get_build_date(void)
{
    return FW_BUILD_DATE;
}

/**
 * @brief  获取编译时间
 */
const char *version_get_build_time(void)
{
    return FW_BUILD_TIME;
}
