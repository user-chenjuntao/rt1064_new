#ifndef __DATA_HANDLE_H_
#define __DATA_HANDLE_H_

#include "zf_common_typedef.h"
#include "zf_common_fifo.h"
#include "zf_driver_uart.h"


// 1. 串口配置（与调试端口分离，使用独立UART）
#define UART_INDEX              UART_1                                          // 使用UART5接收OpenART数据
#define UART_BAUDRATE           115200                                          // 与OpenART波特率一致
#define UART_TX_PIN             UART1_TX_B12                                     // 发送引脚（可选，用于调试回传）
#define UART_RX_PIN             UART1_RX_B13                                     // 接收引脚（接OpenART的TX）

//#define UART_PRIORITY           LPUART1_IRQn                                     // UART5对应的中断号（MIMXRT1064.h中定义）

// 2. FIFO与缓冲区配置
#define FIFO_SIZE               256                                               // FIFO缓冲区大小

#define FIFO_READ_BUF_SIZE      256                                               // FIFO批量读取缓冲区大小

#define FRAME_BUF_SIZE          256                                               // 单帧数据缓冲区大小

#define MID_CAMERA_WIDTH        160
#define MID_CAMERA_HEIGHT       120
#define CAMERA_WIDTH        320
#define CAMERA_HEIGHT       240

// 3. 色块数据结构（解析后的数据）
typedef struct {
    int16_t cx, cy;  // 中心坐标
    int16_t w, h;    // 宽高
    int32_t area;    // 面积
    uint8_t valid;   // 数据有效性（1=有效，0=无效）
	int w_error;
	float distance;
} BlobInfo;

void uart_blob_init(void);
void parse_blob_frame(void);
void process_blob_data(void);

extern fifo_struct uart_data_fifo;
extern BlobInfo blob_info;

#endif
