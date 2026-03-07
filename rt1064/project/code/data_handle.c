#include "data_handle.h"
#include <string.h>
#include <stdio.h>

uint8_t uart_fifo_buf[FIFO_SIZE];                                                // FIFO挂载的缓冲区
fifo_struct uart_data_fifo;                                                      // FIFO对象
uint8_t fifo_read_buf[FIFO_READ_BUF_SIZE] = {0};                                 // 存储FIFO批量读取的数据
uint8_t frame_buf[FRAME_BUF_SIZE] = {0};                                         // 存储完整帧数据
uint16_t frame_idx = 0;                                                          // 帧缓冲区索引
volatile bool is_frame_complete = false;                                         // 帧完整标志


BlobInfo blob_info = {0};                                                         // 初始化为无效

//串口与FIFO初始化
void uart_blob_init(void) {
    // 初始化FIFO（8位数据，挂载缓冲区）
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_fifo_buf, FIFO_SIZE);
    fifo_clear(&uart_data_fifo);  // 清空FIFO

    // 初始化串口
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);  // 开启接收中断
    
}

//传入BlobInfo结构体指针，将其所有成员清0
void clearBlobInfo(BlobInfo* blob) {
    // 空指针判断，避免访问非法内存
    if (blob == NULL) {
        return;
    }
    
    // 结构体各成员清0
    blob->cx = 0;       // 中心X坐标清0
    blob->cy = 0;       // 中心Y坐标清0
    blob->w = 0;        // 宽度清0
    blob->h = 0;        // 高度清0
    blob->area = 0;     // 面积清0
    blob->valid = 0;    // 有效性标记清0（0表示无效）
	blob->w_error = 0;
	blob->distance = 0;
}

//数据解析函数（从完整帧中提取色块信息）
void parse_blob_frame(void) {
    // 帧格式：#cx,cy,w,h,area\r\n（例如：#120,80,50,30,1500\r\n）
    int ret = sscanf((char*)frame_buf, "#%hd,%hd,%hd,%hd,%d\r\n",
                   &blob_info.cx, &blob_info.cy,
                   &blob_info.w, &blob_info.h,
                   &blob_info.area);

    // 校验解析结果（参数个数+数值范围）
    if (ret == 5 
        && blob_info.cx >= 5 && blob_info.cx < 315  // 符合QVGA分辨率x范围
        && blob_info.cy >= 50 && blob_info.cy < 240  // 符合QVGA分辨率y范围
        && blob_info.w > 10 && blob_info.h > 10       // 宽高必须为正
        && blob_info.area > 200)                   // 面积符合OpenART阈值
	{
        blob_info.valid = 1;  // 数据有效
		blob_info.w_error = blob_info.cx - MID_CAMERA_WIDTH;
    } else {
        blob_info.valid = 0;  // 数据无效
    }
	
	if (blob_info.valid == 0)
	{
		clearBlobInfo(&blob_info);
	}

    // 清空帧缓冲区，准备下一帧
    memset(frame_buf, 0, FRAME_BUF_SIZE);
    frame_idx = 0;
    is_frame_complete = false;
}

//主循环数据处理（用fifo_read_buffer批量读取FIFO，提取完整帧）
void process_blob_data(void) {

    // 步骤1：用fifo_read_buffer批量读取FIFO数据（一次读所有可用数据）
	uint32_t read_len = 0;
	fifo_state_enum read_state;
    read_len = fifo_used(&uart_data_fifo);  // 最大读取长度（不超过缓冲区）
	if (read_len != 0)
	{
		read_state = fifo_read_buffer(&uart_data_fifo, fifo_read_buf, &read_len, FIFO_READ_AND_CLEAN);
		// 步骤2：若读取成功且有数据，遍历数据提取完整帧
		if (read_state == FIFO_SUCCESS && read_len > 0) {


			for (uint32_t i = 0; i < read_len && frame_idx < FRAME_BUF_SIZE - 1; i++) {
				uint8_t data = fifo_read_buf[i];

				// 查找帧头'#'：未找到则丢弃前面的无效数据
				if (frame_idx == 0 && data != '#') {
					continue;
				}

				// 存储数据到帧缓冲区
				frame_buf[frame_idx++] = data;

				// 查找帧尾'\r\n'（判断最后2字节是否为帧尾）
				if (frame_idx >= 2 
					&& frame_buf[frame_idx-2] == '\r' 
					&& frame_buf[frame_idx-1] == '\n') {
					is_frame_complete = true;  // 标记帧完整
					break;  // 找到完整帧，退出遍历
				}
			}
		}
	}
   
    // 步骤3：若帧完整，解析数据
    if (is_frame_complete) {
        parse_blob_frame();
    }

}
