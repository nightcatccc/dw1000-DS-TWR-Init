// 注:本程序基于安信可的开发板,系统晶振频率为16MHz

#include "stm32f10x.h" // Device header
#include "delay.h"
#include "UART.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include <string.h>

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 us and 1 us = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
#define POLL_TX_TO_RESP_RX_DLY_UUS 300   // RX after TX delay,当TX设置DWT_RESPONSE_EXPECTED位,自动在300uus后开启RX
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100 // 约为2.66ms,在RX到消息处理,直到信息发送的延迟时间,用于DS_TWR的延时
#define RESP_RX_TIMEOUT_UUS 2700

#define PRE_TIMEOUT 8

static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY 16436//TX天线的发射延时
#define RX_ANT_DLY 16436

static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'G', 'E', 'T', 'A', 0x21, 0, 0};

static uint8 rx_resp_msg_fromA[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'P', 'O', 'L', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_resp_msg_fromB[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'P', 'O', 'L', 'B', 0x10, 0x02, 0, 0, 0, 0};

static uint8 tx_final_msg_toA[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'F', 'I', 'N', 'A', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_final_msg_toB[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'F', 'I', 'N', 'B', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define ALL_MSG_COMMON_LEN 10       // pack str比较长度
#define ALL_MSG_SN_IDX 2            // 计数
#define FINAL_MSG_POLL_TX_TS_IDX 10 // 信息索引
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4 // 1 Timestamps = 4 char

static uint8 frame_seq_nb = 0; // 计数

/* pack缓存区,存RX接收的包 */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* debug用 状态寄存器 */
static uint32 status_reg = 0;

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
typedef signed long long int64;
static uint64 poll_tx_ts_B;
static uint64 resp_rx_ts_B;
static uint64 final_tx_ts_B;

static uint64 poll_tx_ts_A;
static uint64 resp_rx_ts_A;
static uint64 final_tx_ts_A;

/* Distant cal values */
#define SPEED_OF_LIGHT 299702547
static double tof;
static double distance;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

int main(void)
{
	/* LED Init */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	/* Preiph Init */
	delay_init();
	UART_Init();
	peripherals_init();

	UART_SendStr(USART1, "Start DW1000 Init\n");
	/* DW Init */
	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	port_set_dw1000_slowrate();
	if (dwt_initialise(DWT_LOADUCODE) == -1)
	{
		while (1)
		{
			UART_SendStr(USART1, "Init Fail\n");
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			deca_sleep(1000);
		}
	}
	port_set_dw1000_fastrate();
	dwt_configure(&config);
	dwt_setleds(1); // 允许dwt1000开关灯

	/* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* 设置预期响应的延迟和超时时间。请参阅下面的注释 4、5 和 6。由于此示例只处理一个始终具有相同延迟和超时的传入帧，因此这些值可以在此处一次性设置。 */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
	
    /* 无限循环，持续发起测距交换。 */
    while (1)
    {
        /* 将帧数据写入 DW1000 并准备传输 */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* TX 缓冲区零偏移 */
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /*TX 缓冲区中的零偏移，正在进行测距*///设置此帧为测距帧

        /* 开始传输，并指示期望收到响应，以便在帧发送后并经过 dwt_setrxaftertxdelay() 设置的延迟时间后自动启用接收。 */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//延时、响应

        /* 我们假设传输已成功完成，轮询接收帧或检查错误/超时。请参阅下面的注释 9。 */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        /* 在传输轮询消息后递增帧序列号（取模 256）。*/
        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* 清除 DW1000 状态寄存器中的有效接收帧事件和传输帧发送事件。 */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /*帧已接收，将其读取到本地缓冲区。*/
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* 检查帧是否为预期的来自配对设备 "DS TWR 响应器" 示例的响应.由于帧中的序列号字段不相关，因此将其清除，以简化帧的验证。*/
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg_fromA, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time_A;
                int ret;

                /* 获取轮询传输和响应接收的时间戳。*/
                poll_tx_ts_A = get_tx_timestamp_u64();
                resp_rx_ts_A = get_rx_timestamp_u64();

                /*计算最终消息传输时间。请参阅下面的注释 10。*/
                final_tx_time_A = (resp_rx_ts_A + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time_A);

                /* 最终传输时间戳是我们编程设置的传输时间加上 TX 天线延迟。 */
                final_tx_ts_A = (((uint64)(final_tx_time_A & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                /*将所有时间戳写入最终消息。请参阅下面的注释 11。 */
                final_msg_set_ts(&tx_final_msg_toA[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts_A);
                final_msg_set_ts(&tx_final_msg_toA[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts_A);
                final_msg_set_ts(&tx_final_msg_toA[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts_A);

                /* 编写并发送最终消息。请参阅下面的注释 8。*/
                tx_final_msg_toA[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg_toA), tx_final_msg_toA, 0); /* 缓冲区零偏移 */
                dwt_writetxfctrl(sizeof(tx_final_msg_toA), 0, 1); /*设置发射帧 */
                ret = dwt_starttx(DWT_START_TX_DELAYED);//延时、非响应

                /* 如果 `dwt_starttx()` 返回错误，放弃此次测距交换并继续进行下一个交换。请参阅下面的注释 12。*/
                if (ret == DWT_SUCCESS)
                {
                    /* 轮询 DW1000，直到 TX 帧发送事件被设置。请参见下方的注释 9。 */
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    /*清除TX事件 */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    /* 在最终消息传输完成后，递增帧序列号（模 256）。*/
                    frame_seq_nb++;
                }
            }
//			else if(memcmp(rx_buffer, rx_resp_msg_fromB, ALL_MSG_COMMON_LEN) == 0)
//			{
//				uint32 final_tx_time_B;
//                int ret;

//                /* Retrieve poll transmission and response reception timestamp. */
//                poll_tx_ts_B = get_tx_timestamp_u64();
//                resp_rx_ts_B = get_rx_timestamp_u64();

//                /* Compute final message transmission time. See NOTE 10 below. */
//                final_tx_time_B = (resp_rx_ts_B + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
//                dwt_setdelayedtrxtime(final_tx_time_B);

//                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
//                final_tx_ts_B = (((uint64)(final_tx_time_B & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

//                /* Write all timestamps in the final message. See NOTE 11 below. */
//                final_msg_set_ts(&tx_final_msg_toB[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts_B);
//                final_msg_set_ts(&tx_final_msg_toB[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts_B);
//                final_msg_set_ts(&tx_final_msg_toB[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts_B);

//                /* Write and send final message. See NOTE 8 below. */
//                tx_final_msg_toB[ALL_MSG_SN_IDX] = frame_seq_nb;
//                dwt_writetxdata(sizeof(tx_final_msg_toB), tx_final_msg_toB, 0); /* Zero offset in TX buffer. */
//                dwt_writetxfctrl(sizeof(tx_final_msg_toB), 0, 1); /* Zero offset in TX buffer, ranging. */
//                ret = dwt_starttx(DWT_START_TX_DELAYED);

//                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
//                if (ret == DWT_SUCCESS)
//                {
//                    /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
//                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
//                    { };

//                    /* Clear TXFRS event. */
//                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

//                    /* Increment frame sequence number after transmission of the final message (modulo 256). */
//                    frame_seq_nb++;
//                }
//			}
		
		}
        else
        {
            /* 清除 DW1000 状态寄存器中的 RX 错误/超时事件。 */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* 重置 RX 以正确重新初始化 LDE 操作。 */
            dwt_rxreset();
        }

        /* 在测距交换之间执行延迟。. */
        delay_ms(1000);	//注意这里有坑，正点的Delay函数不能大于1800左右
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}