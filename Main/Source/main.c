/*
 * pencil_prod_counter_receiver
 * ペンシルケースカウンタ受信機
 *
 * データ受信時、モノスティックであれば内部ＬＥＤ、
 * そうでないときはDO4に接続されたＬＥＤを光らせる
 *
 * シリアルで受け付けるコマンド文字は以下の通り
 * "Q"または"q"　カウンタへ情報要求コマンド"Q"を送信する
 * "R"または"r"　カウンタへカウンタリセットコマンド"R"を送信する
 *
 * 受信されたデータはそのままシリアルで親に転送する
 *
 * スイッチのON/OFFはシリアルに送信される
 */

#define _MONOSTICK
//#define _DEBUG

#include <AppHardwareApi.h>
#include "utils.h"
#include "ToCoNet.h"
#include "serial.h"         // シリアル用
#include "string.h"
#include "sprintf.h"
#include "ToCoNet_mod_prototype.h" // ToCoNet モジュール定義(無線で使う)

#define UART_BAUD 115200 	// シリアルのボーレート
#ifndef _MONOSTICK
#define LED       9         // デジタル出力4
#else
#define LED       16        // モノスティックの場合
#endif
#define DI1       12        // デジタル入力1
#define DI2       13        // デジタル入力2

// ToCoNet 用パラメータ
#define APP_ID   0x67720103
#define CHANNEL  18

static uint32 u32Seq = 0;          // 送信パケットのシーケンス番号

static tsFILE sSerStream;          // シリアル用ストリーム
static tsSerialPortSetup sSerPort; // シリアルポートデスクリプタ

// メッセージ出力用
#define debug(...) vfPrintf(&sSerStream, LB __VA_ARGS__)

static bool_t sendBroadcast(char *p);

// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
    static int count = 0;

    static int sw1_cnt = 0;
    static bool sw1 = FALSE;
    static int sw2_cnt = 0;
    static bool sw2 = FALSE;

	// 起動時メッセージ
	if (eEvent == E_EVENT_START_UP) {

    }
    // データ受信でDOxをHIに。(LED ON)
    else if (eEvent == E_ORDER_KICK) {
        count = 40;
        // LED ON処理
        vPortSetLo(LED);
    }
	// 4ms毎タイマー
    else if (eEvent == E_EVENT_TICK_TIMER) {

        // LED OFF処理
        if (count > 0) {
            count--;
        } else {
            vPortSetHi(LED);
        }

        // スイッチ入力チェック
        if(sw1 == bPortRead(DI1)) {
            sw1_cnt = 0;
        } else {
            if(++sw1_cnt > 10) {
                sw1 = !sw1;
                if(sw1) {
                    debug("SW1 ON\n");
                } else {
                    debug("SW1 OFF\n");
                }
            }
        }
        if(sw2 == bPortRead(DI2)) {
            sw2_cnt = 0;
        } else {
            if(++sw2_cnt > 10) {
                sw2 = !sw2;
                if(sw2) {
                    debug("SW2 ON\n");
                } else {
                    debug("SW2 OFF\n");
                }
            }
        }


        // シリアル入力チェック
		while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort))
		{
			// FIFOキューから１バイトずつ取り出して処理する。
			int16 i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

            switch(i16Char & 255)
            {
            case 'R':
            case 'r':
#ifdef _DEBUG
                debug("Send RESET\n");
#endif
                sendBroadcast("R");
                break;
            case 'Q':
            case 'q':
#ifdef _DEBUG
                debug("Send REQUEST\n");
#endif
                sendBroadcast("Q");
                break;
#ifdef _DEBUG
            default:
                debug("Unknown command\n");
#endif
            }
		}
	}
}

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
        // 必要モジュール登録手続き
        ToCoNet_REG_MOD_ALL();
	} else {
        // SPRINTF 初期化
        SPRINTF_vInit128();

        // ToCoNet パラメータ
        sToCoNet_AppContext.u32AppId = APP_ID;
        sToCoNet_AppContext.u8Channel = CHANNEL;
        sToCoNet_AppContext.bRxOnIdle = TRUE; // アイドル時にも受信
        u32Seq = 0;

        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

		// シリアル出力用
		vSerialInit();
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

        vPortAsOutput(LED);
        vPortSetHi(LED);

        // MAC 層開始
        ToCoNet_vMacStart();
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
	switch(eEvent) {
	default:
		break;
	}
}

// パケット受信時
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
    static uint32 u32SrcAddrPrev = 0;
    static uint8 u8seqPrev = 0xFF;

    // 前回と同一の送信元＋シーケンス番号のパケットなら受け流す
    if (pRx->u32SrcAddr == u32SrcAddrPrev && pRx->u8Seq == u8seqPrev) {
        return;
    }

    // そのままシリアルへ出力
    char buf[64];
    int len = (pRx->u8Len < sizeof(buf)) ? pRx->u8Len : sizeof(buf)-1;
    memcpy(buf, pRx->auData, len);
    buf[len] = '\0';
    debug("%s\n", buf);
    //vfPrintf(&sSerStream, buf);

    u32SrcAddrPrev = pRx->u32SrcAddr;
    u8seqPrev = pRx->u8Seq;

    // LEDを一定時間ONする
    ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
}

// ブロードキャスト送信を実行
static bool_t sendBroadcast(char *p)
{
    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();//チップのS/N
    tsTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST;

    u32Seq++;
    tsTx.bAckReq = FALSE;
    tsTx.u8Retry = 0x02; // 送信失敗時は 2回再送
    tsTx.u8CbId = u32Seq & 0xFF;
    tsTx.u8Seq = u32Seq & 0xFF;
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

    // ペイロードを作成
    memcpy(tsTx.auData, p, strlen(p));
    tsTx.u8Len = strlen(p);

    // 送信
    return ToCoNet_bMacTxReq(&tsTx);
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
#ifdef _DEBUG
   debug(">> SENT %s seq=%08X\n", bStatus ? "OK" : "NG", u32Seq);
#endif
}

// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    //switch (u32DeviceId) {
    //default:
    //	break;
    //}
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	return FALSE;
}

// メイン
void cbToCoNet_vMain(void)
{
}
