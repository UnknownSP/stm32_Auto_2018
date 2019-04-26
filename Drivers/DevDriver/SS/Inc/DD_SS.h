/* ===Kisarazu RBKN Library===
 *
 * autor          : Idutsu
 * version        : v0.10
 * last update    : 20180501
 *
 * **overview***
 * SDの通信プロトコルを定める。
 *
 * ・I2Cのみのサポート
 */

#ifndef __SS_H
#define __SS_H
#include <stdint.h>

#define RINGSIZE 32

typedef enum{
  D_STYP_ENCODER    = 0,
  D_STYP_SRF02      = 1, //超音波距離センサ
  D_STYP_GP2Y0A710K = 2, //SHARP赤外線距離センサ
  D_STYP_BMX055     = 3, //9軸センサ
  D_STYP_PHOTOARRAY = 4, //フォトリフレクタアレイ
}DD_SSType_t;

typedef struct{
  uint8_t add;       /*I2C アドレス格納変数*/
  uint8_t data_size; /*データサイズ格納変数*/
  uint8_t data[8];   /*データ格納配列*/
  DD_SSType_t type;  /*センサ種類*/
}DD_SSHand_t;

typedef enum{
  TRANSMIT_PHASE = 0,
  RECEIVE_PHASE  = 1,
}DD_SSCom_Phase;


/*
 * 指定バイト受信。
 * [data[7:0](7:0)]*指定バイト分
 *
 */
int DD_receive2SS(void);
int SS_Init(DD_SSHand_t *dmd);

/*
 *SD handlerを表示。
 *
 *SD(Add:hex):[データサイズ],[受信データ]*データサイズ分
 */
void DD_SSHandPrint(DD_SSHand_t *dmd);

int DD_SSPutReceiveRequest(uint8_t num);
int DD_SSPullReceiveRequest(uint8_t *num);
int Empty_Check(void);

int Read_ENCODER(DD_SSHand_t *dmd);
int Read_SRF02(DD_SSHand_t *dmd);
int Read_GP2Y0A710K(DD_SSHand_t *dmd);
int Read_BMX055(DD_SSHand_t *dmd);
int Write_BMX055(DD_SSHand_t *dmd);
int Read_PHOTOARRAY(DD_SSHand_t *dmd);

#endif
