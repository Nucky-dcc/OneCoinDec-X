/*******************************************************************************
 *
 * ワンコインデコーダＸ  for WCH CH32V003 シリーズ (J4M6 / F4P6)
 * Copyright (C) 2026  Nucky
 * File Name  : main.c
 * Version    : 1.1.1
 * Date       : 2026/10/27
 * Author     : Nucky
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * See LICENSE-GPLv3.txt for details.
 *
 * 【概要】
 * 本ファイルは、デコーダの心臓部となるメイン処理が記述されています。
 * マイコン起動時の各種初期化（FlashからのCV読み込み、ピンやタイマーの設定等）
 * を行った後、無限ループ（while(1)）に入ります。
 * 
 * 無限ループ内では、大きく分けて以下の2つのタスクを並行して行います。
 * 1. 【1ms定期タスク】タイマー(TIM2)を参照し、1ミリ秒ごとにモーターの
 *    モメンタム（加減速）処理や、各種保護タイマーの減算を実行します。
 * 2. 【DCCパケット処理】割り込み処理によってバックグラウンドで受信された
 *    DCCパケットのエラーチェック(XOR計算)を行い、自分宛の命令であれば
 *    種類ごとに処理関数（ルーティング）へ引き渡します。
 *
 *******************************************************************************/
 
#include "decoder_config.h"
#include "cv_manager.h"
#include "motor_ctrl.h"
#include "dcc_core.h"

int main(void) {
    // ========================================================================
    //  1. マイコンの基本初期化
    // ========================================================================
    // 割り込みの優先度グループを設定 (グループ1: プリエンプション優先度1bit, サブ優先度3bit)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    
    // マイコンの動作周波数などのコア情報を更新、反映
    SystemCoreClockUpdate(); 
    
    // 自作モジュールの初期化
    EEPROM_Init();  // FlashメモリからCV値を読み出し(未初期化時はデフォルト値を書き込み)
    Outputs_Init(); // GPIOピン(モーター/ライト出力)、PWMタイマー、ADC(BEMF用)の初期化

    // ========================================================================
    //  2. DCC信号入力用のピンと外部割り込み(EXTI)の初期化
    // ========================================================================
    // 使用するGPIOポート(A, D)と、ピンの代替機能(AFIO)のクロックを供給して有効化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    
    // DCC信号入力ピンを「浮き(フローティング)入力」モードに設定
    // ※DCC信号はフォトカプラやオペアンプ等から明確なHigh/Lowで入力される前提です。
    GPIO_InitTypeDef gi = { DCC_PIN, GPIO_Speed_10MHz, GPIO_Mode_IN_FLOATING };
    GPIO_Init(DCC_PORT, &gi);
    
    // ターゲット基板(ピン数)に応じて、EXTIラインをどのポートのピンに接続するか切り替え
#if !defined(TARGET_20PIN_LOCO)
    // 8ピン版: GPIOAのピン(PA2)に接続
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, DCC_PIN_SOURCE);
#else
    // 20ピン版: GPIODのピン(PD4)に接続
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, DCC_PIN_SOURCE);
#endif

    // 割り込みトリガーの設定 (Rising_Falling: 立ち上がりと立ち下がりの両エッジで割り込みを発生させる)
    // DCC信号は交流であり、両方のエッジの時間を測ることでパルス幅を計算するため必須です。
    EXTI_InitTypeDef ei = { DCC_EXTI_LINE, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling, ENABLE };
    EXTI_Init(&ei);
    
    // EXTI7_0 割り込みをNVIC(ネスト型ベクタ割り込みコントローラ)に登録して有効化
    // 最優先(PreemptionPriority=1)に設定し、DCC信号の取りこぼしを防ぎます。
    NVIC_InitTypeDef ni = { DCC_IRQn, 1, 0, ENABLE }; 
    NVIC_Init(&ni);
    
    // ========================================================================
    //  3. ベースタイマー (TIM2) の初期化
    //     ※1us(マイクロ秒)単位でカウントアップし続けるフリーランニングタイマー。
    //     DCC信号のパルス幅計測や、1msごとの定期タスクの基準時間として使用します。
    // ========================================================================
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    // システムクロック(通常48MHz)を1MHz(1us周期)に分周するためのプリスケーラ値を計算して設定
    // カウント最大値は0xFFFF(65535)。65.5msごとに0に戻ります。
    TIM_TimeBaseInitTypeDef tb = { (uint16_t)((SystemCoreClock / 1000000) - 1), TIM_CounterMode_Up, 0xFFFF, TIM_CKD_DIV1, 0 };
    TIM_TimeBaseInit(TIM2, &tb); 
    TIM_Cmd(TIM2, ENABLE); // タイマーのカウントを開始

    // ========================================================================
    //  4. メインループ用 変数の準備
    // ========================================================================
    // メインループ内で安全にパケットを処理するためのローカルバッファ
    uint8_t local_buf[DCC_BUF_SIZE];
    
    // 1ms定期処理のタイミングを計るための、前回タスク実行時のタイマーカウント値
    uint16_t last_ms = TIM2->CNT;
    
    // ========================================================================
    //  5. メインループ (無限ループ)
    // ========================================================================
    while(1) {
        uint16_t now = TIM2->CNT;
        
        // --- A. 1ms (1000us) ごとの定期タスク実行 ---
        // タイマーの値(now)が前回(last_ms)から1000以上進んでいたらタスクを実行。
        // ※ 16bitタイマーのオーバーフロー(65535->0)も uint16_t の引き算で自動的に補正されます。
        if ((uint16_t)(now - last_ms) >= 1000) { 
            last_ms += 1000; // 次回の判定基準をきっちり1000us進める
            Process_1ms_Timer_Tasks(); // モーターの加減速(モメンタム)やタイマー減算を実行
        }
        
        // --- B. DCCパケットの受信完了チェックと解析処理 ---
        if (packet_ready) {
            // [排他制御] メインループでデータを読んでいる最中に、割り込み処理によって
            // パケットバッファが上書き破損するのを防ぐため、一時的に割り込みを禁止します。
            NVIC_DisableIRQ(DCC_IRQn);
            uint8_t len = byte_index; // 受信したパケットのバイト数
            // グローバルバッファからローカルバッファへデータをコピー
            for(uint8_t i = 0; i < len; i++) local_buf[i] = packet_buffer[i];
            
            // 次のパケットを受信できるようにフラグとインデックスをリセット
            packet_ready = 0; 
            byte_index = 0;
            // コピーが終わったので速やかにDCC割り込みを再開
            NVIC_EnableIRQ(DCC_IRQn); 
            
            // --- C. パケットのエラーチェック (チェックサムの検証) ---
            uint8_t ck = 0;
            // DCCパケットの最後のバイトは「エラー検出コード（XORチェックサム）」です。
            // 最後のバイト以外の全てのバイトを排他的論理和(XOR)して計算します。
            for (uint8_t i = 0; i < len - 1; i++) ck ^= local_buf[i];
            
            // 計算したチェックサム(ck)と、パケットの最後尾のバイトが一致するか確認。
            // 一致し、かつ最低限のバイト数(3バイト以上)を満たしている場合のみ処理に進みます。
            if (ck == local_buf[len - 1] && len >= 3) {
                
                // --- D. コマンドの種別判定と処理への振り分け (ルーティング) ---
                
                // 1. サービスモード(プログラミングトラックでのCV読み書き命令) (0111 xxxx)
                if ((local_buf[0] & 0xF0) == 0x70) {
                    Process_Service_Mode(local_buf, len);
                }
                // 2. アクセサリーコマンド(ポイントや信号機など) (10xx xxxx)
                else if ((local_buf[0] & 0xC0) == 0x80) {
                    Process_Accessory_Command(local_buf);
                }
                // 3. モバイルデコーダコマンド(機関車、車両用)
                else {
                    uint8_t cv29 = Read_CV(29); // コンフィギュレーション取得
                    uint8_t match = 0;  // 自分のアドレス宛のパケットかどうかの判定フラグ
                    uint8_t idx = 1;    // 命令(コマンド)データが格納されているバッファの開始位置
                    
                    // CV29のbit5(0x20)が立っていればロングアドレス(2バイトアドレス)を使用
                    if (cv29 & 0x20) { 
                        // ロングアドレスはパケットの第1, 第2バイトに格納されている。CV17, 18と比較。
                        if (len >= 4 && local_buf[0] == Read_CV(17) && local_buf[1] == Read_CV(18)) { 
                            match = 1; 
                            idx = 2; // アドレスが2バイト占有するため、命令データは[2]から始まる
                        } 
                    } 
                    // CV29のbit5が0の場合はショートアドレス(1バイトアドレス)を使用
                    else { 
                        // ショートアドレスはパケットの第1バイトに格納されている。CV1と比較。
                        if (local_buf[0] == Read_CV(1)) { 
                            match = 1; 
                            idx = 1; // アドレスが1バイト占有するため、命令データは[1]から始まる
                        } 
                    }
                    
                    // アドレスが一致した(自分宛の命令だった)場合のみ、具体的なコマンド処理を実行
                    if (match) {
                        Process_Loco_Command(local_buf, len, idx);
                    }
                }
            }
        }
    }
}