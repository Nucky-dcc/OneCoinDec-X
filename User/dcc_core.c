/*******************************************************************************
 *
 * ワンコインデコーダＸ  for WCH CH32V003 シリーズ (J4M6 / F4P6)
 * Copyright (C) 2026  Nucky
 *
 * File Name  : dcc_core.c
 * Version    : 1.1.1
 * Date       : 2026/10/27
 * Author     : Nucky
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * See LICENSE-GPLv3.txt for details.
 *
 * 【概要】
 * 本ファイルは、線路から送られてくるDCC（Digital Command Control）信号を
 * 受信・解析するコア（中核）モジュールです。
 * 
 * 【主な機能】
 * 1. 外部割り込み（EXTI）とタイマー（TIM2）を利用したDCCパルスの幅測定
 * 2. 測定したパルス幅に基づく「0」「1」のビット判定と、パケットの組み立て（ステートマシン）
 * 3. 各種コマンド（サービスモード、アクセサリー、モバイルデコーダ）の構文解析
 *
 * 【DCC信号の基本仕様（NMRA規格）】
 * - 「1」のビット : 短い反転パルス（約58μs）
 * - 「0」のビット : 長い反転パルス（約100μs以上）
 * パケット構造: [プリアンブル(1が10個以上)] + [0] + [データバイト] + [0] + [データバイト]... + [1(終了)]
 *
 *******************************************************************************/
 
#include "dcc_core.h"
#include "cv_manager.h"
#include "motor_ctrl.h"

// ============================================================================
//  DCC受信ステートマシン用の状態定義
// ============================================================================
typedef enum { 
    DCC_STATE_PREAMBLE,    // プリアンブル（連続する '1'、パケット開始の合図）の受信待ち状態
    DCC_STATE_START_BIT,   // データバイト開始前のスタートビット（'0'）待ち状態
    DCC_STATE_DATA_BYTE,   // 8ビットのデータを受信中の状態
    DCC_STATE_END_BIT      // パケット終了を示すエンドビット（'1'）待ち状態 (※実装上START_BITで兼用)
} DCC_State_t;

// ============================================================================
//  内部(静的)グローバル変数
//  ※ 割り込み処理内で高速に状態を遷移・管理するための変数群です。
// ============================================================================
static volatile DCC_State_t dcc_state = DCC_STATE_PREAMBLE; // 現在の受信ステート
static volatile uint16_t preamble_count = 0; // 連続して受信した '1' の数
static volatile uint8_t  bit_count = 0;      // 現在のバイトで何ビット目を受信しているか (0-7)
static volatile uint8_t  current_byte = 0;   // 組み立て中の1バイト(8ビット)データ
static volatile uint16_t last_edge_time = 0; // 前回エッジ(波形の変化)を検知した時のタイマー(TIM2)カウント値

// ============================================================================
//  外部公開用グローバル変数 (dcc_core.h で extern 宣言)
// ============================================================================
volatile uint8_t  byte_index = 0;                 // バッファに格納されたバイト数
volatile uint8_t  packet_buffer[DCC_BUF_SIZE];    // 受信したパケットデータを格納する配列
volatile uint8_t  packet_ready = 0;               // パケット受信完了フラグ (1でメインループへ処理を要求)

// ----------------------------------------------------------------------------
// 関数名: Reset_DCC_State
// 役割  : DCC受信ステートマシンの強制リセット処理
// 補足  : EEPROM(Flash)への書き込み時など、割り込みが一時停止して受信状態が
//         おかしくなった（中途半端にパケットが切れた）場合に呼び出し、状態を初期化します。
// ----------------------------------------------------------------------------
void Reset_DCC_State(void) {
    dcc_state = DCC_STATE_PREAMBLE;
    preamble_count = 0; byte_index = 0; packet_ready = 0;
    last_edge_time = TIM2->CNT; // パルス幅計算の基準時間を現在時刻でリセット
    EXTI_ClearITPendingBit(DCC_EXTI_LINE); // 保留中の古い割り込みフラグ(ゴミ)を消去
}

// ----------------------------------------------------------------------------
// 関数名: Process_Service_Mode
// 役割  : サービスモード (プログラミングトラックでのCV読み書き) の処理
// 補足  : NMRA規格 S-9.2.3 に準拠したパケットを解析します。
//         ダイレクトモード（主流）とページモード（古い機器用）の両方に対応しています。
// ----------------------------------------------------------------------------
void Process_Service_Mode(volatile uint8_t *buf, uint8_t len) {
    uint8_t inst = buf[0]; // 命令(インストラクション)バイト
    
    // --- 1. Direct Mode (ダイレクトモード: 4バイトパケット) ---
    // [命令(CV上位ビット含む)] [CV下位バイト] [データ] [エラー検知]
    if (len >= 4) { 
        // 最初の2バイトからCV番号を復元。
        // (NMRA仕様: CV値は内部的には 実際のCV番号-1 で送られてくるため +1 します)
        uint16_t cv = (((inst & 0x03) << 8) | buf[1]) + 1;
        
        // (a) Byte操作命令 (0111 11CC または 0111 01CC) ※CCはCV上位2bit
        if ((inst & 0xF4) == 0x74) { 
            if (inst & 0x08) { // Write (書き込み命令)
                Send_Ack();           // モータ/ライトを駆動してコマンドステーションに応答(Ack: 電流引き込み)
                Write_CV(cv, buf[2]); // EEPROMへ書き込み
            } else { // Verify (読み出し照合命令)
                // コマンドステーションが推測した値(buf[2])と実際のCV値が一致していればAckを返す
                if (Read_CV(cv) == buf[2]) Send_Ack();
            }
        }
        // (b) Bit操作命令 (0111 10CC)
        // 特定のビットだけを変更または照合する命令です
        else if ((inst & 0xFC) == 0x78) { 
            // どのビット(0-7)を操作対象にするか抽出
            uint8_t bit_pos = buf[2] & 0x07;
            // 現在のCV値から該当ビットの値を抽出 (0 または 1)
            uint8_t bit_val = (Read_CV(cv) >> bit_pos) & 0x01;
            
            if (!(buf[2] & 0x10)) { // Verify Bit (ビット照合)
                // コマンドステーションが指定したビット値と一致すればAck
                if (bit_val == ((buf[2] >> 3) & 0x01)) Send_Ack(); 
            } else { // Write Bit (ビット書き込み)
                Send_Ack(); 
                // 指定されたビットを1にするか0にするかを判定して新しいCV値を生成
                uint8_t new_val = (buf[2] & 0x08) ? 
                                  (Read_CV(cv) | (1 << bit_pos)) :  // bitを立てる(OR演算)
                                  (Read_CV(cv) & ~(1 << bit_pos));  // bitを下ろす(AND NOT演算)
                Write_CV(cv, new_val); 
            }
        }
    }
    // --- 2. Paged Mode (ページモード: 3バイトパケット) ---
    // 古いコマンドステーションで使われる方式。[命令] [データ] [エラー検知]
    else if (len == 3 && (inst & 0xF0) == 0x70) { 
        uint8_t reg = inst & 0x07; // 対象のレジスタ番号 (0-7)
        
        if (reg == 5) { // ページレジスタ(CVのオフセット)への書き込み
            page_register = buf[1]; 
            Send_Ack(); 
        } else {
            // 操作対象のレジスタ番号とページレジスタから、実際のCV番号を算出
            uint16_t cv = (reg <= 3) ? (page_register - 1) * 4 + reg + 1 : // レジスタ1-4
                          (reg == 4) ? 29 :                                // レジスタ5 -> CV29
                          (reg == 6) ? 7  :                                // レジスタ7 -> CV7 (バージョン)
                          (reg == 7) ? 8  : 0;                             // レジスタ8 -> CV8 (メーカーID)
                          
            if (cv > 0) {
                if (inst & 0x08) { // Write (書き込み)
                    Send_Ack(); 
                    Write_CV(cv, buf[1]); 
                } else if (Read_CV(cv) == buf[1]) { // Verify (照合)
                    Send_Ack();
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
// 関数名: Process_Accessory_Command
// 役割  : アクセサリーコマンド (ポイントや信号機用) の処理
// ----------------------------------------------------------------------------
void Process_Accessory_Command(volatile uint8_t *buf) {
#ifdef TARGET_8PIN_POINT
    // ポイント動作中（保護用ガードタイム中）は連続命令を無視する
    if (guard_timer > 0) return;
    
    // 受信したアドレスとデコーダに設定されたアドレス(CV1, CV2)が一致するか確認
    // アドレス上位はペア部分のLSBをマスク(& 0xFE)して比較します。
    if (buf[0] == Read_CV(1) && (buf[1] & 0xFE) == Read_CV(2)) {
        // buf[1]の最下位ビットがポイントの切り替え方向(0:分岐 or 1:直進)
        uint8_t target_dir = buf[1] & 0x01;
        // 現在の方向と異なる場合のみ、ポイント駆動処理(Execute_Turnout)を実行
        if (target_dir != current_point_dir) { Execute_Turnout(target_dir); }
    }
#endif
}

// ----------------------------------------------------------------------------
// 関数名: Process_Loco_Command
// 役割  : モバイルデコーダ (機関車、車両用) コマンドの処理
// 補足  : 速度、方向、ファンクション(ライト等)、Ops Mode(PoM) のCV書き込みを処理します。
// ----------------------------------------------------------------------------
void Process_Loco_Command(volatile uint8_t *buf, uint8_t len, uint8_t idx) {
    uint8_t cmd = buf[idx]; // コマンドバイトの取得
    
    // 1. 128ステップ速度制御コマンド (0011 1111)
    if (cmd == 0x3F && len >= (idx + 3)) { 
        uint8_t new_dir = (buf[idx+1] & 0x80) ? 1 : 0; // 最上位ビットが方向 (1:前進, 0:後進)
        uint8_t raw = buf[idx+1] & 0x7F;               // 下位7ビットが速度データ (0-127)
        
        if (raw == 1) { 
            // raw==1 は「非常停止 (E-Stop)」を意味します
            current_speed = 0; cmd_speed = 0; is_kicking = 0; cmd_dir = new_dir; Update_Outputs();
        } else {
            // raw==0 は通常停止。それ以外は 1 引いて 1-126 の速度ステップとする
            uint8_t s = (raw == 0) ? 0 : raw - 1;
            
            // 停止状態から動き出す際、キックスタート(CV47)が設定されていれば有効化する
            if (current_dir == new_dir && current_speed == 0 && s > 0 && Read_CV(47) > 0) { 
                is_kicking = 1; kick_timer = Read_CV(47) * 10; 
            } else if (s == 0) is_kicking = 0; // 停止指示ならキックスタート解除
            
            // 目標方向と目標速度を更新
            cmd_dir = new_dir; cmd_speed = s;
        }
    } 
    // 2. 14/28ステップ速度制御コマンド (0100 0000 - 0111 1111)
    else if ((cmd & 0xC0) == 0x40) { 
        uint8_t new_dir = (cmd & 0x20) ? 1 : 0; // bit5 が方向
        // 28ステップ用にビットを再配置 (C S S S S -> S S S S C)
        uint8_t s_val = ((cmd&0x0F)<<1) | ((cmd&0x10)>>4);
        
        if (s_val == 2 || s_val == 3) { 
            // 2, 3 は非常停止
            current_speed = 0; cmd_speed = 0; is_kicking = 0; cmd_dir = new_dir; Update_Outputs();
        } else {
            // 28ステップの値を、内部処理用の128ステップ(最大126)スケールに変換
            // 例: 28ステップでの1段目は、128ステップの4段目に相当
            uint8_t s = (s_val <= 1) ? 0 : (s_val - 3) * 4; 
            
            // キックスタートの判定
            if (current_dir == new_dir && current_speed == 0 && s > 0 && Read_CV(47) > 0) { 
                is_kicking = 1; kick_timer = Read_CV(47) * 10; 
            } else if (s == 0) is_kicking = 0;
            
            cmd_dir = new_dir; cmd_speed = s;
        }
    } 
    // 3. ファンクション、グループ 1 (F0, F1-F4) コマンド (1000 0000)
    else if ((cmd & 0xE0) == 0x80) { 
        f0_state=(cmd&0x10)?1:0; f4_state=(cmd&0x08)?1:0; f3_state=(cmd&0x04)?1:0; f2_state=(cmd&0x02)?1:0; f1_state=(cmd&0x01)?1:0; 
        Update_Outputs(); // 状態を更新してピン出力に反映
    } 
    // 4. ファンクション、グループ 2 (F5-F8) コマンド (1011 0000)
    else if ((cmd & 0xF0) == 0xB0) { 
        f8_state=(cmd&0x08)?1:0; f7_state=(cmd&0x04)?1:0; f6_state=(cmd&0x02)?1:0; f5_state=(cmd&0x01)?1:0; 
        Update_Outputs(); 
    }
    // 5. ファンクション、グループ 2 拡張 (F9-F12) コマンド (1010 0000)
    else if ((cmd & 0xF0) == 0xA0) { 
        f12_state=(cmd&0x08)?1:0; f11_state=(cmd&0x04)?1:0; f10_state=(cmd&0x02)?1:0; f9_state=(cmd&0x01)?1:0; 
        Update_Outputs(); 
    }
    // 6. Ops Mode (PoM: Programming on the Main) CV書き込みコマンド (1110 11CC)
    // 本線上を走りながら（プログラミング線路を使わずに）CV値を変更する命令です。
    else if ((cmd & 0xFC) == 0xEC && len >= (idx + 4)) { 
        // 指定されたCVにデータを書き込む (Ack応答は不要)
        Write_CV((((cmd&0x03)<<8)|buf[idx+1])+1, buf[idx+2]); 
    } 
}

// ----------------------------------------------------------------------------
// 関数名: Process_DCC_Bit
// 役割  : 割り込みで計測された1ビット(0 または 1)をステートマシンで処理して
//         パケットを組み立てる関数。
// ----------------------------------------------------------------------------
static void Process_DCC_Bit(uint8_t bit) {
    switch (dcc_state) {
        case DCC_STATE_PREAMBLE:
            // プリアンブル(パケットの開始を知らせる連続した '1')を数える
            if (bit == 1) { 
                if (preamble_count < 250) preamble_count++; 
            } 
            else { 
                // '0'が来たとき、プリアンブルが10個以上連続していれば有効なパケットの開始とみなす
                if (preamble_count >= 10) { 
                    dcc_state = DCC_STATE_DATA_BYTE; 
                    bit_count = 0; current_byte = 0; byte_index = 0; 
                } 
                // 10個未満ならノイズと見なしてリセット
                preamble_count = 0; 
            }
            break;
            
        case DCC_STATE_DATA_BYTE:
            // 受信したビットを左にシフトしながらMSBファーストで1バイト(8ビット)のデータに組み立てる
            current_byte = (current_byte << 1) | bit;
            if (++bit_count == 8) { 
                // 1バイト(8ビット)完成したらバッファに格納し、次のスタート/エンドビット判定へ移行
                if (byte_index < DCC_BUF_SIZE) packet_buffer[byte_index++] = current_byte; 
                dcc_state = DCC_STATE_START_BIT; 
            }
            break;
            
        case DCC_STATE_START_BIT: 
            // バイトとバイトの間の区切りビットの判定
            if (bit == 0) { 
                // '0' なら次のデータバイトが続く(スタートビット)
                dcc_state = DCC_STATE_DATA_BYTE; bit_count = 0; current_byte = 0; 
            } 
            else { 
                // '1' ならパケットの終了(エンドビット)。受信完了フラグを立ててメインループへ処理を要求
                packet_ready = 1; 
                // 次のパケットのプリアンブル1個目としてカウントを始める
                dcc_state = DCC_STATE_PREAMBLE; preamble_count = 1; 
            }
            break;
            
        // 異常なステートに入った場合は強制リセット
        default: 
            dcc_state = DCC_STATE_PREAMBLE; preamble_count = 0; 
            break;
    }
}

// ----------------------------------------------------------------------------
// 外部割り込みハンドラ (EXTI7_0_IRQHandler)
// 役割  : DCC信号入力ピンの電位が変化（エッジ）した瞬間に呼ばれる割り込み。
// 補足  : レールからのDCC信号が H->L または L->H に変化した瞬間にハードウェアから呼ばれます。
//         ここでパルス幅（時間）を計測し、「0」か「1」かを判定します。
// ----------------------------------------------------------------------------
void EXTI7_0_IRQHandler(void) {
    // DCC信号ピン(EXTIライン)の割り込み要求フラグが立っているか確認
    if(EXTI_GetITStatus(DCC_EXTI_LINE) != RESET) { 
        EXTI_ClearITPendingBit(DCC_EXTI_LINE); // 次の割り込みを受け付けるためにフラグをクリア
        
        // 1us(マイクロ秒)単位でカウントアップする TIM2 の値を取得し、前回からの経過時間を計算
        uint16_t now = TIM2->CNT; 
        uint16_t elapsed = now - last_edge_time; // パルス幅
        
        // 短すぎるパルス幅(20us未満)は電気的なノイズとみなして無視する(チャタリング防止)
        if (elapsed < 20) return;
        last_edge_time = now; // 次回の計算のために現在時刻を保存
        
        // DCC信号の半波長(エッジがLowに落ちたタイミング)でのみパルス幅を評価する
        if (GPIO_ReadInputDataBit(DCC_PORT, DCC_PIN) == Bit_RESET) {
            
            // パルス幅が異常に長い(10000us = 10ms以上)場合は、レール電源遮断等とみなしリセット
            if (elapsed > 10000) { 
                dcc_state = DCC_STATE_PREAMBLE; preamble_count = 0; 
                return; 
            } 
            
            // NMRA規格では「1」は約58μs、「0」は100μs以上。
            // 閾値を 80us とし、80us以下なら 1、それより長ければ 0 として解釈関数に渡す
            Process_DCC_Bit((elapsed <= 80) ? 1 : 0);
        }
    }
}