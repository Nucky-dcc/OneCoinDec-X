/*******************************************************************************
 *
 * ワンコインデコーダＸ  for WCH CH32V003 シリーズ (J4M6 / F4P6)
 * Copyright (C) 2026  Nucky
 * File Name  : decoder_config.h
 * Version    : 1.1.1
 * Date       : 2026/10/27
 * Author     : Nucky
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * See LICENSE-GPLv3.txt for details.
 *
 * 【概要】
 * ハードウェア（マイコンのピン配置、使用するタイマー、割り込み設定など）および
 * デコーダの「ビルド対象（ターゲット）」の定義をこのファイル一つに集約した
 * ヘッダファイルです。
 *
 * 【なぜこのファイルが必要なのか】
 * ワンコインデコーダXは、1つのソースコードで「20ピン車載用」「8ピン車載用」
 * 「ファンクション専用」「ポイント専用」といった複数のバリエーションの基板に
 * 対応するクロスコンパイル設計になっています。
 * プログラムのあちこちに直接ピン番号を書くのではなく、ここでマクロとして
 * 名前（例: DCC_PIN）を付けることで、ターゲットを切り替えるだけで全ファイルの
 * ピン割り当てが一括で切り替わるようになっています。
 *
 *******************************************************************************/

#ifndef DECODER_CONFIG_H
#define DECODER_CONFIG_H

#include "ch32v00x.h" // CH32V003シリーズ専用のレジスタ定義やペリフェラルライブラリ

// ============================================================================
//  ★ ターゲットハードウェア ＆ コンパイルスイッチの選択 ★
//  ----------------------------------------------------------------------------
//  ビルドする対象の基板やデコーダの種別をここで選択します。
//  有効にしたいターゲットのコメント(`//`)を外し、それ以外はコメントアウトしてください。
//  ※同時に複数有効にするとコンパイルエラーや予期せぬ動作を引き起こします。
// ============================================================================
//#define TARGET_20PIN_LOCO    // 車載用フルスペック（20ピン：CH32V003F4P6）: モーター制御＋BEMF＋多数のファンクション出力
#define TARGET_8PIN_LOCO       // 車載用（8ピン：CH32V003J4M6）: モーター制御＋前後ライト出力のみの標準版
//#define TARGET_8PIN_FUNC     // 車載用ファンクション（8ピン：CH32V003J4M6）: モーター出力なし、ライト等の出力専用
//#define TARGET_8PIN_POINT    // ポイント用（8ピン：CH32V003J4M6）: ポイントマシンのソレノイド駆動専用デコーダ

// ----------------------------------------------------------------------------
//  モータ出力機能の自動判定
// ----------------------------------------------------------------------------
// 選択されたターゲットが「モーターやソレノイドを駆動するタイプ」の場合、
// 自動的に HAS_MOTOR_OUTPUT というフラグ(マクロ)を定義します。
// これにより、後のソースコード内で #ifdef HAS_MOTOR_OUTPUT と書くだけで
// PWMタイマーなどの処理を一括で有効/無効化できるようになります。
#if defined(TARGET_20PIN_LOCO) || defined(TARGET_8PIN_LOCO) || defined(TARGET_8PIN_POINT)
    #define HAS_MOTOR_OUTPUT
#endif

// ----------------------------------------------------------------------------
//  モータードライバICの制御ロジック選択
// ----------------------------------------------------------------------------
// 使用するモータードライバIC（Hブリッジ）に合わせて、PWM制御の論理
// （アクティブハイかローか）を選択します。
#ifdef HAS_MOTOR_OUTPUT
    #define USE_TB67H450       // 負論理入力 (Active-Low) : ピンがLow(0)の時にモーターONになるドライバ用(TB67H450等)
    //#define USE_BD6231       // 正論理入力 (Active-High): ピンがHigh(1)の時にモーターONになるドライバ用(BD6231等)
#endif

// ============================================================================
//  ハードウェア抽象化 (HAL) ピン割り当て
//  ----------------------------------------------------------------------------
//  各ターゲット基板における実際のマイコンのピン（GPIOポート、ピン番号、タイマー等）の
//  割り当てを定義します。プログラム本体からはこのマクロ名を使って制御します。
// ============================================================================

#ifdef TARGET_20PIN_LOCO
    // ------------------------------------------------------------------------
    // 【20ピンパッケージ(F4P6)用 ピン設定】
    // ------------------------------------------------------------------------
    
    // DCC信号入力ピン (EXTI外部割り込みを使用)
    #define DCC_PORT GPIOD
    #define DCC_PIN  GPIO_Pin_4
    #define DCC_EXTI_LINE EXTI_Line4
    #define DCC_PIN_SOURCE GPIO_PinSource4
    
    // モーター制御出力ピン (TIM1のアドバンストPWM出力を利用)
    #define MOTOR_FWD_PORT GPIOD
    #define PIN_MOTOR_FWD  GPIO_Pin_2 // TIM1_CH1: 前進側出力
    #define MOTOR_REV_PORT GPIOC
    #define PIN_MOTOR_REV  GPIO_Pin_3 // TIM1_CH3: 後進側出力
    // モーターのPWM Duty比をレジスタに直接設定するためのマクロ
    #define SET_MOTOR_FWD(val) TIM_SetCompare1(TIM1, (val))
    #define SET_MOTOR_REV(val) TIM_SetCompare3(TIM1, (val))
    
    // ファンクション（ライト等の汎用）出力ピン
    #define LIGHT_PORT GPIOC
    #define PIN_F0_FWD GPIO_Pin_1     // F0_FWD (前進時ヘッドライト)
    #define PIN_F0_REV GPIO_Pin_2     // F0_REV (後進時テールライト)
    #define PIN_F1     GPIO_Pin_4     // AUX1出力
    #define PIN_F2     GPIO_Pin_5     // AUX2出力
    #define PIN_F3     GPIO_Pin_7     // AUX3出力
    #define PIN_F4     GPIO_Pin_6     // AUX4出力
    // 初期化やAck時の全点灯処理用にピンを一括指定するマスク (OR演算で結合)
    #define LIGHT_PINS (PIN_F0_FWD|PIN_F0_REV|PIN_F1|PIN_F2|PIN_F3|PIN_F4)

#elif defined(TARGET_8PIN_LOCO) || defined(TARGET_8PIN_POINT)
    // ------------------------------------------------------------------------
    // 【8ピンパッケージ(J4M6)用 (モーター有 / ポイント用) ピン設定】
    // ※ピン数が少ないため、必要最小限の割り当てになります。
    // ------------------------------------------------------------------------
    
    // DCC信号入力ピン
    #define DCC_PORT GPIOA
    #define DCC_PIN  GPIO_Pin_2
    #define DCC_EXTI_LINE EXTI_Line2
    #define DCC_PIN_SOURCE GPIO_PinSource2
    
    // モーター制御出力ピン (20ピン版とは使用するTIM1のチャンネルが異なります)
    #define MOTOR_FWD_PORT GPIOA
    #define PIN_MOTOR_FWD  GPIO_Pin_1 // TIM1_CH2: 前進側(またはポイント正転側)
    #define MOTOR_REV_PORT GPIOC
    #define PIN_MOTOR_REV  GPIO_Pin_4 // TIM1_CH4: 後進側(またはポイント反転側)
    #define SET_MOTOR_FWD(val) TIM_SetCompare2(TIM1, (val))
    #define SET_MOTOR_REV(val) TIM_SetCompare4(TIM1, (val))
    
    // ファンクション出力ピン (小型版のため前後ライトの2出力のみ)
    #define LIGHT_PORT GPIOC
    #define PIN_F0_FWD GPIO_Pin_1     // LED1 / F0 (前進用) / ポイント状態表示LED等
    #define PIN_F0_REV GPIO_Pin_2     // LED2 / F0 (後進用)
    #define LIGHT_PINS (PIN_F0_FWD | PIN_F0_REV)

#elif defined(TARGET_8PIN_FUNC)
    // ------------------------------------------------------------------------
    // 【8ピンパッケージ(J4M6)用 (ファンクション専用) ピン設定】
    // ※モーター出力を削り、その分のピンをライト等の出力(AUX)に回しています。
    // ------------------------------------------------------------------------
    
    // DCC信号入力ピン
    #define DCC_PORT GPIOA
    #define DCC_PIN  GPIO_Pin_2
    #define DCC_EXTI_LINE EXTI_Line2
    #define DCC_PIN_SOURCE GPIO_PinSource2
    
    // ファンクション出力ピン (計4出力を確保)
    // ※ポート(GPIOAとGPIOC)が跨っているため、個別のポートとピンを定義します
    #define PIN_F0_FWD_PORT GPIOC
    #define PIN_F0_FWD      GPIO_Pin_1 
    #define PIN_F0_REV_PORT GPIOC
    #define PIN_F0_REV      GPIO_Pin_2     
    #define PIN_F1_PORT     GPIOA
    #define PIN_F1          GPIO_Pin_1     
    #define PIN_F2_PORT     GPIOC
    #define PIN_F2          GPIO_Pin_4     
    
    // GPIOの一括初期化やAck処理用に、ポートごとのピンマスクを定義
    #define LIGHT_PINS_C    (PIN_F0_FWD | PIN_F0_REV | PIN_F2) // PortCに属するピン(3本)
    #define LIGHT_PINS_A    (PIN_F1)                           // PortAに属するピン(1本)
#endif

// ----------------------------------------------------------------------------
//  割り込み定義
// ----------------------------------------------------------------------------
// CH32V003特有の割り込み制約：
// 割り込みベクタの数が節約されており、EXTI0〜EXTI7までは
// 全て「EXTI7_0_IRQn」という1つの割り込みとしてCPUに通知されます。
// そのため全ターゲット共通で、EXTI7_0 をDCC信号受信用に使用します。
#define DCC_IRQn EXTI7_0_IRQn

#endif // DECODER_CONFIG_H