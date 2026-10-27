/*******************************************************************************
 *
 * ワンコインデコーダＸ  for WCH CH32V003 シリーズ (J4M6 / F4P6)
 * Copyright (C) 2026  Nucky
 * File Name  : motor_ctrl.c
 * Version    : 1.1.1
 * Date       : 2026/10/27
 * Author     : Nucky
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * See LICENSE-GPLv3.txt for details.
 *
 * 【概要】
 * 本ファイルは、デコーダの「出力制御」全般を担います。
 * コマンドステーションから受信した指示（目標速度・目標方向）を元に、
 * モメンタム（加減速）、速度カーブ（Vstart, Vmid, Vhigh）、キックスタート、
 * そしてBEMF（逆起電力）を用いたロードコンペンセーション（負荷補償）などを
 * 計算し、最終的なPWMのDuty比やGPIOのON/OFFを決定して出力します。
 * ファンクションマッピングによるライト制御やポイント制御もここで行われます。
 *
 *******************************************************************************/

#include "motor_ctrl.h"
#include "cv_manager.h"

// ============================================================================
//  グローバル変数 (実体定義)
// ============================================================================
uint8_t cmd_dir = 1, cmd_speed = 0;         // コマンドステーションからの目標値 (方向、速度)
uint8_t current_dir = 1, current_speed = 0; // デコーダが現在実際に出力している値 (モメンタム等で徐々に追従)
uint16_t guard_timer = 0;                   // ポイントデコーダ用の連続動作防止(保護)タイマー
uint8_t  is_kicking = 0;                    // キックスタート(起動時の電圧ブースト)実行中フラグ
uint16_t kick_timer = 0;                    // キックスタートの残り時間(ms)

#if defined(HAS_MOTOR_OUTPUT) && !defined(TARGET_8PIN_POINT)
static uint16_t momentum_timer = 0;         // 加減速(CV3, CV4)のステップ進行を管理するタイマー
static uint16_t reverse_pause_timer = 0;    // 方向転換時にモーターを一旦停止させるポーズ時間のタイマー
#endif

// ファンクション(F0-F12)のON/OFF状態を保持する変数 (1=ON, 0=OFF)
uint8_t f0_state = 0, f1_state = 0, f2_state = 0, f3_state = 0, f4_state = 0;
uint8_t f5_state = 0, f6_state = 0, f7_state = 0, f8_state = 0;
uint8_t f9_state = 0, f10_state = 0, f11_state = 0, f12_state = 0;

// ============================================================================
//  BEMF (逆起電力) 制御モジュール (20ピン専用)
//  ※モーターが回転する際に発電する電圧(BEMF)をADCで読み取り、
//    坂道などで速度が落ちた場合に自動でパワー(Duty比)を上げるフィードバック制御です。
// ============================================================================
#ifdef TARGET_20PIN_LOCO
int32_t bemf_integral = 0;      // PID制御におけるI(積分)項のエラー累積値
int32_t bemf_duty_adjust = 0;   // BEMF計算結果から導き出されたPWM Duty比の最終補正値(加減算値)
uint16_t bemf_sample_timer = 0; // BEMFを計測する周期(CV54)を管理するタイマー
uint8_t last_bemf_speed = 0;    // 前回PID計算時の速度(速度変更時に古い積分項をリセットするため)

/*******************************************************************************
 * 関数名: BEMF_Init
 * 役割  : BEMF監視用のADC(アナログ／デジタル変換)の初期化
 * 補足  : モーターの両端(PA1, PA2)の電圧を読み取るための設定を行います。
 *******************************************************************************/
void BEMF_Init(void) {
    // ADCと該当ポートのクロック有効化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); // ADCクロックの設定(8分周)
    
    // GPIOAのPin1, Pin2をアナログ入力モードに設定
    GPIO_InitTypeDef gi = {0};
    gi.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    gi.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gi);

    // ADC1の基本設定 (単独チャンネルをソフトウェアトリガで都度変換するモード)
    ADC_InitTypeDef ai = {0};
    ai.ADC_Mode = ADC_Mode_Independent;
    ai.ADC_ScanConvMode = DISABLE;
    ai.ADC_ContinuousConvMode = DISABLE;
    ai.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ai.ADC_DataAlign = ADC_DataAlign_Right; // 右詰め(0-1023)でデータ取得
    ai.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ai);
    ADC_Cmd(ADC1, ENABLE);

    // ADCのキャリブレーション(マイコン個体差や環境温度による誤差の自動補正)を実行
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

/*******************************************************************************
 * 関数名: Read_ADC
 * 役割  : 指定されたチャンネルのADC値を1回読み取る (ブロッキング処理)
 *******************************************************************************/
uint16_t Read_ADC(uint8_t channel) {
    // チャンネルとサンプリング時間を指定して変換スタート
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_43Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // 変換完了(EOCフラグ)待ち
    return ADC_GetConversionValue(ADC1);
}

/*******************************************************************************
 * 関数名: Measure_BEMF
 * 役割  : モーターへの通電を一瞬切って、モーターが発電している逆起電力を測る
 * 引数  : logical_dir (現在の論理的な進行方向。測るべきピンが変わるため)
 *******************************************************************************/
uint16_t Measure_BEMF(uint8_t logical_dir) {
    // 復帰用に現在のPWM出力(Duty比)を一時退避
    uint32_t saved_fwd = TIM1->CH1CVR;
    uint32_t saved_rev = TIM1->CH3CVR;
    
    // 逆起電力を測るため、一時的にモーターへの給電(PWM)を完全にOFF(フリーラン状態)にする
#ifdef USE_TB67H450
    // TB67H450は負論理(LowでON)のため、常時High(ATRLR+1)を出力してOFFにする
    SET_MOTOR_FWD(TIM1->ATRLR + 1); SET_MOTOR_REV(TIM1->ATRLR + 1);
#else
    // 正論理の場合は0でOFF
    SET_MOTOR_FWD(0); SET_MOTOR_REV(0);
#endif
    
    // PWMオフ直後のノイズ(フライバック電流等)が収まるのを待つ (CV53: ブランキングタイム)
    uint16_t blanking_delay = Read_CV(53);
    uint16_t start_time = TIM2->CNT;
    while((uint16_t)(TIM2->CNT - start_time) < (blanking_delay * 10));
    
    // ノイズ対策として4回連続でADCを読み取り、平均化するための合計値を計算
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        // 進行方向によって起電力が発生するピン(プラス側)が異なるため、読むチャンネルを切り替える
        if (logical_dir) sum += Read_ADC(ADC_Channel_1); 
        else sum += Read_ADC(ADC_Channel_0);             
    }
    
    // モーターへの給電(PWM)を元の状態に復帰
    SET_MOTOR_FWD(saved_fwd);
    SET_MOTOR_REV(saved_rev);
    
    return (uint16_t)(sum / 4); // 4回の平均値を返す
}

/*******************************************************************************
 * 関数名: Calculate_BEMF_PID
 * 役割  : BEMFのPID(PI)演算を行い、PWMのDuty補正値(bemf_duty_adjust)を決定する
 * 補足  : CV51(Pゲイン)、CV52(Iゲイン)を用いて目標速度とのズレを計算します。
 *******************************************************************************/
void Calculate_BEMF_PID(void) {
    uint8_t cv10_cutout = Read_CV(10); // BEMF効果をフェードアウト(無効化)し始める速度ステップ

    // 停止中、最高速、またはカットアウト速度を超過している場合はBEMF制御を行わない
    if (current_speed <= 1 || current_speed >= 126 || (cv10_cutout > 0 && current_speed >= cv10_cutout)) {
        if (bemf_duty_adjust != 0) { 
            bemf_integral = 0; 
            bemf_duty_adjust = 0; // 補正をリセットして出力更新
            Update_Outputs(); 
        }
        return; 
    }

    // 目標速度が変更された場合は、過去の積分エラー(I項)の蓄積をリセットする
    if (current_speed != last_bemf_speed) {
        bemf_integral = 0;
        last_bemf_speed = current_speed;
    }

    // CV29の方向反転を考慮した論理的な進行方向
    uint8_t logical_dir = current_dir;
    if (Read_CV(29) & 0x01) logical_dir ^= 1;
    
    // 実際のBEMF値を取得 (0-1023)
    uint16_t current_bemf = Measure_BEMF(logical_dir);
    
    // 目標となるBEMF値 (現在の速度ステップに比例した理想的なADC値)
    int32_t target_bemf = ((int32_t)current_speed * 1023) / 127;
    
    // エラー(目標値と実際の回転数のズレ)を計算
    int32_t error = target_bemf - (int32_t)current_bemf;
    
    // CV値からPIDゲインを取得
    uint8_t kp = Read_CV(51); // P: 比例ゲイン (現在のズレに対する補正の強さ)
    uint8_t ki = Read_CV(52); // I: 積分ゲイン (過去のズレの蓄積に対する補正の強さ)
    
    // P(比例)項の計算 (整数演算のため、後で /100 してスケールを合わせる)
    int32_t p_term = (error * kp) / 100;
    
    // I(積分)項の計算 (ワインドアップ現象=積分値の暴走を防ぐため、上限下限を設定)
    bemf_integral += error;
    if (bemf_integral > 10000) bemf_integral = 10000;
    if (bemf_integral < -10000) bemf_integral = -10000;
    
    int32_t i_term = (bemf_integral * ki) / 1000; // /1000 してスケール調整
    
    // P項とI項を足して、最終的なDuty比の補正量を決定
    bemf_duty_adjust = p_term + i_term;
    
    // カットアウト(CV10)設定による、高速域でのBEMF効果のフェードアウト処理
    if (cv10_cutout > 0) {
        bemf_duty_adjust = (bemf_duty_adjust * (cv10_cutout - current_speed)) / cv10_cutout;
    }

    // 補正量が大きすぎるとガクガク動く(発振する)ため、±120(Duty比の約半分弱)にリミットをかける
    if (bemf_duty_adjust > 120) bemf_duty_adjust = 120;
    if (bemf_duty_adjust < -120) bemf_duty_adjust = -120;
    
    // 計算した補正値を実際のPWMに反映させる
    Update_Outputs(); 
}
#endif

// ============================================================================
//  モーターおよび出力制御の基本機能
// ============================================================================

/*******************************************************************************
 * 関数名: Update_PWM_Freq
 * 役割  : モーター駆動用PWMのキャリア周波数を更新(設定)する
 * 補足  : CV9の設定に基づき、タイマー1の分周比と周期を変更します。
 *******************************************************************************/
#ifdef HAS_MOTOR_OUTPUT
void Update_PWM_Freq(void) {
    uint32_t target_hz = 32000; // デフォルト 32kHz (可聴域外、モーターからピー音が鳴らない設定)
#ifdef TARGET_8PIN_POINT
    target_hz = 32000;          // ポイントデコーダは32kHz固定
#else
    // CV9の設定に従ってPWM周波数を切り替え
    uint8_t cv9 = Read_CV(9);
    uint8_t base = cv9 & 0x07;
    if (base == 1) target_hz = 8000;       // 8kHz
    else if (base == 2) target_hz = 2000;  // 2kHz
    else if (base == 3) target_hz = 500;   // 500Hz (古いモーター等、トルク重視用)
    if (cv9 & 0x08) target_hz /= 2;        // bit3が立っていたらさらに半分にする機能
#endif
    
    // タイマー(TIM1)の分周比(Prescaler)と周期(Autoreload)を計算
    uint32_t clk = SystemCoreClock; 
    uint16_t psc = 0;
    // 周波数が低すぎて16bitタイマーの限界(65535)を超える場合は分周比(psc)を上げる
    if ((clk / target_hz) > 65000) { psc = 7; clk /= 8; }
    uint16_t period = (uint16_t)((clk / target_hz) - 1);
    
    TIM_PrescalerConfig(TIM1, psc, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM1, period); // PWMの100%となる上限値を設定
    
    Update_Outputs(); // 周期(上限値)が変わったので、それに合わせてDuty比も再計算して設定
}
#endif

/*******************************************************************************
 * 関数名: Outputs_Init
 * 役割  : マイコン起動時のピン、タイマー初期化
 * 補足  : GPIOやTIM1のアウトプットコンペアを設定します。
 *******************************************************************************/
void Outputs_Init(void) {
    // 必要なGPIOポートとAFIOのクロックを有効化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
#ifdef HAS_MOTOR_OUTPUT
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // TIM1(PWM出力用)
#endif

#ifdef TARGET_20PIN_LOCO
    BEMF_Init(); 
#endif

    // --- GPIOピンの初期化 ---
    GPIO_InitTypeDef gi = {0};
    gi.GPIO_Mode = GPIO_Mode_Out_PP; // プッシュプル出力 (自力で電圧をドライブするモード)
    gi.GPIO_Speed = GPIO_Speed_10MHz;
    
#ifdef TARGET_8PIN_FUNC
    // ファンクション専用版: CポートとAポートを個別に初期化
    gi.GPIO_Pin = LIGHT_PINS_C; GPIO_Init(GPIOC, &gi);
    gi.GPIO_Pin = LIGHT_PINS_A; GPIO_Init(GPIOA, &gi);
#else
    // ファンクション用ピンの初期化
    gi.GPIO_Pin = LIGHT_PINS; GPIO_Init(LIGHT_PORT, &gi);
    #ifdef HAS_MOTOR_OUTPUT
    // モーター出力ピンはTIM1の代替機能(AF_PP: Alternate Function Push-Pull)として設定
    gi.GPIO_Pin = PIN_MOTOR_FWD; gi.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(MOTOR_FWD_PORT, &gi);
    gi.GPIO_Pin = PIN_MOTOR_REV; gi.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(MOTOR_REV_PORT, &gi);
    #endif
#endif

    // --- TIM1 PWM出力の初期化 ---
#ifdef HAS_MOTOR_OUTPUT
    // 基本設定 (周期などは Update_PWM_Freq で再設定されます)
    TIM_TimeBaseInitTypeDef tb = {0, TIM_CounterMode_Up, 1499, TIM_CKD_DIV1, 0};
    TIM_TimeBaseInit(TIM1, &tb);
    
    // PWM1モードの設定 (カウンタ < 比較レジスタ の時に出力がアクティブになる)
    TIM_OCInitTypeDef oc = {TIM_OCMode_PWM1, TIM_OutputState_Enable, 0, 0, TIM_OCPolarity_High};
    
    // ターゲット基板によって使用するタイマーチャンネルが異なる
    #if defined(TARGET_20PIN_LOCO)
    TIM_OC1Init(TIM1, &oc); TIM_OC3Init(TIM1, &oc); // 20ピンはCH1, CH3
    #else
    TIM_OC2Init(TIM1, &oc); TIM_OC4Init(TIM1, &oc); // 8ピンはCH2, CH4
    #endif
    
    // アドバンストタイマ特有の出力許可設定
    TIM_CtrlPWMOutputs(TIM1, ENABLE); 
    TIM_Cmd(TIM1, ENABLE); // タイマー開始
    Update_PWM_Freq();     // CV9の設定に基づいてPWM周波数を適用
#endif
}

/*******************************************************************************
 * 関数名: Update_Point_LED
 * 役割  : ポイントデコーダ用のLED表示更新
 * 補足  : 現在の開通方向に応じて、2つのLEDをトグル(片方点灯・片方消灯)します。
 *******************************************************************************/
#ifdef TARGET_8PIN_POINT
static void Update_Point_LED(void) {
    if (current_point_dir == 0) {
        GPIO_WriteBit(LIGHT_PORT, PIN_F0_FWD, Bit_RESET); GPIO_WriteBit(LIGHT_PORT, PIN_F0_REV, Bit_SET); 
    } else {
        GPIO_WriteBit(LIGHT_PORT, PIN_F0_FWD, Bit_SET); GPIO_WriteBit(LIGHT_PORT, PIN_F0_REV, Bit_RESET); 
    }
}
#endif

/*******************************************************************************
 * 関数名: Update_Outputs
 * 役割  : すべての出力(ライト、モーター)の最新状態をハードウェアに書き込む
 * 補足  : 1msタスクやDCCパケット受信時など、状態が変化するたびに頻繁に呼ばれます。
 *******************************************************************************/
void Update_Outputs(void) {
#ifdef TARGET_8PIN_POINT
    Update_Point_LED(); return; // ポイントデコーダはモーターの常時駆動不要のためここで終了
#endif

    // CV29による方向反転を考慮した「論理的な進行方向」を決定
    uint8_t logical_dir = current_dir;
    if (Read_CV(29) & 0x01) logical_dir ^= 1;
    
    // ========================================================================
    //  1. ライト、ファンクション出力の決定 (ファンクションマッピング CV33-CV46)
    // ========================================================================
    uint8_t active = 0; // どの出力ピンをONにするかのビットマスク
    
    // DCCパケットから取得したf0_state等がONの場合、対応するCV値をマスクに合成(OR演算)していく
    if (f0_state) active |= (logical_dir) ? Read_CV(33) : Read_CV(34); // F0は方向でCV(出力ピン)が分かれる
    if (f1_state) active |= Read_CV(35); if (f2_state) active |= Read_CV(36);
    if (f3_state) active |= Read_CV(37); if (f4_state) active |= Read_CV(38);
    if (f5_state) active |= Read_CV(39); if (f6_state) active |= Read_CV(40);
    if (f7_state) active |= Read_CV(41); if (f8_state) active |= Read_CV(42);
    if (f9_state) active |= Read_CV(43); if (f10_state) active |= Read_CV(44);
    if (f11_state) active |= Read_CV(45); if (f12_state) active |= Read_CV(46);

    // 実際のGPIOピンへ書き込み (マスクとAND演算し、該当ビットが立っていればBit_SET)
#ifdef TARGET_8PIN_FUNC
    // ファンクション専用版はピンがバラバラなため個別書き込み
    GPIO_WriteBit(PIN_F0_FWD_PORT, PIN_F0_FWD, (active & 0x01) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(PIN_F0_REV_PORT, PIN_F0_REV, (active & 0x02) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(PIN_F1_PORT,     PIN_F1,     (active & 0x04) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(PIN_F2_PORT,     PIN_F2,     (active & 0x08) ? Bit_SET : Bit_RESET);
#else
    GPIO_WriteBit(LIGHT_PORT, PIN_F0_FWD, (active & 0x01) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(LIGHT_PORT, PIN_F0_REV, (active & 0x02) ? Bit_SET : Bit_RESET); 
    #ifdef TARGET_20PIN_LOCO
    GPIO_WriteBit(LIGHT_PORT, PIN_F1, (active & 0x04) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(LIGHT_PORT, PIN_F2, (active & 0x08) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(LIGHT_PORT, PIN_F3, (active & 0x10) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(LIGHT_PORT, PIN_F4, (active & 0x20) ? Bit_SET : Bit_RESET);
    #endif

    // ========================================================================
    //  2. モーターPWM出力の計算 (3点速度カーブ)
    // ========================================================================
    #ifdef HAS_MOTOR_OUTPUT
    uint32_t max_p = TIM1->ATRLR; // PWMの最大値(100% Duty時のカウント値)
    
    // 特殊モード: CV30=2 の場合、モータ出力をF0の転極回路（ポイント等の極性反転）として使う
    if (Read_CV(30) == 2) { 
        uint32_t cf = 0, cr = 0;
        // F0がONの時だけ、進行方向に応じてフルパワー(max_p+1)で通電
        if (f0_state) { if (logical_dir) cf = max_p+1; else cr = max_p+1; }
        SET_MOTOR_FWD(cf); SET_MOTOR_REV(cr);
    } 
    // 通常のモーター制御モード
    else { 
        uint16_t vs = Read_CV(2); // Vstart (起動電圧)
        uint32_t duty = 0;        // 算出する内部PWM値 (0-255ベース)
        
        if (current_speed > 0) {
            uint16_t kick = (is_kicking) ? Read_CV(48) : 0; // キックスタート時のブースト加算値
            uint16_t vh = Read_CV(5), vm = Read_CV(6);      // Vhigh(最高速), Vmid(中間速)
            if (vh <= 1) vh = 255;                          // Vhigh未設定(0)時はMAX(255)とする
            if (vm <= 1) vm = vs + (vh-vs)/2;               // Vmid未設定(0)時は直線的(中間)とする
            
            // 速度ステップ(1-126)を、Vstart-Vmid-Vhighの折れ線グラフ(3点カーブ)に当てはめる
            if (current_speed == 1) duty = vs + kick;
            // 中間速度以下の場合
            else if (current_speed <= 63) duty = vs + ((vm-vs)*(current_speed-1))/62 + kick;
            // 中間速度以上の場合
            else duty = vm + ((vh-vm)*(current_speed-63))/63 + kick;

#ifdef TARGET_20PIN_LOCO
            // BEMF制御有効時、PIDで計算された補正値(bemf_duty_adjust)を加減算する
            if (Read_CV(50) > 0 && current_speed > 1) {
                int32_t adjusted = (int32_t)duty + bemf_duty_adjust;
                if (adjusted < (int32_t)vs) adjusted = vs; // 補正でVstartより遅くならないように保護
                if (adjusted > 255) adjusted = 255;
                duty = (uint32_t)adjusted;
            }
#endif
            if (duty > 255) duty = 255;
            
            // 0-255の内部スケールを、実際のタイマー周期(max_p)のスケールに変換する
            duty = (duty * max_p) / 255;
        }

        // 実際のPWMレジスタへの書き込み (モータードライバICの仕様に合わせる)
#ifdef USE_TB67H450
        // TB67H450(負論理入力): ピンがLowの時にONになるため、Duty値を反転(引く)する
        duty = max_p - duty;
        if (logical_dir) { SET_MOTOR_FWD(max_p+1); SET_MOTOR_REV(duty); } // FWD: 常時High(OFF), REV: PWM波形
        else { SET_MOTOR_FWD(duty); SET_MOTOR_REV(max_p+1); }             // REV: 常時High(OFF), FWD: PWM波形
#else
        // BD6231等(正論理入力): ピンがHighの時にON
        if (logical_dir) { SET_MOTOR_FWD(duty); SET_MOTOR_REV(0); }
        else { SET_MOTOR_FWD(0); SET_MOTOR_REV(duty); }
#endif
    }
    #endif // HAS_MOTOR_OUTPUT
#endif // TARGET_8PIN_FUNC
}

/*******************************************************************************
 * 関数名: Send_Ack
 * 役割  : コマンドステーションにAck(受領確認)を返すための電流消費処理
 * 補足  : DCC規格では、デコーダは「60mA以上の電流を5ms以上引き込む」ことで
 *         コマンドステーションへ「Yes(完了)」を伝えます。
 *******************************************************************************/
void Send_Ack(void) {
    // ACK処理は一部のコマンドステーションとの通信互換性を考慮し、
    // 実機検証のうえ応答のタイミングを少し遅らせる (約5ms待機)
    uint16_t pre_wait = TIM2->CNT;
    while ((uint16_t)(TIM2->CNT - pre_wait) < 5000);

    // すべてのライト(AUX)を強制的に全点灯させて電流を消費する
#ifdef TARGET_8PIN_FUNC
    GPIO_WriteBit(GPIOC, LIGHT_PINS_C, Bit_SET);
    GPIO_WriteBit(GPIOA, LIGHT_PINS_A, Bit_SET);
#else
    GPIO_WriteBit(LIGHT_PORT, LIGHT_PINS, Bit_SET);
#endif

    // モーターも一瞬だけ全開で回して電流を消費する
#ifdef HAS_MOTOR_OUTPUT
    uint32_t mp = TIM1->ATRLR; 
    SET_MOTOR_FWD(mp+1); SET_MOTOR_REV(0);
#endif
    
    // Ack規定時間 (約10ms = 10000us) だけその状態を維持
    uint16_t start_time = TIM2->CNT;
    while ((uint16_t)(TIM2->CNT - start_time) < 10000);

    // ポイントデコーダ特有のモータードライバ復帰処理
#if defined(HAS_MOTOR_OUTPUT) && defined(TARGET_8PIN_POINT)
    SET_MOTOR_FWD(0); SET_MOTOR_REV(0);
#ifdef USE_TB67H450
    // TB67H450の場合、両入力Lowの「スリープ(スタンバイ)状態」に確実に入れるため、
    // PWM(AFIO)モードから一度汎用GPIOモードに戻してLowを出力します。
    GPIO_InitTypeDef gi = {PIN_MOTOR_FWD, GPIO_Speed_10MHz, GPIO_Mode_Out_PP}; GPIO_Init(MOTOR_FWD_PORT, &gi);
    gi.GPIO_Pin = PIN_MOTOR_REV; GPIO_Init(MOTOR_REV_PORT, &gi);
    GPIO_WriteBit(MOTOR_FWD_PORT, PIN_MOTOR_FWD, Bit_RESET); GPIO_WriteBit(MOTOR_REV_PORT, PIN_MOTOR_REV, Bit_RESET);
    // その後、再びPWM(AFIO)モードに復元
    gi.GPIO_Pin = PIN_MOTOR_FWD; gi.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(MOTOR_FWD_PORT, &gi);
    gi.GPIO_Pin = PIN_MOTOR_REV; gi.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(MOTOR_REV_PORT, &gi);
#endif
#endif

    // Ack完了後、元々の状態(ライト、モーターの本来の指令値)に復帰する
    Update_Outputs();
}

/*******************************************************************************
 * 関数名: Execute_Turnout
 * 役割  : ポイント切り替えの実行処理 (ポイントデコーダ専用)
 * 引数  : target_dir (0 または 1)
 *******************************************************************************/
void Execute_Turnout(uint8_t target_dir) {
#ifdef TARGET_8PIN_POINT
    // 1. 反転ディレイ (CV5: 電流を流し始める前の待ち時間)
    uint16_t delay = Read_CV(5) * 10;
    // (割り込みを邪魔しないようTIM2を使わずループで簡易的に待機)
    for (uint16_t i = 0; i < delay; i++) { for(volatile uint32_t j=0; j<4000; j++); }
    
    // 貫通電流などによる焼損を防ぐため、安全のため一旦両極をOFFにする
    SET_MOTOR_FWD(0); SET_MOTOR_REV(0);
    for(volatile uint32_t j=0; j<400; j++); 

    // 2. ソレノイド駆動
    // CV6のPWM Duty比で出力を制限し、大電流によるコイルの焼損を防ぐ
    uint32_t max_p = TIM1->ATRLR;
    uint32_t duty = (uint32_t)Read_CV(6) * max_p / 255;
    
    if (target_dir == 1) { 
#ifdef USE_TB67H450
        SET_MOTOR_FWD(max_p+1); SET_MOTOR_REV(max_p - duty); // TB67H450(負論理)
#else
        SET_MOTOR_FWD(duty); SET_MOTOR_REV(0); // 正論理
#endif
    } else { 
#ifdef USE_TB67H450
        SET_MOTOR_FWD(max_p - duty); SET_MOTOR_REV(max_p+1);
#else
        SET_MOTOR_FWD(0); SET_MOTOR_REV(duty);
#endif
    }

    // 3. 通電時間の維持 (CV3: ソレノイドが完全に切り替わるまで電流を流し続ける時間)
    uint16_t act_time = Read_CV(3) * 10;
    for (uint16_t i = 0; i < act_time; i++) { for(volatile uint32_t j=0; j<4000; j++); }

    // 4. 通電終了後の状態保持設定 (CV4 bit0) 
    //    1=通電終了後に完全OFF(スリープに移行)、0=保持(特殊なポイントマシン用)
    if (Read_CV(4) & 0x01) {
        SET_MOTOR_FWD(0); SET_MOTOR_REV(0);
#ifdef USE_TB67H450
        // Ack時と同様に、スリープに入れるため一時的にGPIO操作を行う
        GPIO_InitTypeDef gi = {PIN_MOTOR_FWD, GPIO_Speed_10MHz, GPIO_Mode_Out_PP}; GPIO_Init(MOTOR_FWD_PORT, &gi);
        gi.GPIO_Pin = PIN_MOTOR_REV; GPIO_Init(MOTOR_REV_PORT, &gi);
        GPIO_WriteBit(MOTOR_FWD_PORT, PIN_MOTOR_FWD, Bit_RESET); GPIO_WriteBit(MOTOR_REV_PORT, PIN_MOTOR_REV, Bit_RESET);
        gi.GPIO_Pin = PIN_MOTOR_FWD; gi.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(MOTOR_FWD_PORT, &gi);
        gi.GPIO_Pin = PIN_MOTOR_REV; gi.GPIO_Mode = GPIO_Mode_AF_PP; GPIO_Init(MOTOR_REV_PORT, &gi);
#endif
    }

    // 方向が変化した場合は、次回起動時のためにFlash(EEPROM領域のCV64)へ状態を保存
    if (current_point_dir != target_dir) {
        current_point_dir = target_dir;
        cv_cache[63] = current_point_dir; EEPROM_SaveAll(); 
    }
    
    // ガードタイム(CV9: 切り替え直後に連続動作を受け付けない保護時間)を設定
    guard_timer = (uint16_t)Read_CV(9) * 100;
    Update_Point_LED();
#endif
}

/*******************************************************************************
 * 関数名: Process_1ms_Timer_Tasks
 * 役割  : 1msごとの定期タスク (メインループから呼ばれる)
 * 補足  : モメンタムや各種保護タイマーのカウントダウンを行います。
 *******************************************************************************/
void Process_1ms_Timer_Tasks(void) {
    // ポイント用の連続動作ガードタイマーの減算
    if (guard_timer > 0) guard_timer--;
    
#if defined(HAS_MOTOR_OUTPUT) && !defined(TARGET_8PIN_POINT)
    // キックスタートタイマーの減算 (0になったらキックスタート終了)
    if (is_kicking && kick_timer > 0) { 
        if (--kick_timer == 0) { is_kicking = 0; Update_Outputs(); } 
    }
    
    // ========================================================================
    //  進行方向変更時のポーズ処理とモメンタム(加減速)処理
    // ========================================================================
    uint8_t target = cmd_speed; 
    uint8_t safe = 0; // 反転ポーズ中(速度0への強制減速中)を示すフラグ
    
    // 方向が切り替わった場合、すぐには逆転させず一度速度0(停止)へ向かわせる
    if (current_dir != cmd_dir) {
        target = 0; safe = 1; // 目標速度を0に強制
        // 完全に停止し、かつポーズ時間(250ms)が経過したら、実際の方向(current_dir)を切り替える
        if (current_speed == 0 && ++reverse_pause_timer >= 250) {
            current_dir = cmd_dir; reverse_pause_timer = 0; Update_Outputs();
        }
    } else { 
        reverse_pause_timer = 0; 
    }
    
    // モメンタム: 現在速度が目標速度に達していない場合、徐々に近づける
    if (current_speed != target) {
        momentum_timer++;
        // 目標より遅ければCV3(加速率)、速ければCV4(減速率)を参照する
        uint8_t cv = (current_speed < target) ? Read_CV(3) : Read_CV(4);
        
        // 方向転換のための強制減速中かつ、CV4=0(減速遅延なし)の場合は
        // 急停車による脱線を防ぐため少しだけ遅延(2)を挟む
        if (safe && cv == 0) cv = 2;
        
        // タイマーがCV値で設定された間隔(x4ms)に達したら1ステップ増減する
        if (momentum_timer >= (uint16_t)cv * 4) { 
            if (current_speed < target) current_speed++; else current_speed--; 
            momentum_timer = 0; Update_Outputs(); 
        }
    }

#ifdef TARGET_20PIN_LOCO
    // BEMFの定期サンプリング実行タイマー (CV54: サンプリング周期)
    if (Read_CV(50) > 0 && current_speed > 1) {
        uint8_t interval = Read_CV(54); 
        if (interval == 0) interval = 1; // 0除算や処理停止防止

        // 指定間隔(ms)ごとにPIDを計算する
        if (++bemf_sample_timer >= interval) { 
            bemf_sample_timer = 0;
            Calculate_BEMF_PID(); 
        }
    }
#endif

#elif defined(TARGET_8PIN_FUNC)
    // ファンクション専用デコーダの場合はモメンタム(加減速)処理が不要なため、
    // 即座に目標値(cmd_*)を現在値(current_*)として適用し出力します。
    if (current_dir != cmd_dir || current_speed != cmd_speed) { 
        current_dir = cmd_dir; current_speed = cmd_speed; Update_Outputs(); 
    }
#endif
}