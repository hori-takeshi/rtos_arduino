=====================================================================
       TOPPERS RTE/RTOS compatible with Arduino libraries : R2CA  
                                   Last Modified:2016 Oct 23 16:29:13
=====================================================================

○概要

TOPPERS RTE/RTOS compatible with Arduino libraries(R2CA)は，TOPPERS上で
Arduino ライブラリを使用するための環境である．

多くのコードをArduino IDE付属の物をベースとしているため，ライセンスは
GPLである．各コードのライセンスはコード毎のライセンスに従う．

○サポート

サポートしているプロセッサは，Arduino M0/M0 Pro である．

○動作確認済みのバージョン

動作確認済みのバージョンは次の通りである．

・Arduino IDE 1.7.11
・TOPPERS/ASPカーネル 1.9.2
・ Atmel Studio 7.0 (build 594)

○クイックスタート(デバッガなし/GDBによるデバッグ)

Windows環境でのデバッガなしの実行方法について説明する．

●インストール

Arduino IDE のインストール
 ・http://www.arduino.org/ からダウンロードしてインストール．
 ・インストーラに従ってインストールを実施する．

Arduino IDE のインストールパスの設定
 ・C:\Program Files (x86)\Arduino にインストールした場合は必要ない
 ・インストールしたフォルダを以下のファイルに設定する
   ・example/do_path.bat
     SET ARDUINO_DIR=C:\Program Files (x86)\Arduino
   ・asp_1.9.1/target/arduino_m0_gcc/
     ARDUINO_BASE_DIR_WIN = C:\Program Files (x86)\Arduino
     
●ビルド
 ・フォルダ ./example/basic を開く
 ・./do_make.bat を実行

●実行
 ・ボードのPROGRAMポートとPCのUSBを接続する．
 ・Arduino IDEを起動する
   ・ツール -> ポート -> COMx(Arduino M0 Pro (Programmmming Port)) を選択．
   ・ツール -> シリアルモニタ を選択してシリアルモニタを実行する．
   ・シリアルモニタの右下の速度を115200bpsに変更
 ・フォルダ ./example/basic を開く
 ・./do_run.bat を実行
 
●デバッグ
 ・フォルダ ./example/basic を開く
 ・./do_debug.bat を実行 
 
●クリーン
 ・フォルダ ./example/basic を開く
 ・./do_clean.bat を実行 
 
 
○クイックスタート(AtmelStudio使用)

Windows環境でAtmelStudioを使用した実行方法について説明する．

●インストール

Atmel Studio のインストール
 ・ http://www.atmel.com/ja/jp/tools/ATMELSTUDIO.aspx からダウンロードしてインストール．
 ・インストーラに従ってインストールを実施する．

●プロジェクトを開く

各サンプルのフォルダ以下にある，

\example\basic\asp.atsln

をタブルクリックするとAtmel Studioが起動する．

●ビルド

メニュー -> Build -> Build Solution を実行

●実行

メニュー -> Debug -> Start Debugging and Brake を実行

メモリに書き込まれるため，実行を開始する．
ファイルメニューからrca_app.cpp を選択してブレークポイントを置くことが
可能．


○動作モデル

Arduinoライブラリを実行するためのタスクとして，メインタスクとタスク1,
タスク2,...を用意している．メインタスクは必ず生成され，タスクxは，幾つ
生成するかはマクロで定義可能である．現状最大数は5個である．

それぞれのタスク名(ID)は次の通りである．

 メインタスク : R2CA_MAINTASK
 タスク1      : R2CA_TASK1
 タスク2      : R2CA_TASK2
 タスク3      : R2CA_TASK3
 タスク4      : R2CA_TASK4
 タスク5      : R2CA_TASK5 


各タスクは，次の関数を実行する．これらの関数の本体を記述する．

 メインタスク : setup/loop
 タスク1      : loop1
 タスク2      : loop2
 タスク3      : loop3
 タスク4      : loop4
 タスク5      : loop5
  
実行時の振る舞いは次の通りである．
  
OS起動後にメインタスクが実行状態となりsetup()が実行される．setup()実行
終了後に他のタスクが起動され，各ループ関数が実行される．
 
○スケジューリング

基本的には優先度ベースのスケジューリングとなる．

オプションでラウンドロビンスケジューリングを選択可能である．ラウンドロ
ビンスケジュールリングの周期と対象とする優先度はマクロで変更可能である．

ラウンドロビンスケジューリングの対象としない優先度では優先度ベースのス
ケジューリングとなる．

○マクロ

ユーザーインクルードファイル"rca_app.h"で指定可能なマクロについて説明
する．

R2CA_NUM_TASK
・メインタスク数以外のタスクの数

R2CA_MAINTASK_PRI/R2CA_TASKx_PRI（x:1～5）
・各タスクの優先度．
・指定しない場合
  ・全て'5'となる．

R2CA_MAINTASK_STACK_SIZE/R2CA_TASKx_STACK_SIZE（x:1～5）
・各タスクのスタックサイズ
・指定しない場合
  ・メインタスク : 2048
  ・タスクx      : 1024
  
R2CA_RR_SCHEDULE_CYCLE
・ラウンドロビンスケジューリングの周期(ms)
・指定しない場合
  ・1
  
R2CA_RR_SCHEDULE_PRI
・ラウンドロビンスケジューリングの対象の優先度
  ・LSBから優先度1,2,3と割り当てている
・指定しない場合
  ・0x0000

○ライブラリ

次のライブラリは動作確認済みである

コアライブラリ

コアライブラリは常にコンパイル&リンクされる．

  ・PWM
  ・digital入出力    
  ・delay
  ・analog入出力
  ・attachInterrupt()
     noInterrupt() : Primaskを使用しているため問題ない
   ・USB Uart  
   ・Tone
   
その他のライブラリ

次のライブラリは使用する場合はMakefileで各マクロをtrueに定義すること．

  ・SPI : USE_ARDUINO_SPI
  ・SD  : USE_ARDUINO_SD
  ・TFT : USE_ARDUINO_TFT
  ・RTC : USE_ARDUINO_RTC
  ・I2C : USE_ARDUINO_WIRE  
  ・ETHERNET2 : USE_ETHERNET2 
  ・NAXESMOTION : USE_NAXESMOTION
  ・NCESCAN : USE_NCESCAN
    
Wire(I2C)の排他制御

複数のタスクからWrireを使用する場合は，使用の前後で，以下のマクロを呼
び出して排他制御を行うこと．

WIRE_ENTER_CRITICAL;
WIRE_LEAVE_CRITICAL;


○ファイル
./lib         
  ・RCA関連のライブラリ
./example      
  ・R2CAのサンプル
./arduino_lib
  ・Arduino IDE付属のライブラリ(パッチ済み)
  ・ディレクトリ構成はオリジナルと同等だが以下のフォルダは必要ないので削除している．   
  ./hardware/arduino/avr
  ./hardware/arduino/sam
  ./hardware/arduino/samd/bootloaders 
  ./hardware/arduino/samd/libraries/GSM
  ./tools/avr
  ./tools/gcc-arm-none-eabi-4.8.3-2014q1
  ./tools/OpenOCD-0.9.0-arduino
  ./tools/CMSIS/CMSIS_RTX
  ./tools/CMSIS/CMSIS/Driver
  ./tools/CMSIS/CMSIS/DSP_Lib
  ./tools/CMSIS/CMSIS/Lib
  ./tools/CMSIS/CMSIS/RTOS  
  ./tools/CMSIS/CMSIS/SVD
  ./tools/CMSIS/CMSIS/UserCodeTemplates  
  ./tools/CMSIS/Device/ARM
  ./tools/CMSIS/Device/ATMEL/samd21 以外
./libraries
    以下のフォルダは必要無いと判断し削除している
    ./Ethernet
    ./Scheduler
    ./SpacebrewYun
    ./Ciao
    ./WiFi
    以下のファイルを追加している
    ./ESP8266 _Arduino_AT
      下記で公開されているコードをベースにM0に対応   
      https://github.com/itead/ITEADLIB_Arduino_WeeESP8266
    ./NcesCan 
      Seeedの2015/11/08版をベースにAPIを変更
      https://github.com/Seeed-Studio/CAN_BUS_Shield       
    ./Milkcocoa_Arduino_SDK
      ESP8266を使用してMilkcocoaに接続するためのライブラリ．
      https://github.com/milk-cocoa/Milkcocoa_Arduino_SDK   
    ./thingspeak-arduino
      ESP8266を使用してThingspeakに接続するためのライブラリ．    
      https://github.com/mathworks/thingspeak-arduino
    ./ArduinoJson
      BuleMixに接続するためのJsonライブラリ．
      https://github.com/bblanchon/ArduinoJson 
      third-partyのフォルダを削除
    ./pubsubclient-2.6
      BuleMixに接続するためのJsonライブラリ．
      https://github.com/knolleary/pubsubclient/releases/tag/v2.6 

./asp_1.9.2
  ・ASPカーネルのソースコード
      
○サンプルプログラム
./Basic
 ・Arduinoの基本的なプログラムの詰め合わせ．
 ・ソースの先頭のifdefで実行するプログラムを選択する．
./CompositeExample 
 ・各種機能を組み合わせたサンプル
./Milkcocoa_basic
 ・Milkcocoaに接続する基本的なサンプル．
./MultiTtask
 ・マルチタスクのサンプル．
./NAxesMotion
 ・9軸モーションシールドのサンプル．
./NCESCan
 ・NCES CAN シールドのサンプル．
./Profiling
 ・プロファイリングのサンプル
./RRscheduling
 ・ラウンドロビンスケジューリングのサンプル．
./ThingsSpeak_basic
 ・ThingsSpeakに接続する基本的なサンプル．
./WifiEcho
 ・Wifiのサンプル．

○コンフィギュレーション項目

●rca_app.h

以下の項目を rca_app.h に設定可能である．

メインタスク以外のRCAタスク数の指定 : 現状5個までサポート
 RCA_NUM_TASK : 0～5

優先度 : 定義しないと初期値を使用
 RCA_MAINTASK_SETUP_PRI
 RCA_MAINTASK_LOOP_PRI
 
 RCA_TASK1_SETUP_PRI
 RCA_TASK1_LOOP_PRI 
 ...
 
スタック : 定義しないと初期値を使用
 RCA_MAINTASK_STACK_SIZE
 
 RCA_TASK1_STACK_SIZE
 ... 

スケジューリング : 
 RCA_RR_SCEDULE : 
  ビット指定した優先度(0は無視)をラウンドロビンスケジューリングする
 RCA_RR_SCEDULE_CYCLE : ラウンドロビンスケジューリングの周期
 
●Makefile

以下の項目を指定可能である．

ARDUINO_SERIAL
シリアル(Serial)の扱いを設定以下のいずれかに定義
NOUSE_SERIAL : Serialは使用しない．syslog/logtaskを使用した出力が可能．
USE_SERIAL   : Serialを使用する．logtaskは使用しない．syslogは低レベル
               出力となる．
      
○Arduinoライブラリへのパッチ : 対象 IDE 1.7.10

●バグ 
./hardware/arduino/samd/variants/arduino_zero/variant.h : 68行目  定義ミス
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )

./hardware/arduino/samd/cores/arduino/USB/samd21_host.c : ハンドラ名の重複

62行目 定義の追加
 static void UHD_ISR(void);

74行目 ハンドラ名の変更
 USB_SetHandler(&UHD_ISR);
 
177行目 ハンドラ名の変更
 static void UHD_ISR(void)

./hardware/arduino/samd/cores/arduino/Tone.cpp : 210/227行目 レジスタクリア
      TCx->COUNT8.CTRLA.reg &= ~0x700;
      TCCx->CTRLA.reg &= ~0x700;
            
libraries\TFT\src\utility\いくつかのファイル
ARDUINO_ARCH_SAM -> ARDUINO_ARCH_SAD
glcdfont.c:1:#ifndef ARDUINO_ARCH_SAMD
Adafruit_ST7735.cpp:334:#if defined(ARDUINO_ARCH_SAMD)
Adafruit_ST7735.h:138: #if defined(ARDUINO_ARCH_SAMD)

libraries\NAxesMotion\NAxisMotion.h : 139行目
M0の時はアドレスを変更する．さらにシールドのADRESSのジャンパを外す
 
#if defined(ARDUINO_ARCH_SAMD)
	void initSensor(unsigned int address = 0x29);
#else /* !defined(ARDUINO_ARCH_SAMD) */    
	void initSensor(unsigned int address = 0x28);
#endif /* defined(ARDUINO_ARCH_SAMD) */    



●TOPPERS対応
/hardware/tools/CMSIS/CMSIS/Include/core_cm0plus.h : 646行目
割込み禁止・許可をAPIを呼び出すように変更

__STATIC_INLINE void NVIC_EnableIRQ(IRQn_Type IRQn)
{
//  NVIC->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
    extern void rca_ena_int(uint32_t intno);
    rca_ena_int(IRQn);
}


/** \brief  Disable External Interrupt

    The function disables a device-specific interrupt in the NVIC interrupt controller.

    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_DisableIRQ(IRQn_Type IRQn)
{
//  NVIC->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
    extern void rca_dis_int(uint32_t intno);
    rca_dis_int(IRQn);
}

/arduino_lib/libraries/SD/src/utility/Sd2Card.cpp
SPI使用時にセマフォを使用するよう変更．

/arduino_lib/hardware/arduino/samd/cores/arduino/delay.c : 58行目
指定時間以上待つ様にdelay()関数内でms経過まで待つ様に変更．(オリジナル
は-1)．

  } while ( _ulTickCount - start <= (ms) ) ;

●機能追加

Serial3の追加 
SERCOM2を使用
ピン
 5 : RX
 4 : TX 
http://ehbtj.com/electronics/arduino-m0-hacks

●ライブラリ対応

\arduino_lib\hardware\arduino\samd\cores\arduino\RingBuffer.h

ESP8266の通信用にリングバッファサイズを大きくする．

#define SERIAL_BUFFER_SIZE 1024


○本パッケージの名称

Aruduinoのガイドライン(https://www.arduino.cc/en/Main/FAQ)では，

Arduino Xxxxxx      : NG
Xxxxxx for Arduino  : OK

とあり，Arduinoとの関係が分かる書き方なら問題ないとのこと．

TOPPERS RTE/RTOS Compatible with Arduino libraries  : R2CA

以上．
