#
#  ターゲットの指定（Makefile.targetで上書きされるのを防ぐため）
#
all:

#
#  アプリケーションファイル
#
APPL_CXXOBJS += r2ca_app.o
APPL_COBJS   += 
APPL_CFG      = r2ca_app.cfg

#
#  Serialの扱い
#   ARDUINO_SERIAL
#    NOUSE_SERIAL
#    USE_SERIAL
#    USE_SERIALUSB_AS_SERIAL
#    USE_SERIAL5_AS_SERIAL
#
#ARDUINO_SERIAL = NOUSE_SERIAL
ARDUINO_SERIAL = USE_SERIAL
#ARDUINO_SERIAL = USE_SERIALUSB_AS_SERIAL
#ARDUINO_SERIAL = USE_SERIAL5_AS_SERIAL

#
#  ライブラリの使用
#  
#USE_ARDUINO_SPI = true
#USE_ARDUINO_NCESCAN = true
#USE_ARDUINO_SD  = true
#USE_ARDUINO_TFT = true
#USE_ETHERNET2 = true
USE_NAXESMOTION = true
#USE_ARDUINO_RTC = true

#
#  R2CAのディレクトリの定義
#
R2CA_DIR = ../..

#
#  ASPのソースファイルのディレクトリの定義
#
SRCDIR = ../../asp_1.9.2

#
#  その他のオプション
#
APPL_CFLAGS =
APPL_LIBS =
APPL_DIR = 

include $(R2CA_DIR)/lib/Makefile.r2ca
