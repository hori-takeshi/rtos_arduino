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


APPL_CXXOBJS += ZumoMotors.o

#
#  Serialの扱い
#   ARDUINO_SERIAL
#    NOUSE_SERIAL
#    USE_SERIAL
#
#ARDUINO_SERIAL = NOUSE_SERIAL
ARDUINO_SERIAL = USE_SERIAL

#
#  ライブラリの使用
#  
USE_ARDUINO_SPI = true
#USE_ARDUINO_NCESCAN = true
#USE_ARDUINO_SD  = true
#USE_ARDUINO_TFT = true
#USE_ETHERNET2 = true
#USE_NAXESMOTION = true
#USE_WIRE = true

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
APPL_CFLAGS +=
APPL_LIBS +=
APPL_DIR     += 

APPL_DIR     += $(R2CA_DIR)/arduino_lib/libraries/ESP8266_Arudino_AT/
APPL_CXXOBJS += ESP8266.o

APPL_DIR     += $(R2CA_DIR)/arduino_lib/libraries/Pixy

include $(R2CA_DIR)/lib/Makefile.r2ca
