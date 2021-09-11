DEVICE ?= cloudcrowd00001
PORT ?= /dev/ttyUSB0
BAUD ?= 460800
#BAUD ?= 115200
PROXYHOST ?= piranha
PROXYPORT ?= $(PORT)
DISTHOST ?= iocore
PROGRAM ?= $(shell basename $${PWD})
DISTDIR ?= firmware/$(PROGRAM)

CORE ?= esp32
CHIP ?= esp32
BOARD ?= ttgo-lora32-v1
FQBN=$(CORE):$(CHIP):$(BOARD)
IDF ?= $(HOME)/src/esp/esp-idf-v4.0

ifeq ($(CORE),esp32)
## Old style packages/core/ hierarchy
HWDIR ?= $(HOME)/.arduino15/packages
SDKVERSION ?= $(shell ls -1 $(HWDIR)/$(CORE)/hardware/$(CHIP)/ | tail -1)
OTAPROG ?= $(HWDIR)/$(CORE)/hardware/$(CHIP)/$(SDKVERSION)/tools/espota.py
ifeq ($(CHIP),esp8266)
ESPTOOL ?= $(HWDIR)/$(CORE)/hardware/$(CHIP)/$(SDKVERSION)/tools/esptool/esptool.py
else
ESPTOOL ?= $(HWDIR)/$(CORE)/hardware/$(CHIP)/$(SDKVERSION)/tools/esptool.py
endif
else
## New style (2020) Arduino/hardware/core/chip
HWDIR ?= $(HOME)/Arduino/hardware
OTAPROG ?= $(HWDIR)/$(CORE)/$(CHIP)/tools/espota.py
ESPTOOL ?= $(HWDIR)/$(CORE)/$(CHIP)/tools/esptool.py
endif # CORELIB
## End of hardware directory voodoo

LIBDIR ?= $(HOME)/Arduino/libraries
BINDIR ?= build/$(shell echo $(FQBN) | tr : .)
OTAPASS ?= itsatrap

#CCFLAGS ?= --verbose --warnings all
MAIN = $(PROGRAM).ino
OBJ = $(BINDIR)/$(PROGRAM).ino.bin
SRCS = $(MAIN) config.h gps.ino mpu6050.ino oled.ino Makefile

# LIBS are the libraries you can install through the arduino library manager
# Format is LIBNAME[@VERSION]
LIBS = 	"Adafruit SGP30 Sensor" \
	DallasTemperature \
	"ESP8266 and ESP32 Oled Driver for SSD1306 display" \
	"Sharp GP2Y Dust Sensor" \
	OneWire \
	"SparkFun SCD30 Arduino Library" \

# EXTRALIBS are the libraries that are not in the arduino library manager, but installed via git
# Format is LIBNAME@REPOURL
EXTRALIBS = DHT12_sensor_library@https://github.com/xreef/DHT12_sensor_library \
	SimpleMap@https://github.com/spacehuhn/SimpleMap \
	Shell@https://github.com/accelerando-consulting/Shell.git 


build: $(OBJ)

$(OBJ): $(SRCS) Makefile
	arduino-cli compile -b $(FQBN) --build-cache-path . $(CCFLAGS) $(MAIN)

localdeps:
	cd $(LIBDIR)/.. && \
	git clone https://github.com/botletics/SIM7000-LTE-Shield.git && \
	ln -s $${PWD}/SIM7000-LTE-Shield/Code $(LIBDIR)/SIM7000-LTE-Shield

clean:
	rm -f $(OBJ)

ota: $(OBJ)
	@if [ -z "$$IP" ] ; then \
		IP=`avahi-browse -ptr  "_arduino._tcp" | egrep ^= | cut -d\; -f4,8,9 | grep ^$$DEVICE | cut -d\; -f2` -p `avahi-browse -ptr  "_arduino._tcp" | egrep ^= | cut -d\; -f4,8,9 | grep ^$$DEVICE | cut -d\; -f3` ;\
	fi ;\
	python $(OTAPROG) -i $(IP) "--auth=$(OTAPASS)" -f $(OBJ)

find:
	@if [ `uname -s` = Darwin ] ; then \
		dns-sd -B _arduino._tcp ;\
	else \
		avahi-browse -ptr  "_arduino._tcp" | egrep ^= | cut -d\; -f4,8,9 ;\
	fi

upload: #$(OBJ)
	python $(ESPTOOL) --port $(PORT) --baud $(BAUD) write_flash 0x10000 $(OBJ)
#	arduino-cli upload -b $(FQBN) -p $(PORT) -i $(OBJ) -v -t
	say "Nah yeah, its all bloody marvellous!"

proxyupload:
	scp $(OBJ) $(PROXYHOST):tmp/$(PROGRAM).ino.bin
	ssh $(PROXYHOST) esptool.py -p $(PROXYPORT) write_flash 0x10000 tmp/$(PROGRAM).ino.bin

dist:
	scp $(OBJ) $(DISTHOST):$(DISTDIR)

erase:
	python $(ESPTOOL) --port $(PORT) erase_flash

fuse:
	$(IDF)/components/esptool_py/esptool/espefuse.py --port $(PORT) set_flash_voltage 3.3V

monitor:
#	cu -s 115200 -l $(PORT)
	miniterm --rts 0 --dtr 0 $(PORT) 115200

go: build upload

gosho: go monitor

increment_build:
	@perl -pi -e '\
          if (/^#define BUILD_NUMBER (\d+)/) {\
            my $$oldbuild=$$1;\
            my $$newbuild=$$oldbuild+1;\
            s/BUILD_NUMBER $$oldbuild/BUILD_NUMBER $$newbuild/;\
          }' config.h
	@grep BUILD_NUMBER config.h

include cli.mk
