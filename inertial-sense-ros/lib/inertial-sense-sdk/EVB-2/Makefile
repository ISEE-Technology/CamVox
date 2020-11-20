TARGET	?= EVB-2

SERIAL_DEVICE ?= /dev/ttyUSB0

DEBUG ?=

#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line

#################################
# Working directories
#################################
ROOT          = .
SRC_DIR       = $(ROOT)/IS_EVB-2/src
ASF_DIR       = $(SRC_DIR)/ASF
SDK_DIR       = ..
HW_LIBS_DIR   = ../hw-libs

OBJECT_DIR    = $(ROOT)/obj
BIN_DIR       = $(ROOT)/obj


#################################
# Source Files
#################################

# Add Source Files
C_SRCS += \
          $(SDK_DIR)/src/ISUtilities.cpp                                                                 \
          $(SDK_DIR)/src/ISFileManager.cpp                                                               \
          $(SDK_DIR)/src/ISLogger.cpp                                                                    \
          $(SDK_DIR)/src/DeviceLog.cpp                                                                   \
          $(SDK_DIR)/src/DeviceLogSerial.cpp                                                             \
          $(SDK_DIR)/src/DataChunk.cpp                                                                   \
          $(SDK_DIR)/src/ISDataMappings.cpp                                                              \
          $(SDK_DIR)/src/ISPose.cpp                                                                      \
          $(SDK_DIR)/src/convert_ins.cpp                                                                 \
          $(SDK_DIR)/src/data_sets.c                                                                     \
          $(SDK_DIR)/src/com_manager.c                                                                   \
          $(SDK_DIR)/src/ISComm.c                                                                        \
          $(ASF_DIR)/sam/utils/syscalls/gcc/syscalls.c                                                   \
          $(ASF_DIR)/common/components/memory/sd_mmc/sd_mmc.c                                            \
          $(ASF_DIR)/common/components/memory/sd_mmc/sd_mmc_mem.c                                        \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/common/source/nm_common.c                  \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/m2m_wifi.c                   \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/bsp/source/nm_bsp_same70.c                 \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/socket/source/socket.c                     \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/m2m_hif.c                    \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmbus.c                      \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmspi.c                      \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/bus_wrapper/source/nm_bus_wrapper_same70.c \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmasic.c                     \
          $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv/driver/source/nmdrv.c                      \
          $(ASF_DIR)/common/services/clock/same70/sysclk.c                                               \
          $(ASF_DIR)/common/services/delay/sam/cycle_counter.c                                           \
          $(ASF_DIR)/common/services/serial/usart_serial.c                                               \
          $(ASF_DIR)/common/services/sleepmgr/sam/sleepmgr.c                                             \
          $(ASF_DIR)/common/services/spi/sam_spi/spi_master.c                                            \
          $(ASF_DIR)/common/services/spi/sam_usart_spi/usart_spi.c                                       \
          $(ASF_DIR)/common/services/storage/ctrl_access/ctrl_access.c                                   \
          $(ASF_DIR)/common/services/usb/class/cdc/device/udi_cdc.c                                      \
          $(ASF_DIR)/common/services/usb/class/cdc/device/udi_cdc_desc.c                                 \
          $(ASF_DIR)/common/services/usb/udc/udc.c                                                       \
          $(ASF_DIR)/common/utils/interrupt/interrupt_sam_nvic.c                                         \
          $(ASF_DIR)/common/utils/stdio/read.c                                                           \
          $(ASF_DIR)/common/utils/stdio/write.c                                                          \
          $(ASF_DIR)/sam/boards/user_board/init.c                                                        \
          $(ASF_DIR)/sam/drivers/efc/efc.c                                                               \
          $(ASF_DIR)/sam/drivers/hsmci/hsmci.c                                                           \
          $(ASF_DIR)/sam/drivers/mcan/mcan.c                                                             \
          $(ASF_DIR)/sam/drivers/mpu/mpu.c                                                               \
          $(ASF_DIR)/sam/drivers/pio/pio.c                                                               \
          $(ASF_DIR)/sam/drivers/pio/pio_handler.c                                                       \
          $(ASF_DIR)/sam/drivers/pmc/pmc.c                                                               \
          $(ASF_DIR)/sam/drivers/pmc/sleep.c                                                             \
          $(ASF_DIR)/sam/drivers/rtc/rtc.c                                                               \
          $(ASF_DIR)/sam/drivers/rtt/rtt.c                                                               \
          $(ASF_DIR)/sam/drivers/spi/spi.c                                                               \
          $(ASF_DIR)/sam/drivers/tc/tc.c                                                                 \
          $(ASF_DIR)/sam/drivers/twihs/twihs.c                                                           \
          $(ASF_DIR)/sam/drivers/uart/uart.c                                                             \
          $(ASF_DIR)/sam/drivers/usart/usart.c                                                           \
          $(ASF_DIR)/sam/drivers/usbhs/usbhs_device.c                                                    \
          $(ASF_DIR)/sam/drivers/wdt/wdt.c                                                               \
          $(ASF_DIR)/sam/drivers/xdmac/xdmac.c                                                           \
          $(ASF_DIR)/sam/services/flash_efc/flash_efc.c                                                  \
          $(ASF_DIR)/sam/utils/cmsis/same70/source/templates/gcc/startup_same70.c                        \
          $(ASF_DIR)/sam/utils/cmsis/same70/source/templates/system_same70.c                             \
          $(ASF_DIR)/thirdparty/fatfs/fatfs-port-r0.09/diskio.c                                          \
          $(ASF_DIR)/thirdparty/fatfs/fatfs-port-r0.09/sam/fattime_rtc.c                                 \
          $(ASF_DIR)/thirdparty/fatfs/fatfs-r0.09/src/ff.c                                               \
          $(ASF_DIR)/thirdparty/fatfs/fatfs-r0.09/src/option/ccsbcs.c                                    \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/croutine.c                               \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/event_groups.c                           \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/list.c                                   \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM7/r0p1/port.c         \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/portable/MemMang/heap_4.c                \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/queue.c                                  \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/stream_buffer.c                          \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/tasks.c                                  \
          $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/timers.c                                 \
          $(HW_LIBS_DIR)/FreeRTOS_new.cpp                                                                \
          $(HW_LIBS_DIR)/drivers/d_flash.c                                                               \
          $(HW_LIBS_DIR)/drivers/d_usartDMA.c                                                            \
          $(HW_LIBS_DIR)/misc/bootloaderApp.c                                                            \
          $(HW_LIBS_DIR)/misc/rtos.c                                                                     \
          $(HW_LIBS_DIR)/printf-master/printf.c                                                          \
          $(SRC_DIR)/CAN.cpp                                                                             \
          $(SRC_DIR)/communications.cpp                                                                  \
          $(SRC_DIR)/control_law.cpp                                                                     \
          $(SRC_DIR)/drivers/d_time.c                                                                    \
          $(SRC_DIR)/drivers/d_quadEnc.c                                                                 \
          $(SRC_DIR)/globals.c                                                                           \
          $(SRC_DIR)/ISLogFileFatFs.cpp                                                                  \
          $(SRC_DIR)/sd_card_logger.cpp                                                                  \
          $(SRC_DIR)/spiTouINS.c                                                                         \
          $(SRC_DIR)/spiTouINS.c                                                                         \
          $(SRC_DIR)/user_interface.cpp                                                                  \
          $(SRC_DIR)/wifi.c                                                                              \
          $(SRC_DIR)/xbee.cpp                                                                            \
          $(SRC_DIR)/main.cpp                                                                            \


# Set Include Directories

INCLUDES +=	$(SRC_DIR) \
            $(SRC_DIR)/config \
            $(SRC_DIR)/drivers 

INCLUDES += $(SDK_DIR)/src

INCLUDES += $(HW_LIBS_DIR)                \
            $(HW_LIBS_DIR)/communications \
            $(HW_LIBS_DIR)/config         \
            $(HW_LIBS_DIR)/drivers        \
            $(HW_LIBS_DIR)/hardware       \
            $(HW_LIBS_DIR)/libusb/include \
            $(HW_LIBS_DIR)/printf-master  

INCLUDES += $(ASF_DIR)/common/boards                                                        \
            $(ASF_DIR)/common/boards/user_board                                             \
            $(ASF_DIR)/common/components/memory/sd_mmc                                      \
            $(ASF_DIR)/common/components/wifi/winc3400/wifi_drv                             \
            $(ASF_DIR)/common/services/clock                                                \
            $(ASF_DIR)/common/services/delay                                                \
            $(ASF_DIR)/common/services/gpio                                                 \
            $(ASF_DIR)/common/services/ioport                                               \
            $(ASF_DIR)/common/services/serial/sam_uart                                      \
            $(ASF_DIR)/common/services/serial                                               \
            $(ASF_DIR)/common/services/sleepmgr                                             \
            $(ASF_DIR)/common/services/spi/sam_spi                                          \
            $(ASF_DIR)/common/services/spi/sam_usart_spi                                    \
            $(ASF_DIR)/common/services/spi                                                  \
            $(ASF_DIR)/common/services/storage/ctrl_access                                  \
            $(ASF_DIR)/common/services/usb/class/cdc/device                                 \
            $(ASF_DIR)/common/services/usb/class/cdc                                        \
            $(ASF_DIR)/common/services/usb/udc                                              \
            $(ASF_DIR)/common/services/usb                                                  \
            $(ASF_DIR)/common/utils/stdio/stdio_serial                                      \
            $(ASF_DIR)/common/utils                                                         \
            $(ASF_DIR)/sam/boards/user_board                                                \
            $(ASF_DIR)/sam/boards                                                           \
            $(ASF_DIR)/sam/drivers/efc                                                      \
            $(ASF_DIR)/sam/drivers/hsmci                                                    \
            $(ASF_DIR)/sam/drivers/mcan                                                     \
            $(ASF_DIR)/sam/drivers/mpu                                                      \
            $(ASF_DIR)/sam/drivers/pio                                                      \
            $(ASF_DIR)/sam/drivers/pmc                                                      \
            $(ASF_DIR)/sam/drivers/rtc                                                      \
            $(ASF_DIR)/sam/drivers/rtt                                                      \
            $(ASF_DIR)/sam/drivers/spi                                                      \
            $(ASF_DIR)/sam/drivers/tc                                                       \
            $(ASF_DIR)/sam/drivers/twihs                                                    \
            $(ASF_DIR)/sam/drivers/uart                                                     \
            $(ASF_DIR)/sam/drivers/usart                                                    \
            $(ASF_DIR)/sam/drivers/usbhs                                                    \
            $(ASF_DIR)/sam/drivers/wdt                                                      \
            $(ASF_DIR)/sam/drivers/xdmac                                                    \
            $(ASF_DIR)/sam/services/flash_efc                                               \
            $(ASF_DIR)/sam/utils/cmsis/same70/include                                       \
            $(ASF_DIR)/sam/utils/cmsis/same70/include/component                             \
            $(ASF_DIR)/sam/utils/cmsis/same70/source/templates                              \
            $(ASF_DIR)/sam/utils/fpu                                                        \
            $(ASF_DIR)/sam/utils/header_files                                               \
            $(ASF_DIR)/sam/utils/preprocessor                                               \
            $(ASF_DIR)/sam/utils                                                            \
            $(ASF_DIR)/thirdparty/CMSIS/Include                                             \
            $(ASF_DIR)/thirdparty/CMSIS/Lib/GCC                                             \
            $(ASF_DIR)/thirdparty/fatfs/fatfs-port-r0.09/sam                                \
            $(ASF_DIR)/thirdparty/fatfs/fatfs-r0.09/src                                     \
            $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/include                   \
            $(ASF_DIR)/thirdparty/freertos/freertos-10.0.0/Source/portable/GCC/ARM_CM7/r0p1



LDSCRIPT=$(ASF_DIR)/sam/utils/linker_scripts/same70/same70q21/gcc/flash.ld

#################################
# Object List
#################################
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET),$(basename $(C_SRCS))))


#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET).hex


#################################
# Flags
#################################
MCFLAGS= -mcpu=cortex-m7 \
        -mthumb \
        -mfloat-abi=hard \
        -mfpu=fpv5-sp-d16


MAP_FLAG = -Wl,-Map="IS_EVB-2.map"

DEFS=   -D__INERTIAL_SENSE_EVB_2__ \
        -D__SAME70Q20B__ \
        -DBOARD=USER_BOARD \
        -DPLATFORM_IS_AVR=false \
        -DARM_MATH_CM7=true \
        -D__FREERTOS__ \
        -DARM \
        -DPOSIX_SHARED \
        -DSD_MMC_ENABLE
        # -DUSE_IS_INTERNAL \


FILE_SIZE_FLAGS=    -ffunction-sections \
 		    -fdata-sections \
		    -fno-exceptions




## BUILD CONFIGURATION
ifeq ($(DEBUG), GDB)
    $(info ***** Building Debug *****)
    DEBUG_FLAGS = -g3
    OPTIMIZE = -O0
    DEFS += -DDEBUG
else
    $(info ***** Building Release *****)
    DEBUG_FLAGS = -g3
    OPTIMIZE = -O2
    LTO_FLAGS =
    DEFS += -DNDEBUG
endif

CFLAGS = -c  $(DEFS)  $(addprefix -I,$(INCLUDES)) $(DEBUG_FLAGS) $(OPTIMIZE) $(FILE_SIZE_FLAGS) $(MCFLAGS) -pipe -fno-strict-aliasing $(WARN_FLAGS)  --param max-inline-insns-single=500

CXXFLAGS = -c $(DEFS) $(addprefix -I,$(INCLUDES)) $(DEBUG_FLAGS) $(OPTIMIZE) $(FILE_SIZE_FLAGS) -fno-rtti $(WARN_FLAGS) $(MCFLAGS) -std=c++11

#LDFLAGS = -mthumb -Wl,-Map="IS_EVB-2.map" --specs=nano.specs -Wl,--start-group  -Wl,--end-group -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--cref -T$(LDSCRIPT) $(MCFLAGS) -u _printf_float $(LTO_FLAGS) -Wl,-Map=main.map
LDFLAGS = -mthumb -Wl,-Map="IS_EVB-2.map" --specs=nano.specs  -Wl,--start-group  -Wl,--end-group -Wl,--gc-sections -Wl,--cref -T$(LDSCRIPT) $(MCFLAGS) -u _printf_float $(LTO_FLAGS)

#################################
# Build
#################################
$(TARGET_HEX): $(TARGET_ELF)
	$(CP) -O ihex $< $@

$(TARGET_ELF): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(BIN_DIR)/$(TARGET)%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(BIN_DIR)/$(TARGET)%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

$(BIN_DIR)/$(TARGET)%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
	rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_HEX) $(BIN_DIR)/output.map

all: $(TARGET_HEX)
