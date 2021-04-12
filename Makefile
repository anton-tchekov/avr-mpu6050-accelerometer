SILENT ?= @
CROSS ?= avr-

MCU ?= atmega328p
F_CPU ?= 16000000

TARGET = main

SRCS = main.c

FEATURE_SET_TIME ?= YES
FEATURE_CHARACTERS ?= YES
FEATURE_CHANGE_TWI_ADDRESS ?= YES
FEATURE_SHOW_ADDRESS_ON_STARTUP ?= YES
FEATURE_LOWERCASE ?= YES

ifeq ($(MCU), attiny4313)
  FEATURE_CHANGE_TWI_ADDRESS ?= YES
  FEATURE_SHOW_ADDRESS_ON_NO_DATA ?= YES
endif

ifneq ($(DEFAULT_BRIGHTNESS), )
  CFLAGS += -DDEFAULT_BRIGHTNESS=$(DEFAULT_BRIGHTNESS)
endif

SPECIAL_DEFS += DEMO \
	FEATURE_SET_TIME \
	FEATURE_CHARACTERS \
	FEATURE_CHANGE_TWI_ADDRESS \
	FEATURE_SHOW_ADDRESS_ON_STARTUP \
	FEATURE_LOWERCASE

OBJS = $(SRCS:.c=.o)

ifneq ($(CROSS), )
  CC = $(CROSS)gcc
  CXX = $(CROSS)g++
  OBJCOPY = $(CROSS)objcopy
  OBJDUMP = $(CROSS)objdump
  SIZE = $(CROSS)size
endif

ifneq ($(F_CPU),)
  CFLAGS += -DF_CPU=$(F_CPU)
endif

define CHECK_ANSWER
  ifeq ($$($(1)), YES)
    CFLAGS += -D$(1)
  endif
endef

$(foreach i,$(SPECIAL_DEFS),$(eval $(call CHECK_ANSWER,$(i))))

OPT=s

CFLAGS += -g -O$(OPT) \
-ffreestanding -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
-Wall -Wstrict-prototypes \
-Wa,-adhlns=$(<:.c=.lst) -std=gnu99 -mmcu=$(MCU) 

all: $(TARGET).elf size

size: $(TARGET).elf
	$(SILENT) $(SIZE) -C --mcu=$(MCU) $(TARGET).elf 

ifneq ($(wildcard $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).eep $(OBJS:%.o=%.d)), )
clean:
	-rm $(wildcard $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).eep $(OBJS:%.o=%.d) $(OBJS:%.o=%.lst))
else
clean:
	@echo "Nothing to clean."
endif

.SECONDARY:

%.elf: $(OBJS)
	@echo "[$(TARGET)] Linking:" $@...
	$(SILENT) $(CC) $(CFLAGS) $(OBJS) --output $@ $(LDFLAGS)

%.o : %.cpp
	@echo "[$(TARGET)] Compiling:" $@... 
	$(SILENT) $(CXX) $(CXXFLAGS) -MMD -MF $(@:%.o=%.d) -c $< -o $@

%.o : %.c
	@echo "[$(TARGET)] Compiling:" $@...
	$(SILENT) $(CC) $(CFLAGS) -MMD -MF $(@:%.o=%.d) -c $< -o $@

%.d : %.cpp
	@echo "[$(TARGET)] Generating dependency:" $@...
	$(SILENT) $(CXX) $(CXXFLAGS) -MM -MT $(addsuffix .o, $(basename $@)) -MF $@ $<

%.d : %.c
	@echo "[$(TARGET)] Generating dependency:" $@...
	$(SILENT) $(CC) $(CFLAGS) -MM -MT $(addsuffix .o, $(basename $@)) -MF $@ $<

PRIOR_OBJS := $(wildcard $(OBJS))
include $(PRIOR_OBJS:%.o=%.d)
