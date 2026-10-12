CORE_VERSION = 1.15

# Target devices
PARTS = ATxmega32A4U ATxmega64A4U ATxmega128A4U ATxmega128A1U ATxmega16A4U
MCU_ATxmega32A4U = atxmega32a4u
MCU_ATxmega64A4U = atxmega64a4u
MCU_ATxmega128A4U = atxmega128a4u
MCU_ATxmega128A1U = atxmega128a1u
MCU_ATxmega16A4U = atxmega16a4u
LIB_NAME_ATxmega16A4U = ATxmega16A4U_ClockSync
lib_name = $(or $(LIB_NAME_$(1)),$(1))

# Files and directories
BUILD_DIR = bin
SRCS = $(sort $(wildcard *.c))
LIBRARY = $(BUILD_DIR)/lib$(call lib_name,$(1))-$(CORE_VERSION).a
OBJECTS = $(SRCS:%.c=$(BUILD_DIR)/$(1)_%.o)
LIBRARIES = $(foreach part,$(PARTS),$(call LIBRARY,$(part)))
ALL_OBJECTS = $(foreach part,$(PARTS),$(call OBJECTS,$(part)))

# Compiler flags
CC = avr-gcc
CFLAGS = \
-x c \
-funsigned-char \
-funsigned-bitfields \
-DDEBUG \
-O3 \
-ffunction-sections \
-fdata-sections \
-fpack-struct \
-fshort-enums \
-mrelax \
-g2 \
-Wall \
-std=gnu99

# Archiver flags
AR = avr-ar
ARFLAGS = rcs

all: $(LIBRARIES)

clean:
	rm -f $(LIBRARIES)
	rm -f $(ALL_OBJECTS) $(ALL_OBJECTS:.o=.d)

$(BUILD_DIR):
	mkdir "$(BUILD_DIR)"

define PART_RULES
$(BUILD_DIR)/$(1)_%.o: %.c | $(BUILD_DIR)
	$$(CC) $$(CFLAGS) -mmcu=$(MCU_$(1)) -c -MD -MP -MF "$$(@:.o=.d)" -MT"$$@" -o $$@ $$<

$(call LIBRARY,$(1)): $(call OBJECTS,$(1))
	$$(AR) $$(ARFLAGS) $$@ $$^
endef
$(foreach part,$(PARTS),$(eval $(call PART_RULES,$(part))))

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean
