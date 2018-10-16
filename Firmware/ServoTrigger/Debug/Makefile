################################################################################
# Makefile compatible with GNU toolchain for avr (http://gnutoolchains.com/avr/)
#
# Build command:
# C:\ServoTrigger\Debug>make all
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../ServoTrigger.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
ServoTrigger.o

OBJS_AS_ARGS +=  \
ServoTrigger.o

C_DEPS +=  \
ServoTrigger.d

C_DEPS_AS_ARGS +=  \
ServoTrigger.d

OUTPUT_FILE_PATH +=ServoTrigger.elf

OUTPUT_FILE_PATH_AS_ARGS +=ServoTrigger.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

#    AVR32/GNU   C   Compiler



./%.o: .././%.c
	@echo  Building file: $<
	@echo  Invoking: AVR/GNU C Compiler : 4.8.1
	$(QUOTE)avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DFSMA=ctpFSM -DFSMB=togglingFSM -DDEBUG  -DSAFERANGE  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -dD -Wall -mmcu=attiny84 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.8.1
	$(QUOTE)avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="ServoTrigger.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=attiny84  
	@echo Finished building target: $@
	"avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "ServoTrigger.elf" "ServoTrigger.hex"
	"avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "ServoTrigger.elf" "ServoTrigger.eep" || exit 0
	"avr-objdump.exe" -h -S "ServoTrigger.elf" > "ServoTrigger.lss"
	"avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "ServoTrigger.elf" "ServoTrigger.srec"
	"avr-size.exe" "ServoTrigger.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "ServoTrigger.elf" "ServoTrigger.a" "ServoTrigger.hex" "ServoTrigger.lss" "ServoTrigger.eep" "ServoTrigger.map" "ServoTrigger.srec" "ServoTrigger.usersignatures"
	