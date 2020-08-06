################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/74lv8153.c \
../src/board.c \
../src/leiodcat.c \
../src/mcueecfg.c \
../src/modbus.c \
../src/modbussl.c \
../src/powman.c \
../src/spi.c \
../src/timer.c \
../src/usart.c 

S_UPPER_SRCS += \
../src/irqproc.S 

OBJS += \
./src/74lv8153.o \
./src/board.o \
./src/irqproc.o \
./src/leiodcat.o \
./src/mcueecfg.o \
./src/modbus.o \
./src/modbussl.o \
./src/powman.o \
./src/spi.o \
./src/timer.o \
./src/usart.o 

S_UPPER_DEPS += \
./src/irqproc.d 

C_DEPS += \
./src/74lv8153.d \
./src/board.d \
./src/leiodcat.d \
./src/mcueecfg.d \
./src/modbus.d \
./src/modbussl.d \
./src/powman.d \
./src/spi.d \
./src/timer.d \
./src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I../include -DEEPROM_CFG -D_DISABLE_BOARD_AUTOID -DMCUTYPE=AVR -Wall -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atxmega128a1 -DF_CPU=18432000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -mmcu=atxmega128a1 -DF_CPU=18432000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


