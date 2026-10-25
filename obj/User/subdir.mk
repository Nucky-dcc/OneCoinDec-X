################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v00x_it.c \
../User/cv_manager.c \
../User/dcc_core.c \
../User/main.c \
../User/motor_ctrl.c \
../User/system_ch32v00x.c 

C_DEPS += \
./User/ch32v00x_it.d \
./User/cv_manager.d \
./User/dcc_core.d \
./User/main.d \
./User/motor_ctrl.d \
./User/system_ch32v00x.d 

OBJS += \
./User/ch32v00x_it.o \
./User/cv_manager.o \
./User/dcc_core.o \
./User/main.o \
./User/motor_ctrl.o \
./User/system_ch32v00x.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/OneCoinDec-X/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/OneCoinDec-X/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/OneCoinDec-X/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/OneCoinDec-X/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

