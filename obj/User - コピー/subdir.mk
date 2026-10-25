################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User\ -\ ¥³¥Ô©`/ch32v00x_it.c \
../User\ -\ ¥³¥Ô©`/cv_manager.c \
../User\ -\ ¥³¥Ô©`/dcc_core.c \
../User\ -\ ¥³¥Ô©`/main.c \
../User\ -\ ¥³¥Ô©`/motor_ctrl.c \
../User\ -\ ¥³¥Ô©`/system_ch32v00x.c 

C_DEPS += \
./User\ -\ ¥³¥Ô©`/ch32v00x_it.d \
./User\ -\ ¥³¥Ô©`/cv_manager.d \
./User\ -\ ¥³¥Ô©`/dcc_core.d \
./User\ -\ ¥³¥Ô©`/main.d \
./User\ -\ ¥³¥Ô©`/motor_ctrl.d \
./User\ -\ ¥³¥Ô©`/system_ch32v00x.d 

OBJS += \
./User\ -\ ¥³¥Ô©`/ch32v00x_it.o \
./User\ -\ ¥³¥Ô©`/cv_manager.o \
./User\ -\ ¥³¥Ô©`/dcc_core.o \
./User\ -\ ¥³¥Ô©`/main.o \
./User\ -\ ¥³¥Ô©`/motor_ctrl.o \
./User\ -\ ¥³¥Ô©`/system_ch32v00x.o 

DIR_OBJS += \
./User\ -\ ¥³¥Ô©`/*.o \

DIR_DEPS += \
./User\ -\ ¥³¥Ô©`/*.d \

DIR_EXPANDS += \
./User\ -\ ¥³¥Ô©`/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User\ -\ ¥³¥Ô©`/%.o: ../User\ -\ ¥³¥Ô©`/%.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User\ -\ ¥³¥Ô©`/ch32v00x_it.o: ../User\ -\ ¥³¥Ô©`/ch32v00x_it.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User\ -\ ¥³¥Ô©`/cv_manager.o: ../User\ -\ ¥³¥Ô©`/cv_manager.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User\ -\ ¥³¥Ô©`/dcc_core.o: ../User\ -\ ¥³¥Ô©`/dcc_core.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User\ -\ ¥³¥Ô©`/main.o: ../User\ -\ ¥³¥Ô©`/main.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User\ -\ ¥³¥Ô©`/motor_ctrl.o: ../User\ -\ ¥³¥Ô©`/motor_ctrl.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
User\ -\ ¥³¥Ô©`/system_ch32v00x.o: ../User\ -\ ¥³¥Ô©`/system_ch32v00x.c
	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Debug" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Core" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/User" -I"c:/Users/yuich/OneDrive/mounriver-studio-projects/CH32V003F4P-dcc/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

