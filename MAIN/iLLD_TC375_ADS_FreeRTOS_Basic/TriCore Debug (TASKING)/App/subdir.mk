################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../App/can_handler.c" \
"../App/sensor_driver.c" \
"../App/state_machine.c" \
"../App/tof_sensor.c" \
"../App/ultrasonic_isr.c" 

COMPILED_SRCS += \
"App/can_handler.src" \
"App/sensor_driver.src" \
"App/state_machine.src" \
"App/tof_sensor.src" \
"App/ultrasonic_isr.src" 

C_DEPS += \
"./App/can_handler.d" \
"./App/sensor_driver.d" \
"./App/state_machine.d" \
"./App/tof_sensor.d" \
"./App/ultrasonic_isr.d" 

OBJS += \
"App/can_handler.o" \
"App/sensor_driver.o" \
"App/state_machine.o" \
"App/tof_sensor.o" \
"App/ultrasonic_isr.o" 


# Each subdirectory must supply rules for building sources it contributes
"App/can_handler.src":"../App/can_handler.c" "App/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc37x "-fC:/Users/gkqls/AURIX-v1.10.32-workspace/iLLD_TC375_ADS_FreeRTOS_Basic/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"App/can_handler.o":"App/can_handler.src" "App/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"App/sensor_driver.src":"../App/sensor_driver.c" "App/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc37x "-fC:/Users/gkqls/AURIX-v1.10.32-workspace/iLLD_TC375_ADS_FreeRTOS_Basic/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"App/sensor_driver.o":"App/sensor_driver.src" "App/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"App/state_machine.src":"../App/state_machine.c" "App/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc37x "-fC:/Users/gkqls/AURIX-v1.10.32-workspace/iLLD_TC375_ADS_FreeRTOS_Basic/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"App/state_machine.o":"App/state_machine.src" "App/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"App/tof_sensor.src":"../App/tof_sensor.c" "App/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc37x "-fC:/Users/gkqls/AURIX-v1.10.32-workspace/iLLD_TC375_ADS_FreeRTOS_Basic/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"App/tof_sensor.o":"App/tof_sensor.src" "App/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"App/ultrasonic_isr.src":"../App/ultrasonic_isr.c" "App/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 -D__CPU__=tc37x "-fC:/Users/gkqls/AURIX-v1.10.32-workspace/iLLD_TC375_ADS_FreeRTOS_Basic/TriCore Debug (TASKING)/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"App/ultrasonic_isr.o":"App/ultrasonic_isr.src" "App/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-App

clean-App:
	-$(RM) ./App/can_handler.d ./App/can_handler.o ./App/can_handler.src ./App/sensor_driver.d ./App/sensor_driver.o ./App/sensor_driver.src ./App/state_machine.d ./App/state_machine.o ./App/state_machine.src ./App/tof_sensor.d ./App/tof_sensor.o ./App/tof_sensor.src ./App/ultrasonic_isr.d ./App/ultrasonic_isr.o ./App/ultrasonic_isr.src

.PHONY: clean-App

