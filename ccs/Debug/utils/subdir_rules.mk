################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
utils/cpu_usage.obj: ../utils/cpu_usage.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: TMS470 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M3 -g -O0 --gcc --define=ccs --define=PART_LM3S9B96 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="C:/Users/hqu/Desktop/temp/ccs" --include_path="C:/Users/hqu/Desktop/temp/ccs/lwip" --diag_warning=225 -me --gen_func_subsections --abi=eabi --code_state=16 --ual --preproc_with_compile --preproc_dependency="utils/cpu_usage.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/flash_pb.obj: ../utils/flash_pb.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: TMS470 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M3 -g -O0 --gcc --define=ccs --define=PART_LM3S9B96 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="C:/Users/hqu/Desktop/temp/ccs" --include_path="C:/Users/hqu/Desktop/temp/ccs/lwip" --diag_warning=225 -me --gen_func_subsections --abi=eabi --code_state=16 --ual --preproc_with_compile --preproc_dependency="utils/flash_pb.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/lwiplib.obj: ../utils/lwiplib.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: TMS470 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M3 -g -O0 --gcc --define=ccs --define=PART_LM3S9B96 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="C:/Users/hqu/Desktop/temp/ccs" --include_path="C:/Users/hqu/Desktop/temp/ccs/lwip" --diag_warning=225 -me --gen_func_subsections --abi=eabi --code_state=16 --ual --preproc_with_compile --preproc_dependency="utils/lwiplib.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

utils/sine.obj: ../utils/sine.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: TMS470 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M3 -g -O0 --gcc --define=ccs --define=PART_LM3S9B96 --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --include_path="C:/Users/hqu/Desktop/temp/ccs" --include_path="C:/Users/hqu/Desktop/temp/ccs/lwip" --diag_warning=225 -me --gen_func_subsections --abi=eabi --code_state=16 --ual --preproc_with_compile --preproc_dependency="utils/sine.pp" --obj_directory="utils" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


