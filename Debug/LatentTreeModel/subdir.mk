################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LatentTreeModel/BuildNJTree.cpp \
../LatentTreeModel/CLNJ.cpp \
../LatentTreeModel/MeanGeodesicDistances.cpp \
../LatentTreeModel/MinimalSpanningTree.cpp \
../LatentTreeModel/NeighbourJoining.cpp 

OBJS += \
./LatentTreeModel/BuildNJTree.o \
./LatentTreeModel/CLNJ.o \
./LatentTreeModel/MeanGeodesicDistances.o \
./LatentTreeModel/MinimalSpanningTree.o \
./LatentTreeModel/NeighbourJoining.o 

CPP_DEPS += \
./LatentTreeModel/BuildNJTree.d \
./LatentTreeModel/CLNJ.d \
./LatentTreeModel/MeanGeodesicDistances.d \
./LatentTreeModel/MinimalSpanningTree.d \
./LatentTreeModel/NeighbourJoining.d 


# Each subdirectory must supply rules for building sources it contributes
LatentTreeModel/%.o: ../LatentTreeModel/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


