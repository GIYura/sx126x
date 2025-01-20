## Radio lib

### Setup

- radio module: EBYTE E22 900M30S (based on sx126x);
- host controller: NUCLEO-F411RE. 

### Connection

### Settings

### Get started

1. Download the project
```
git clone <project path>
```

2. Download GCC toolchain (refer to Resources section)

3. Download STM32 SDK (refer to Resources section)

4. Create local '.config' file based on 'default.config'

5. Edit '.config' to specify **GCC_PATH** and **SDK_PATH**

6. Compile the project:
```
make 
```

7. Clean the project:
```
make clean
```

### Resourses

1. GNU Arm toolchain

https://developer.arm.com/downloads/-/gnu-rm

2. GCC command options

https://gcc.gnu.org/onlinedocs/gcc/index.html#SEC_Contents


3. STM32 SDK

https://github.com/STMicroelectronics/STM32CubeF4


### Reference

1. https://sourceware.org/binutils/docs/ld/Options.html

2. https://www.gnu.org/software/make/manual/ 

