# Prerequisites

For the Emergence Project, we decided to leverage Open-Source tools whenever it's possible.

## GNU Arm Embedded Toolchain

The **GNU Embedded Toolchain** for Arm is a ready-to-use, open source suite of tools for C, C++ and Assembly programming targeting Arm Cortex-M and Cortex-R family of processors. It includes the GNU Compiler (GCC) and is available free of charge directly from Arm for embedded software development on Windows, Linux and Mac OS X operating systems.

The reference version for Emergence Project is **Version 7-2017-q4-major** (Released: *December 18, 2017*).

### Download

Download the toolchain for [Mac OS X 64-bit](https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-mac.tar.bz2?revision=7f453378-b2c3-4c0d-8eab-e7d5db8ea32e?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Mac%20OS%20X,7-2017-q4-major) platform. For other platforms, refer to [Arm Developer website](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads).

### Installation

Move the downloaded package in the *Toolchains* directory and uncompress it. *Make* will automatically use the installed toolchain if the version matches. Binaries have to be in *Toolchains/<version\>/bin*. From the project root, you can verify installation correctness with:
```
>> ls Toolchains/gcc-arm-none-eabi-7-2017-q4-major/bin/arm-none-eabi-*
```

## ST-LINK Utilities

The Open-Source version of **ST-LINK Utilities** - [stlink](https://github.com/texane/stlink) developed by [texane](https://github.com/texane) - is a great alternative to STMicroelectronics tools and running on Mac OS X.

### Installation

I recommend to use *brew* for the installation:
```
>> brew install stlink
```
If you need a custom installation, refer to [stlink](https://github.com/texane/stlink) documentation in GitHub.

# Getting Started

Once the Toolchain and ST-LINK Utilities have been installed, you are now ready to *compile* and *flash* Emergence firmware on your Nucleo-F767ZI.

## Compilation

From the project root, simply run *make* to compile Emergence firmware:
```
>> make
```

## Flashing

Then use *st-flash* to flash the binary on your board:
```
>> st-flash write build/Emergence.bin 0x8000000
```
