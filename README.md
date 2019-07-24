Kendryte K210 SDK with FreeRTOS
======

This SDK is for Kendryte K210 which contains FreeRTOS support. <br> 
Ported from [Kendryte `kendryte-freertos-sdk`](https://github.com/kendryte/kendryte-freertos-sdk)

## Changes to make the SDK work better with MicroPython, but can be used as SDK for other applications to

```
cd build
cmake ../ -DPROJ=K210_SDK -DTOOLCHAIN=<path_to_kendryte-toolchain>/bin
make 
```

Use `-jN` option for faster build.

The following libraries are created:

```
cmake/third_party/fatfs/libfatfs.a
cmake/third_party/lwip/liblwipcore.a
cmake/SDK/bsp/libbsp.a
cmake/SDK/drivers/libdrivers.a
cmake/SDK/hal/libhal.a
cmake/SDK/posix/libposix.a
cmake/SDK/freertos/libfreertos.a
```

---

_**The following files are modified:**_

* lib/bsp/
  * `printf.c`
  * device/
    * `rtc.cpp`
    * `uart.cpp`
    * `gpiohs.cpp`
* lib/arch/include/
  * `encoding.h`
* lib/drivers/src/storage/
  * `sdcard.cpp`
* lib/utils/include/
  * `syslog.h`
* lib/freertos/
  * include/
    * `task.h`
    * `devices.h`
    * `FreeRTOS.h`
    * `filesystem.h`
    * `portabe.h`
  * kernel/
    * `devices.cpp`
    * storage/
      * `filesystem.cpp`
  * conf/
    * `FreeRTOSConfig.h`
  * portable/
    * `port.c`
    * `portmacro.h`
    * `heap_4.c`
* third_party/
  * lwip/
    * src/
      * `Filelist.cmake`
      * include/
        * netif/
          * ppp/
            * `ppp_opts.h`
  * fatfs/
    * source/
      * `ffconf.h`

