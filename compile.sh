#!/bin/sh
make -C ../linux-2.6.30 ARCH=arm CROSS_COMPILE=/usr/local/arm-2007q1/bin/arm-none-linux-gnueabi- M=`pwd` modules
