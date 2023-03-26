#!/usr/sh

echo "Switching to non secure build dir..."
pushd freertos_GC9A01A_ns/cm33_core0/armgcc
echo "Cleaning..."
sh ./clean.sh
popd
echo "Cleaned!"

echo "Switching to secure build dir..."
pushd freertos_GC9A01A_s/cm33_core0/armgcc
echo "Cleaning..."
sh ./clean.sh
echo "Cleaned!"
popd

