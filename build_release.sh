#!/usr/sh

echo "Switching to non secure build dir..."
pushd freertos_GC9A01A_ns/cm33_core0/armgcc
echo "Building non secure image..."
sh ./build_release.sh
popd
echo "Non secure image built!"

echo "Switching to secure build dir..."
pushd freertos_GC9A01A_s/cm33_core0/armgcc
echo "Building secure image..."
sh ./build_release.sh
echo "Secure image built!"
popd

