#! /vendor/bin/sh

# Check for gesture lib to prevent loading
if [ -f /system/lib/libjni_latinimegoogle.so ]; then
    rm /system/lib/libjni_latinimegoogle.so
fi

start tsdriver
