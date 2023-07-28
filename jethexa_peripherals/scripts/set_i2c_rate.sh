#!/bin/bash
# set i2c speed
NEW_SPEED=100000
sudo bash -c "echo '$NEW_SPEED' > /sys/bus/i2c/devices/i2c-1/bus_clk_rate"

