#!/bin/bash

sudo openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -c "program ${1} verify reset exit"