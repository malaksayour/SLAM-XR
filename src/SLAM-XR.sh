#!/bin/bash

sed -i.bak ~/.config/terminator/config


## Launch a terminator instance using the new layout
terminator -l SLAM-XR

## Return the original config file
mv ~/.config/terminator/config.bak ~/.config/terminator/config