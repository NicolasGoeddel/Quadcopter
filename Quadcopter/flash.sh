#/bin/bash

avrdude -px128a1 -cavrisp2 -Pusb -Uflash:w:Release/Quadcopter.hex:a -v
