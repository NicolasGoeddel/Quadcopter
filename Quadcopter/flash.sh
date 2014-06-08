#/bin/bash

avrdude -px128a1 -cavrisp2 -Pusb -Uflash:w:Release/Quadcopter_Test2.hex:a -v
