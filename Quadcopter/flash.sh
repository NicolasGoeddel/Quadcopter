#/bin/bash

avrdude -px128a1u -cavrisp2 -Pusb -Uflash:w:Release/Quadcopter.hex:a -v
