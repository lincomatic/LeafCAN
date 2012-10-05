avrdude -c usbtiny -p at90can128  -U lfuse:w:0xFF:m
avrdude -c usbtiny -p at90can128  -U hfuse:w:0x19:m
avrdude -c usbtiny -p at90can128  -U efuse:w:0xFF:m
