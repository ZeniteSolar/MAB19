cd lib/avr-can-lib/src/
make clean
make lib
cd ../../../
wget https://raw.githubusercontent.com/ZeniteSolar/CAN_IDS/master/can_ids.h -O src/can_ids.h
