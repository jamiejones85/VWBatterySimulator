# VWBatterySimulator
Simulate the modules in VW PHEV battery, temperature, voltage and balancing state. Use for developing/testing BMS controller boards.

Uses an ESP32 and a can transceiver to mock an 8 module pack from a VW PHEV.

It doesn't mock exactly, in that the 0x0BA message to report voltage cannot be repeated to normal modules, it reality
the 2nd byte contains a counter, the at least seem to check the counter is different to the previous message, this is not implemented.

The command to turn on the bleed resistors for balancing also differs, the messages contain a time to be on for, this simulator will take any 0 time value to be on indefinitely and 0 will turn them off. This should be sufficient for now  
