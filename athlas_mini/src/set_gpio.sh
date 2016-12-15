#!/bin/bash
#Script to configure GPIO pins

echo -e This script sets the GPIO pins, you need to have wiringPi installed for this

gpio unexportall
#gpio readall
gpio export 11 out
gpio export 15 out
gpio export 12 out #servo vorne links
gpio export 14 out #servo vorne rechts
gpio export 16 out #servo hinten links
gpio export 18 out #servo hinten rechts

gpio readall

echo -e Pins set
