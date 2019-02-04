#!/bin/bash
killall soccer 2&> /dev/null
killall direct_drive  2&> /dev/null
killall simulator 2&> /dev/null
killall viewer 2&> /dev/null
killall minutebots 2&> /dev/null

soccer_count=$(ps aux | grep soccer | wc -l)
if [ "$soccer_count" -gt "1" ]; then
    echo "Soccer still around, killall -9ing..."
    killall -9 soccer
fi

direct_drive_count=$(ps aux | grep direct_drive | wc -l)
if [ "$direct_drive_count" -gt "1" ]; then
    echo "direct_drive_tester still around, killall -9ing..."
    killall -9 direct_drive
fi

simulator_count=$(ps aux | grep simulator | wc -l)
if [ "$simulator_count" -gt "1" ]; then
    echo "simulator still around, killall -9ing..."
    killall -9 simulator
fi

viewer_count=$(ps aux | grep viewer | wc -l)
if [ "$viewer_count" -gt "1" ]; then
    echo "viewer still around, killall -9ing..."
    killall -9 viewer
fi

minutebots_count=$(ps aux | grep minutebots | wc -l)
if [ "$minutebots_count" -gt "1" ]; then
    echo "minutebots_radio_server still around, killall -9ing..."
    killall -9 minutebots
fi

echo "Complete!";
