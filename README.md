# hipnuc_m5stack

This repo is some tools that collabrate M5Stack and Hipnuc imu module CH100, RTK module HI600

## imu_decode

Decode the data packet of CH100 using M5Stack. Record or process online.

## record_rtcm

Record the RTCM data packet to tf card. The it can be processed in laptop.

## rtcm_decode

Decode the RTCM frame and convert to RINEX. Not finished yet because of M5Stack Basic V2.6 have not enought RAM to decode RTCM3 frame.
