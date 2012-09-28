if [ -d rtt_brics_rn_msgs ]; then
    rm -rf rtt_brics_rn_msgs
fi
rosrun rtt_rosnode create_rtt_msgs brics_rn_msgs
if [ -d rtt_mbn_msgs ]; then
    rm -rf rtt_mbn_msgs
fi
rosrun rtt_rosnode create_rtt_msgs mbn_msgs
rosmake rtt_brics_rn_msgs rtt_mbn_msgs

