#!/bin/bash

rosrun kalibr kalibr_calibrate_cameras \
--target aprilgrid_4x3_50mm_tagsize.yaml \
--models pinhole-radtan pinhole-radtan \
--topics /vlc_leftfront/hololens_ag0/vlc_image /vlc_rightfront/hololens_ag0/vlc_image \
--bag 2025.03.15_ARL2_4x3_aprilgrid_50mm_tagsize_12.5mm_tagspacing.bag \