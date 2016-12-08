# jsk_mbzirc_task3

##controller
Obtain control of M100:  R2 + L2 + Cross

Release control of M100: R2 + L2 + Circle

Release the magnets:  Circle + Triangle

Take Off:   L1 + R1 + Cross

Landing:    L1 + R1 + Circle

Gohome:   L1 + R1 +  Square

Gimbal:   L2 + L and R axis stick

xyz+yaw:  L1 + L and R axis stick

Fast Mode xyz+yaw: L1 + L2 + L and R axis stick (Be careful about this mode)

Virtual RC mode:   press start to enable, then no need for DJI Remote controller, re-press it again to disable Virtual RC mode.
 
Other:  TBC.

##PC setup
###Remote PC
three terminals.

`
sixad -s
`

`
rosrun joy joy_node 
`

`
roslaunch jsk_task3_m100 remotepc.launch
`

for the last one, please open it and set the ip address correctly

###M100 PC
jsk-m100-a-c: 192.168.8.222
jsk-m100-a-v: 192.168.8.111

temporary 4 terminals
`
roslaunch dji_sdk sdk_manifold.launch
`

`
sudo -s
roslaunch dji_sdk_read_cam manifold_cam.launch
`

`
rosrun jsk_task3_m100 uav_teleop_joy
`

`
roslaunch jsk_task3_m100 jsk_udp.launch
`

More details to be added.... Feel free to ask...

















