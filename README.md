# jsk_mbzirc_task3

##controller
Obtain control of M100:  R2 + L2 + Cross

Release control of M100: R2 + L2 + Circle

Release the magnets:  Circle + Triangle

Take Off:   L1 + R1 + Cross

Landing:    L1 + R1 + Circle

Gohome:   L1 + R1 +  Square

Gimbal:   L2 + L and R axis stick

Gimbal face downward:   L2 + cross

xyz+yaw:  L1 + L and R axis stick

Fast Mode xyz+yaw: L1 + L2 + L and R axis stick (Be careful about this mode)

Virtual RC mode:   press start to enable, then no need for DJI Remote controller, re-press it again to disable Virtual RC mode.
 
Other:  TBC.

##PC setup
### In your own PC, you can either copy and compile the whole package or just use the jsk_network_tools sub-package(under jsk_mbzirc_task3, not the original version under jsk_common), do

`
sixad -s
`

`
rosrun joy joy_node 
`

`
roslaunch jsk_task3_m100 remotepc.launch
`

or just copy the remotepc.launch file to jsk_network_tools(under jsk_mbzirc_task3), and only build jsk_network_tools and launch from there

### Please send you host pc wifi to ip 192.168.97.188 (connect to JSK300)

###M100 PC
jsk-m100-a-c: 192.168.8.222

jsk-m100-a-v: 192.168.8.111

Username for both: ubuntu

Passwd for both: jsk_mbzirc


### ssh into jsk-m100-a-v, do 

`
roslaunch jsk_task3_m100 jsk_udp.launch
`

### ssh into v, then ssh into jsk-m100-a-c, do

`
sudo -s
`

`
roslaunch jsk_task3_m100 jsk_task3_m100.launch
`



next step: write a node to compress the image.

Feel free to ask...

















