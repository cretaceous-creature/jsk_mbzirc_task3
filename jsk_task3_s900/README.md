- The package for S900 in task3.
-- Usage:
--- UAV side:
```
$ roslaunch jsk_task3_s900 s900_bringup.launch
```

If you want to specify the IP address for remote communication, please add additional parameters like following exmaple:

```
$ roslaunch jsk_task3_s900 s900_bringup.launch LOCAL_IP:=192.168.97.10 REMOTE_IP:=192.168.97.102
```
--- Remote PC side:

$ roslaunch jsk_task3_m100 remotepc.launch
```
roslaunch jsk_task3_m100 remotepc.launch
```

If you want to specify the IP address for remote communication, please add additional paramters too.
