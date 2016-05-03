TSC bringup - activities
===============

To start the activities recording action server run:
```
roslaunch tsc_bringup activity_recordings.launch
```

To trigger an action run:

```
rosrun actionlib axclient.py /skeleton_action
```


## Nodes started


The launch file above is equivalent to the following:

* ```roslaunch skeleton_tracker tracker.launch camera:=head_xtion```
* ```rosrun skeleton_tracker skeleton_action.py```
* ```rosrun skeleton_tracker data_deleter.py``` 
* ```rosrun skeleton_tracker data_logger.py``` 


## Webserver node


If activity_recordings.launch is run on a side PC, also run the following node on the main PC (along with the webserver UI):
```
rosrun consent_tsc consent_for_images.py
```
