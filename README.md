# cplusplus_ROS_filewriter
Rather than gathering topics in a rosbag I made this class as a way to :
* get the parameters I desired.
* In the format I desired.

I hope this file can help others use file streams in c++ to write a csv while using ROS .

At this moment I am not including the CMakeList.txt as it works like any another class.
I will most likely post one at a later date as part of another project.


# NOTE
The main logic for the filewriter can be found in this [file](https://github.com/1hada/cplusplus_ROS_filewriter/blob/master/include/diff_drive_controller/gather_to_file.h) 

and at this [line](https://github.com/1hada/cplusplus_ROS_filewriter/blob/master/src/gather_to_file.cpp#L94) everything else was made for a specific use case. 


Enjoy.

