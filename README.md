debris_capture_robo_description  
======================
Gazebo/ROS simulation model for debris capture robot.

Usage 使い方
------  
* Install below dependancy  
`sudo apt-get install ros-hydro-gazebo(4)-ros ros-hydro-gazebo(4)-ros-control ros-hydro-controller-manager ros-hydro-ros-controllers ros-indigo-rqt-ez-publisher`  
* `echo "source /usr/share/gazebo-4.1/setup.sh" > (YOUR SHELL SETTING FILE)`  
* `echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/indigo/lib" > (YOUR SHELL SETTING FILE)`  
* Launch simulator by `roslauch debris_capture_robo_description model_spawn.launch`  
* Launch control board by `roslauch debris_capture_robo_description control_board.launch`  


Parameters パラメータの解説
----------------
N/A

Information 関連情報
--------
N/A

Licences ライセンス
----------
N/A
Copyright &copy; 2011 xxxxxx  
Licensed under the [Apache License, Version 2.0][Apache]  
Distributed under the [MIT License][mit].  
Dual licensed under the [MIT license][MIT] and [GPL license][GPL].  

[Apache]: http://www.apache.org/licenses/LICENSE-2.0  
[MIT]: http://www.opensource.org/licenses/mit-license.php  
[GPL]: http://www.gnu.org/licenses/gpl.html
