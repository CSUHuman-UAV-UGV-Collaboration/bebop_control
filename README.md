# bebop_control

## TODO:

* NAVIGATING THE DRONE

* LANDING THE DRONE
* ~~position the camera facing fully down (camera_control topic angular.y)~~
* ~~get the pose of the marker using ar_track_alvar~~
* ~~PART 1: align the drone orientation with the marker (angular velocity, rotation)~~
* ~~convert quaternion (x,y,z,w) to euler (roll, pitch, yaw) if needed~~
* ~~http://wiki.ros.org/tf2/Tutorials/Quaternions~~
* ~~PART 2: align the drone position with the marker (linear velocity, translation)~~
* ~~lower drone and land on the landing pad~~
* improve landing performance
	* ~~implement 'smooth' translation~~
	* implement variable lock zone size
	* flat trim before flight
	* clean bottom camera
	* add more visual features on the landing pad

orb_slam_2_ros notes

https://answers.ros.org/question/282343/working-catkinized-orb-slam-2-or-other-monocular-slam/

Also use Eigen 3.2 instead
