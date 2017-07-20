# curve_compression
Compress points from a curve into keyframes

#dependencies
- Beyond ROS, all you need is Eigen

```sudo apt-get install libeigen3-dev ```


## Compiling

- Copy the present REPO into a catkin workspace, e.g.:

```
cd ~/catkin_ws/src
git clone https://github.com/marcelinomalmeidan/curve_compression.git
```

- Compile the catkin workspace, e.g.:

```
cd ~/catkin_ws
catkin_make
```

## Testing
- Run the service:
```
rosrun curve_compression curveCompression
```

- An example client can be called as:
```
rosrun curve_compression curveCompressionClient
```
