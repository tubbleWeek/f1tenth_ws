# Development Guide

I found that the remote server vs-code extension to be very helpful when coding. If you running into issues coding in c++ with your intellisense not being able to find paths you should specify in the c_cpp_properties file in your .vscode folder. Miniconda is also a good resource for python development, however it takes up a lot of space. If your robot has a smaller amount of memory Miniconda is not a good option, as most of your space will be taken up by other packages.

Example:
```
"includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/foxy/include/**",
                "/usr/include/eigen3",
                "${workspaceFolder}/src/costmap_plugins/include",
                "${workspaceFolder}/src/f1tenth_planners/include"
            ]
```

### Creating New Packages

For C++:

```
ros2 pkg create --build-type ament_cmake <package_name>
```

For Python:

```
ros2 pkg create --build-type ament_python <package_name>
```

### Compiling Packages
If you have made changes to the code or parameters, you will need to compile yoour code before you run your nodes. Otherwise, your changes will not be present. You must run these commands in your root ros directory ie in `f1tenth_ws`

For python it is a good idea to run this command in the root of your ROS workspace:

```
rosdep install -i --from-path src --rosdistro foxy -y
```

`foxy` should be replaced by the version of ros that you are using


To compile all packages:

```
colcon build
```

To compile select packages:

```
colcon build --packages-select <package_name>
```

### Other Considerations

If you are planning to use the nav2 stack for controller and planner development, you will need to use C++. There is lots of documentation and tutorials on the nav2 website available for use.

If you are running into issues not being able to run your ros nodes, make sure you have sourced your setup.bash. You can do this by running:

```
source install/setup.bash
```

In the root of your ros workspace.


Sometime you may make changes to your code and compile, but your changes are not present when running the ROS nodes. In this case you should delete your `build`, `install`, and `logs` folders.

```
rm -fr build install logs
```

Then recompile your entire code base with `colcon build`
