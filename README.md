```bash
echo "deb [trusted=yes] https://github.com/ArendJan/mirte-ros-packages/raw/ros_err_amd64/ ./" | sudo tee /etc/apt/sources.list.d/ArendJan_mirte-ros-packages.list
echo "yaml https://github.com/ArendJan/mirte-ros-packages/raw/ros_err_amd64/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-ArendJan_mirte-ros-packages.list
```
