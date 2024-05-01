# NAV 2 Behavior Tree Runner

## Build

```bash
. ./setup.bash
colcon build --symlink-install
```

## Run

```bash
# Example 1
ros2 run nav2_bt_pkg nav2_bt_runner

# Example 2
ros2 run nav2_bt_pkg nav2_bt_runner talking_tree.xml
```
