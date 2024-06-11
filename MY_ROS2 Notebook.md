# Chapter1 ROS2 & Environment

Last Update by Jackson Chen on Jun 11

## What is ROS2?

### My Understand

ROS2 is a standardization-oriented software development kit (SDK) that defines various **standard** yet **flexible** communication protocols, which allows a wide range of function modules to be **integrated** on and off eaily with manifold data flow.

### Official Concept:

“ROS 2 is a middleware based on a strongly-typed, anonymous publish/subscribe mechanism that allows for message passing between different processes.” - [ROS 2 Documentation — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/index.html)

## What does ROS2 include?

### 1. Plumbing通信

Middleware: The core of ROS, it’s a message delivering system with many standard interfaces. The interface can also be customized by user. It allows the communication among different modules while it can also saperate moduels to isolate different malfunction, making ROS easy to maintain, increasing the reusability of functions.

### 2. Tools工具

ROS2 provides different tools to help the development process, e.g., launch, debug, visualization, drawing, recording, etc.

### 3. Capabilities功能

ROS2 provides various drivers/interfaces for different sensors/actuators, allowing users to create robots without knowing all the low-level, hardware-related knowledge.

### 4. Communities社区

## ROS2 v.s. ROS

### 1. De-Centralized

No need to have a master node, which may cause the whole system to break down if there’s something wrong with it.

### 2. Bottom Level Communication

ROS2 relies on DDS for bottom level communication, which is much more reliable and faster.

### 3. More Adaptable

Better performance and adaptability for **Swarm Robotics**, small firmware platform.

### 4. More Tools & Libraries

## Hello World!

### C++

#### 1. Create package:

```bash
cd ~/ROS2_ws/src
ros2 pkg create pkg01_helloworld_cpp --build-type ament_cmake --dependencies rclcpp --node-name helloworld
```

#### 2. Edit source_code.cpp

```c++
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv){
  // Initialize ROS2
  rclcpp::init(argc, argv);
  // Create a node
  auto node = rclcpp::Node::make_shared("helloworld_node");
  // Output text
  RCLCPP_INFO(node->get_logger(), "hello world!");
  // Release resrouce
  rclcpp::shutdown();
  return 0;
}
```

#### 3. package.xml

```xml
<?xml version="1.0"?><?xml-model
href="http://download.ros.org/schema/package_format3.xsd"
schematypens="http://www.w3.org/2001/XMLSchema"?><package format="3">
  <name>pkg01_helloworld_cpp</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="your_email@domain.com">ros2</maintainer>
  <license>TODO: License declaration</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- Required Dependencies: Change per your dependencies-->
  <depend>rclcpp</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export></package>
```

#### 4. CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)

project(pkg01_helloworld_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_option(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# import external package (Change per your packages)
find_package(rclcpp REQUIRED)

# map source file and executable (Change per your project)
add_executable(helloworld src/helloworld.cpp)
# set target dependencies (Change per your dependecies)
ament_target_dependencies(
	helloworld
	"rclcpp"
)

# customize installation rule (Change per your rule)
install(TARGETS helloworld
	DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# comment the line when a copyright and license is added to all source files
	set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# 
```



```python

```



# Chapter2 Communication (Core)

# Chapter3 Communication (Ament)

# Chapter4 Launch & Rosbag2

# Chapter5 Coordinate Transformation

# Chapter6 Visualization









