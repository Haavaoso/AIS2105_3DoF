#include <iostream>
#include <fstream>

int main() {
    // Define the file name and path
    std::string filename = "/home/dbjerkas/ros2_ws/src/my_3dof_platform/urdf/3dof_platform.xacro";
    
    // Create and open the file
    std::ofstream file;
    file.open(filename);

    // Check if the file is opened successfully
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1;
    }

    // Write the XML content to the file
    file << R"(
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="3dof_platform">
    <!-- Base Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.5 0.05" />
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1.0" />
            </material>
        </visual>
    </link>

    <!-- Joint 1 (Pitch) -->
    <joint name="joint_pitch" type="continuous">
        <parent link="base_link"/>
        <child link="pitch_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>

    <!-- Pitch Link -->
    <link name="pitch_link">
        <visual>
            <geometry>
                <box size="0.5 0.1 0.05" />
            </geometry>
            <material name="blue">
                <color rgba="0.0 0.0 1.0 1.0" />
            </material>
        </visual>
    </link>

    <!-- Joint 2 (Roll) -->
    <joint name="joint_roll" type="continuous">
        <parent link="pitch_link"/>
        <child link="roll_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <!-- Roll Link -->
    <link name="roll_link">
        <visual>
            <geometry>
                <box size="0.1 0.5 0.05" />
            </geometry>
            <material name="red">
                <color rgba="1.0 0.0 0.0 1.0" />
            </material>
        </visual>
    </link>

    <!-- Joint 3 (Yaw) -->
    <joint name="joint_yaw" type="continuous">
        <parent link="roll_link"/>
        <child link="yaw_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <!-- Yaw Link -->
    <link name="yaw_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.05" />
            </geometry>
            <material name="green">
                <color rgba="0.0 1.0 0.0 1.0" />
            </material>
        </visual>
    </link>

    <!-- Ball -->
    <link name="ball">
        <visual>
            <geometry>
                <sphere radius="0.05" />
            </geometry>
            <material name="white">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
    </link>
    <joint name="joint_ball" type="floating">
        <parent link="yaw_link"/>
        <child link="ball"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </joint>
</robot>
    )";

    // Close the file
    file.close();

    std::cout << "File " << filename << " created successfully." << std::endl;
    return 0;
}
