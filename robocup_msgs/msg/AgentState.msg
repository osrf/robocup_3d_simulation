float32 temperature # Celsius
float32 battery_percentage

string[] joint_name
float32[] joint_angle_1
float32[] joint_angle_2

string[] gyro_name
string[] gyro_frame
geometry_msgs/Vector3[] gyro_angular
geometry_msgs/Vector3[] gyro_linear

string[] touch_name
string[] touch_frame
bool[] touch_val

string[] force_name
string[] force_frame
geometry_msgs/Vector3[] force_origin
geometry_msgs/Vector3[] force_val

Landmark[] landmarks

Line[] lines
