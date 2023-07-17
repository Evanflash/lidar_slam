# header
std_msgs/Header header

# 6自由度参数，x y z qx qy qz
float32[] trans_form
# 边缘点
Point[] corner_less_sharp
# 平面点
Point[] surf_less_flat
# 地面点
Point[] ground_less_flat