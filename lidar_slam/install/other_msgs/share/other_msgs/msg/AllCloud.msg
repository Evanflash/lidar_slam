# header
std_msgs/Header header


# 6自由度参数，x y z qx qy qz
float32[] trans_form

# corner特征点
Point[] corner_sharp

# 所有边缘点
Point[] corner_less_sharp

# 所有平面点
Point[] surf_less_flat

# 地面特征点
Point[] ground_flat

# 所有地面点
Point[] ground_less_flat