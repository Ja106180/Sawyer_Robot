# PersonKeypoints.msg
# 一个人体的关键点数据

std_msgs/Header header
uint32 person_id          # 人员ID
float32 bbox_x            # 检测框左上角x
float32 bbox_y            # 检测框左上角y
float32 bbox_width        # 检测框宽度
float32 bbox_height       # 检测框高度
float32 bbox_area         # 检测框面积
float32 bbox_confidence   # 检测框置信度（YOLO显示的置信度）

# 右手关键点（COCO格式中通常是右手腕，索引10）
float32 right_wrist_x     # 右手腕x坐标
float32 right_wrist_y     # 右手腕y坐标
float32 right_wrist_conf  # 右手腕置信度

# 右手肘（索引8）
float32 right_elbow_x
float32 right_elbow_y
float32 right_elbow_conf

# 右肩膀（用于判断右手是否伸出）
float32 right_shoulder_x
float32 right_shoulder_y
float32 right_shoulder_conf

# 图像尺寸（用于计算角度）
uint32 image_width
uint32 image_height

