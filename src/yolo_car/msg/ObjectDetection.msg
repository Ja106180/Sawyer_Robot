# ObjectDetection.msg
# 单个物体的检测结果

std_msgs/Header header
uint32 object_id          # 物体ID (1, 2, 3...)
float32 center_x          # 中心点x坐标（像素）
float32 center_y          # 中心点y坐标（像素）
float32 confidence        # 检测置信度
float32 bbox_x            # 边界框左上角x
float32 bbox_y            # 边界框左上角y
float32 bbox_width        # 边界框宽度
float32 bbox_height        # 边界框高度
uint32 image_width        # 图像宽度
uint32 image_height       # 图像高度

