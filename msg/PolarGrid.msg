# PolarGrid.msg
# 극좌표계 그리드 상의 거리 값 배열

# 메시지가 생성된 시간과 좌표계를 포함하는 헤더
std_msgs/Header header

# 거리(range) 값들을 담는 배열.
# global_to_polar_node는 여기에 1081개의 float 값을 채웁니다.
float32[] ranges