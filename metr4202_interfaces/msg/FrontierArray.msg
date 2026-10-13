std_msgs/Header header  # frame_id = "map"
uint32 rows             # number of cluster
uint32 cols             # values per cluster
float32[] data          # row-major; columns: label, size, x, y, distance, utility