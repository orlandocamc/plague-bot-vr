# A single YOLO Detection on a plant, with a 3D position reprojected from the
# D435 depth point cloud. See SPEC §6.3 / CONTEXT "Detection".
string class_name              # YOLO class label (e.g. "pest", "disease")
float32 confidence             # detection score in [0, 1]
int32[4] bbox                  # image-space box: [x_min, y_min, x_max, y_max] in px
geometry_msgs/PointStamped position   # 3D centroid (frame_id set by perception node)
