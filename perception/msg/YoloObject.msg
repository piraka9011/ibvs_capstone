# This message describes a 2d bounding box in an image that yolo has detected an object in, along with the tag and certianty.

# Position
std_msgs/String tag #Yolo tag name
std_msgs/Float32 score # Score assigned to tag by Yolo
std_msgs/Float32 baseX # X-Coordinate for top-left 
std_msgs/Float32 baseY # Y-Coordinate for top-left 
std_msgs/Float32 width # Width of bounding box
std_msgs/Float32 height # Height of bounding box
