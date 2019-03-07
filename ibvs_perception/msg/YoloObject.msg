# This message describes a 2d bounding box in an image that yolo has detected an object in, along with the tag and certianty.

# Position
string tag #Yolo tag name
float32 score # Score assigned to tag by Yolo
float32 baseX # X-Coordinate for top-left 
float32 baseY # Y-Coordinate for top-left 
float32 width # Width of bounding box
float32 height # Height of bounding box
