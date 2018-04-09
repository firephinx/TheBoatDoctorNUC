## convert compressed raspberry pi cam to uncompressed raw image
rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image/raw
