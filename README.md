screen_to_world
===============

A small component to communicate to relevant ros nodes the position of faces and salient objects in world coordinates.  Uses measurements from the camera to transform screen coordinates of these objects to world coordinates.

Subscribes to "facedetect" and "/nmpt_saliency_point" for screen coordinate input and publishes to "facedetect_world" and "/nmpt_saliency_point_world" for the standard output.  Also publishes face IDs on the "simple_face_tracker/user_list" topic and broadcasts face transforms for compatibility with other systems.
