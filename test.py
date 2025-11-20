# test.py -> input manual values for testing callback

# --- Mock ROS2 ---
class MockNode:
    def get_logger(self):
        return self
    def info(self, msg):
        print(msg)

# Mock messages (simplified)
class Point: 
    def __init__(self, x=0, y=0, z=0): self.x, self.y, self.z = x, y, z
class Quaternion: 
    def __init__(self, x=0,y=0,z=0,w=1): self.x,self.y,self.z,self.w=x,y,z,w
class Pose: 
    def __init__(self): self.position = Point(); self.orientation = Quaternion()
class PoseWithCovarianceStamped: 
    def __init__(self): self.pose = type('wrapper', (), {'pose': Pose()})()
class AprilTagDetection: 
    def __init__(self): self.id = []; self.pose = PoseWithCovarianceStamped()
class AprilTagDetectionArray: 
    def __init__(self): self.detections = []

# --- Copy your TagListener but inherit from MockNode ---
class TagListener(MockNode):

    def __init__(self):
        super().__init__()

    def listener_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) == 0:
            self.get_logger().info("No tags detected.")
            return

        det = msg.detections[0]
        pos = det.pose.pose.pose.position

        self.get_logger().info(
            f"Tag ID {det.id[0]} | Position -> x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}"
        )

# --- Build fake message and test ---
node = TagListener()

fake_array = AprilTagDetectionArray()
d = AprilTagDetection()
d.id = [42]

d.pose.pose.pose.position.x = 1.23
d.pose.pose.pose.position.y = -0.45
d.pose.pose.pose.position.z = 0.89

fake_array.detections.append(d)

node.listener_callback(fake_array)
