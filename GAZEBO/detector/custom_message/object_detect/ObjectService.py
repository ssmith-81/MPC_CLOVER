import rospy

from math import radians, degrees

from custom_message.srv import ObjectPose, ObjectPoseResponse

from geometry_msgs.msg import Point, PoseStamped, TwistStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from custom_message.msg import ObjectPub

class clover:

	def __init__(self):
		# Creates a node with name 'object_position_server' and make sure it is a
        	# unique node (using anonymous=True).
		rospy.init_node('object_position_server', anonymous=True)
		
		# Subscribe to object state provided by the LiDAR
		self.state = rospy.Subscriber('/Obstacle_state', ObjectPub, self.updateState)

		# Define a ObjectPub() instance for predition horizon dynamics of the object
		self.current_pose = ObjectPub()
		
	# Some callback functions:

	def updateState(self, data):
        # Callback function which is called when a new message of type PoseStamped is
        # received by the subscriber."""
		self.current_pose = data
		

	def update(self,msg):
	# msg is the frame_id which has no influence on the response right now as the object location is 	provided in the map referance frame and is only changes by modifying the vprn launch file.
	
		pos_x = self.current_pose.x
		pos_y = self.current_pose.y
                
		vel_x = self.current_pose.vx
		vel_y = self.current_pose.vy
		af_x = self.current_pose.ax
		af_y = self.current_pose.ay
		

		return ObjectPoseResponse(pos_x, pos_y, vel_x, vel_y,af_x,af_y)

	def object(self):

		# Create the service that takes in a frame_id and returns the pose of the target object
		s = rospy.Service('ObjectPose',ObjectPose, self.update)
		# Keep the server runn until shutdown,
		rospy.spin()


if __name__ == '__main__':
	try:
		x = clover()
		x.object()

	except rospy.ROSInterruptException:
		pass
		



