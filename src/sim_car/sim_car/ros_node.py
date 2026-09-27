import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
from .core import Simulator, demo_command

class PoseidonNode(Node):
    def __init__(self):
        super().__init__('sim_carulator')
        self.declare_parameter('demo',True)
        self.sim=Simulator(); self.command=(0,0,0); self.received=None
        self.create_subscription(Twist,'cmd_vel',self.command_cb,10)
        self.joints=self.create_publisher(JointState,'joint_states',10)
        self.odom=self.create_publisher(Odometry,'odom',10)
        self.tf=TransformBroadcaster(self)
        self.create_timer(.02,self.tick)
    def command_cb(self,msg):
        self.command=(msg.linear.x,msg.linear.y,msg.angular.z)
        self.received=self.get_clock().now()
    def tick(self):
        now=self.get_clock().now()
        if self.received is not None:
            command=self.command if (now-self.received).nanoseconds<500000000 else (0,0,0)
        else:
            command=demo_command(self.sim.time%32) if self.get_parameter('demo').value else (0,0,0)
        self.sim.step(command,.02); sim=self.sim
        js=JointState(); js.header.stamp=now.to_msg()
        js.name=[n+'_steer_joint' for n in sim.names]+[n+'_wheel_joint' for n in sim.names]
        js.position=sim.angles+sim.spins; self.joints.publish(js)
        tf=TransformStamped(); tf.header.stamp=now.to_msg(); tf.header.frame_id='odom'; tf.child_frame_id='base_footprint'
        tf.transform.translation.x=sim.pose[0]; tf.transform.translation.y=sim.pose[1]
        tf.transform.rotation.z=math.sin(sim.pose[2]/2); tf.transform.rotation.w=math.cos(sim.pose[2]/2)
        self.tf.sendTransform(tf)
        od=Odometry(); od.header=tf.header; od.child_frame_id='base_footprint'
        od.pose.pose.position.x=sim.pose[0]; od.pose.pose.position.y=sim.pose[1]; od.pose.pose.orientation=tf.transform.rotation
        od.twist.twist.linear.x=sim.velocity[0]; od.twist.twist.linear.y=sim.velocity[1]; od.twist.twist.angular.z=sim.velocity[2]
        self.odom.publish(od)

def main():
    rclpy.init(); node=PoseidonNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
