import rclpy
import time
from rclpy.node import Node
from can_msgs.msg import Frame
from sensor_msgs.msg import Joy
from autoware_vehicle_msgs.msg import SteeringReport,VelocityReport
from builtin_interfaces.msg import Time
'''
before building source ~/agilex-hunter/autoware/install/setup.bash 
to get autoware messages 
'''


class CANParser(Node):

    def __init__(self):
        super().__init__('can_parser')
		
        self.subscription = self.create_subscription(
			Joy,
			'/joy',
			self.controllerCallback,
			10)
        self.subscription = self.create_subscription(
			Frame,
			'/CAN/can0/receive',
			self.vehicleFeedback,
			10)
        time.sleep(5)
        timer_period=0.02 #20ms 
        self.timer = self.create_timer(timer_period, self.sendCanMessages)
        self.can_publisher = self.create_publisher(Frame, '/CAN/can0/transmit', 10)

        self.VelocityPublisher=self.create_publisher(VelocityReport, '/vehicle/VelocityReport', 10)
        self.SteeringPublisher=self.create_publisher(SteeringReport, '/vehicle/SteeringReport', 10)
        self.initFlag=False 

        self.throttle=0
        self.steering=0
        self.xButton=0


        self.speedFeedback=0
        self.steeringFeedback=0



        #clearing faults 
        self.statusSetting=Frame()
        self.statusSetting.dlc=1
        self.statusSetting.id=0x441
        self.statusSetting.data=[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        
        #put into CAN Control Mode 
        self.CanMode=Frame()
        self.CanMode.dlc=1
        self.CanMode.id=0x421
        self.CanMode.data=[0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        
        #turn off brakes
        self.brakesOff=Frame()
        self.brakesOff.dlc=1
        self.brakesOff.id=0x131
        self.brakesOff.data=[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

        #send Speedand Turning
        self.sendMovment=Frame()
        self.sendMovment.dlc=8
        self.sendMovment.id=0x111
        self.toSendTurn=0
        self.toSendSpeed=0

        
    def vehicleFeedback(self,msg):
        toSendVelocity=VelocityReport()
        toSendSteering=SteeringReport()
        if msg.id==545: #hex 0x221
            payload=msg.data 
            
            self.speedFeedback=(payload[0]<<8)|payload[1]
            if self.speedFeedback>=0x8000:
                self.speedFeedback-=0x10000
            self.speedFeedback/=1000 #in m/s

            self.steeringFeedback=(payload[6]<<8)|payload[7]
            if self.steeringFeedback>=0x8000:
                self.steeringFeedback-=0x10000
            self.steeringFeedback/=1000 #in Rad
            toSendSteering.stamp=self.get_clock().now().to_msg()

            self.get_logger().info("speed FBK: %s m/s |  steer FBK: %s Rad" %(str(self.speedFeedback),str(self.steeringFeedback)))
            toSendVelocity.header.stamp = self.get_clock().now().to_msg()
            toSendVelocity.longitudinal_velocity=self.speedFeedback
            toSendSteering.steering_tire_angle=self.steeringFeedback
            self.SteeringPublisher.publish(toSendSteering)
            self.VelocityPublisher.publish(toSendVelocity)


        
    

    def sendCanMessages(self):

        #sending Init commands on boot
        if not self.initFlag:

            #clearing faults 
            self.can_publisher.publish(self.statusSetting)
    
            #put into CAN Control Mode 
            self.can_publisher.publish(self.CanMode)
            
            #turn off brakes
            self.can_publisher.publish(self.brakesOff)
            self.initFlag=True 
        else:
            self.can_publisher.publish(self.sendMovment)

            


        #self.get_logger().info('looping...')

	

    def controllerCallback(self, msg):
        self.throttle=msg.axes[1]
        self.steering=msg.axes[3]

        maxSpeed=750#software speed limit 
        #max hardware speed value +-1500
        self.toSendSpeed=min(abs(int(self.throttle * (1500))),maxSpeed)
        if self.throttle<0:
            self.toSendSpeed=self.toSendSpeed*-1
            
        #max hardware turn value +-576
        self.toSendTurn=int(self.steering*576)

        lowerSpeedByte=self.toSendSpeed&0xFF
        higherSpeedByte=(self.toSendSpeed>>8)&0xFF

        lowerTurnByte=self.toSendTurn&0xFF
        higherTurnByte=(self.toSendTurn>>8)&0xFF
        self.sendMovment.data=[higherSpeedByte,lowerSpeedByte,0x00,0x00,0x00,0x00,higherTurnByte,lowerTurnByte]
        #self.get_logger().info("speed: %s"%(str(self.toSendSpeed)))
        #self.get_logger().info("turn: %s"%(str(self.toSendTurn)))



        

	

def main(args=None):


	rclpy.init(args=args)
	can_parser = CANParser()
	rclpy.spin(can_parser)
	can_parser.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()


