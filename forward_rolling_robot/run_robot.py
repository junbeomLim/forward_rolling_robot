import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

# mpu6050
# pip install smbus2
# pip install adafruit-circuitpython-mpu6050
Set_mpu6050 = False

if Set_mpu6050:
    import time
    import board
    import busio
    import adafruit_mpu6050
    import math

    # I2C 버스를 설정합니다.
    i2c = busio.I2C(board.SCL, board.SDA)

    # MPU6050 객체를 생성합니다.
    mpu = adafruit_mpu6050.MPU6050(i2c)

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0  # 초기 각도
        self.last_time = time.time()

    def update(self, accel_y, accel_z, gyro_y):
        # 현재 시간과 지난 시간의 차이를 계산합니다.
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 가속도를 이용하여 Y축 각도를 계산합니다.
        accel_angle_y = math.atan2(accel_y, accel_z) * 180 / math.pi

        # 자이로 데이터를 이용하여 Y축 각도의 변화량을 계산합니다.
        gyro_rate_y = gyro_y

        # 자이로 데이터를 이용하여 새로운 각도를 계산합니다.
        self.angle += gyro_rate_y * dt

        # 상보 필터를 적용하여 각도를 보정합니다.
        self.angle = self.alpha * self.angle + (1 - self.alpha) * accel_angle_y

        return self.angle

class PID_Controller():
    def __init__(self, reference, measure, step_time, P_Gain=0.0, I_Gain=0.0 ,D_Gain=0.0):
        self.target_y = reference
        self.y_measure = measure
        self.step_time = step_time
        self.error = reference-measure
        
        self.P_Gain = P_Gain
        self.I_Gain = I_Gain
        self.D_Gain = D_Gain
    
    def ControllerInput(self, reference, measure):
        self.target_y = reference
        self.y_measure = measure
        
        #errorGap
        self.errorGap = (reference-measure) - self.error
        
        #update error
        self.error = reference-measure
        
        self.p_controller = self.P_Gain * self.error
        self.i_controller = self.I_Gain * self.error * self.step_time
        self.d_controller = self.D_Gain * self.errorGap / self.step_time
        self.u = self.p_controller + self.i_controller + self.d_controller
    
    def clear(self):
        self.target_y = 0
        self.y_measure = 0
        self.errorGap = 0
        self.error = 0
        
        self.p_controller = 0
        self.i_controller = 0
        self.d_controller = 0
        self.u = 0

class run_robot():
    def __init__(self):
        self.bldc_speed = 0
    
    def bldc_move(self,speed):
        self.bldc_speed = speed
        # write bldc move code
    
    def PID_leg_move(self,speed,target):
        #write PID move code (both running bldc and leg to balancing)
        self.bldc_move(speed)
        pass
        
    def turn_left(self):
        # write turn left by using leg code
        pass
    
    def turn_right(self):
        # write turn right by using leg code
        pass
    
    def stand(self):
        # write standing code
        pass

    def get_up(self):
        pass


class control_by_keyboard(Node):
    def __init__(self):
        super().__init__('run_robot')
        self.subscription = self.create_subscription(
            Int16,
            'cmd_vel',
            self.listener_callback,
            3)
        self.subscription  # prevent unused variable warning

        self.prev_msg = 0

        # for mpu6050
        if Set_mpu6050:
            self.filter = ComplementaryFilter()
        
        # for control robot 
        self.robot = run_robot()

        step_time = 0.1
        self.controller = PID_Controller(0, 0, step_time, 1.0, 0.0 ,0.001)
        

    def listener_callback(self, msg):
        self.get_logger().info(f"Subscribe {msg.data}")
        
        # mpu 6050
        if Set_mpu6050:
            # 가속도 데이터를 읽습니다.
            accel_x, accel_y, accel_z = mpu.acceleration
            # 자이로 데이터를 읽습니다.
            gyro_x, gyro_y, gyro_z = mpu.gyro

            # 상보 필터를 적용하여 Y축 각도를 계산합니다.
            filtered_angle_y = filter.update(accel_y, accel_z, gyro_y)
        else:
            filtered_angle_y = 0

        measure = filtered_angle_y #degree
        
        if measure < 45 and measure > -45:
            # run robot
            if self.prev_msg != msg.data:
                self.controller.clear()
                self.get_logger().info("clear")
            
            if msg.data != 1 and msg.data != 2:
                self.controller.clear()

            if msg.data == 1 : #forward
                self.controller.ControllerInput(0,measure)
                self.robot.PID_leg_move(100,self.controller.u)
            
            elif msg.data == 2 : #backward
                self.controller.ControllerInput(0,measure)
                self.robot.PID_leg_move(-100,self.controller.u)
            
            elif msg.data == 3 : # left
                self.robot.turn_left()
            
            elif msg.data == 4 : # right
                self.robot.turn_right()

            else:
                self.robot.stand()
        
        # fall down
        else:
            self.controller.clear()
            self.get_logger().info("clear")
            self.robot.get_up()

        self.prev_msg = msg.data 

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = control_by_keyboard()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()