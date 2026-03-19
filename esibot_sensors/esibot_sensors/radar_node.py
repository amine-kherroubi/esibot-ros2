#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
# Uncomment for real hardware
import RPi.GPIO as GPIO

class EsibotSensors(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

        # Parameters for testing / hardware
        self.angle_min = 0.0              # radians
        self.angle_max = math.radians(180)
        self.angle_increment = math.radians(10)  # 10° step
        self.range_min = 0.02
        self.range_max = 4.0

        
        # Real hardware setup
        GPIO.setmode(GPIO.BCM)
        self.servo_pin = 17 # servo => GPIO 17
        self.trig_pin = 27 # HC-SR04 TRIG (start measurement)=> GPIO 27
        self.echo_pin = 22 # HC-SR04 ECHO (return measurement time)=> GPIO 22
        GPIO.setup(self.servo_pin, GPIO.OUT)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        self.pwm_servo = GPIO.PWM(self.servo_pin, 50) # create a PWM signal oon the servo pin with 50Hz (50 cycles per second)
        self.pwm_servo.start(0)
        

        # Start the test loop
        self.timer = self.create_timer(1.0, self.publish_scan)  # 1 Hz

    # Temporary test function
    def read_distance(self, angle):
        '''
        # --- TEMPORARY TEST MODE ---
        angle_deg = math.degrees(angle)
        try:
            dist = float(input(f"Enter distance at angle {angle_deg:.1f}° (m): "))
        except:
            dist = self.range_max
        return dist
        '''
        
        # REAL HARDWARE VERSION
        self.set_servo_angle(angle) # rotate servo to angle
        return self.hc_sr04_distance() # measure real distance
        

    
    # Real hardware helper functions
    def set_servo_angle(self, angle):
        angle_deg = math.degrees(angle) # convert to degrees
        duty = 2 + (angle_deg / 18) # convert angle to PWM signal 
        # angle 0 => 180 
        # duty 2 => 12 ; duty cycle => percentage of time the signal is HIGH
        self.pwm_servo.ChangeDutyCycle(duty) # move servo (change how long the signal stays HIGH in each cycle )
        time.sleep(0.1) # wait to stabilize

    def hc_sr04_distance(self):
        GPIO.output(self.trig_pin, False) # reset sensor  
        time.sleep(0.01)
        # send 10micro secondes => start measurement 
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)
        # wait for echo to start 
        while GPIO.input(self.echo_pin) == 0:
            start = time.time()
        # measure echo duration (used to calculate the distance)
        while GPIO.input(self.echo_pin) == 1:
            end = time.time()
        # using a physics formula to calculate the distance
        duration = end - start
        distance = (duration * 343) / 2 # speed of sound = 343 m/s ; devided by 2 => for go + return
        return distance
    
    def publish_scan(self):
        msg = LaserScan() # create LiDAR message 
        msg.header.stamp = self.get_clock().now().to_msg() # take the exact time when data is created 
        msg.header.frame_id = 'laser_link'

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Build the ranges array
        ranges = []
        angle = self.angle_min
        while angle <= self.angle_max: # loop from 0 to 180
            dist = self.read_distance(angle) # get the distance for this angle
            ranges.append(dist) # add it the list 
            angle += self.angle_increment

        msg.ranges = ranges
        self.publisher_.publish(msg) # send data to /scan 
        print("Published LaserScan with ranges:", ranges)


def main(args=None):
    rclpy.init(args=args) # start ROS2 
    node = EsibotSensors() # create node 
    rclpy.spin(node) # keep node running 
    node.destroy_node() # clean and shuttdown 
    rclpy.shutdown()


if __name__ == '__main__':
    main()