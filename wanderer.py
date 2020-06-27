#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time					# スリープ処理用
import random				# 乱数用
import brickpi3				# BrickPi3 ドライバ

BP = brickpi3.BrickPi3() 	# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# ロボットクラス
class Wanderer:
	
	WALK_SPEED = 350
	TURN_SPEED = 100
	TURN_INTERVAL = 0.1
	U_TURN_INTERVAL = 4.0
	MONITOR_RANGE = 30
	ACCESSIBLE_RANGE = 70
	
	def __init__(self, left_port, right_port, eye_port, bumper_port):
		self.left_leg  = left_port
		self.right_leg = right_port
		self.eye       = eye_port
		self.bumper    = bumper_port
		
		BP.set_sensor_type(self.eye, BP.SENSOR_TYPE.NXT_ULTRASONIC)
		BP.set_sensor_type(self.bumper, BP.SENSOR_TYPE.TOUCH)	

	# 前進
	def forward(self):
		print('go forward')
		BP.set_motor_dps(self.left_leg,  self.WALK_SPEED)
		BP.set_motor_dps(self.right_leg, self.WALK_SPEED)
		return

	# 後進
	def back(self):
		print('go back')
		BP.set_motor_dps(self.left_leg,  self.WALK_SPEED * -1)
		BP.set_motor_dps(self.right_leg, self.WALK_SPEED * -1)
		return

	# ターン direction 1:LEFT -1:RIGHT
	def turn(self, direction):
		print('turn');
		BP.set_motor_dps(self.left_leg,  self.TURN_SPEED * direction)
		BP.set_motor_dps(self.right_leg, self.TURN_SPEED * direction * -1)
		time.sleep(self.TURN_INTERVAL);
		return

	# Uターン
	def u_turn(self):
		print('U turn');
		BP.set_motor_dps(self.left_leg,  self.TURN_SPEED * -1)
		BP.set_motor_dps(self.right_leg,  self.TURN_SPEED )
		time.sleep(self.U_TURN_INTERVAL);
		return

	# 停止
	def stop(self):
		print('stop')
		BP.set_motor_dps(self.left_leg,  0)
		BP.set_motor_dps(self.right_leg, 0)
		return

	# 終了
	def terminate(self):
		self.stop()
		BP.reset_all()
		print('BYE!')
		return

	# 障害物はあるか
	def is_detect_obstacle(self):
		try:
			distance = BP.get_sensor(self.eye)
			if self.MONITOR_RANGE >= distance:
				print("detect obstacle !")
				return True
			return False
		except brickpi3.SensorError as error:
			print(error)
			
	# 進行できそうな方向か？
	def is_accessible_direction(self):
		try:
			distance = BP.get_sensor(self.eye)
			if self.ACCESSIBLE_RANGE < distance:
				print("found accessible direction !")
				return True
			return False
		except brickpi3.SensorError as error:
			print(error)

	# バンパーへの衝突検知
	def is_detect_collision(self):
		try:
			status = BP.get_sensor(self.bumper)
			if status == 1:
				print("Collision!")
				return True
			return False
		except brickpi3.SensorError as error:
			print(error)

# ロボットを動かす
try:
	robot = Wanderer(BP.PORT_A, BP.PORT_B, BP.PORT_1, BP.PORT_2)

	robot.forward()
	while True:
		obstacle  = robot.is_detect_obstacle()
		collision = robot.is_detect_collision()
		
		# 障害物を見つけた場合
		if obstacle == True:
			robot.stop()
			time.sleep(0.3)
			robot.back()
			time.sleep(0.3)

			# 道がひらけるまで切り返し。切り返し方向はランダムで右左選択
			turn_dir = random.randrange(-1,2,2)
			while True:
				if( robot.is_accessible_direction() ):
					break
				robot.turn( turn_dir )
			robot.forward()
		
		# 衝突を検知した場合はUターン
		if collision == True:
			robot.stop()
			time.sleep(0.3)
			robot.back()
			time.sleep(1.5)
			robot.u_turn()
			robot.forward()
	
		time.sleep(0.2)
		
	robot.terminate()

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.

