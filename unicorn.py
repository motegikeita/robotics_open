#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Program for LEGO Robot 5th Unit(Unicorn)
# by Keita Motegi
#
# LEGO Robot consists of LEGO Mindstorms NXT components and Dexter Industries BrickPi.
# Technology depends on:
#
# BrickPi3
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# pyPS4Controller
# https://pypi.org/project/pyPS4Controller/
# By Artur Spirin
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# 
# Node-RED
# https://nodered.org
# Copyright (c) OpenJS Foundation.
#
# Node-RED Alexa Home Skill Bridge
# https://alexa-node-red.bm.hardill.me.uk
# By Ben Hardill
#

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #

import time					# 基本的にはsleep用
import random				# 乱数
import threading			# スレッド制御
import brickpi3				# BrickPi3 ドライバ

from pyPS4Controller.controller import Controller
from socket import socket, AF_INET, SOCK_STREAM

# ソケット通信設定
COMMANDER_HOST   = 'localhost'
COMMANDER_PORT   = 51000
MAX_MESSAGE      = 2048
THREADS          = 4

# BrickPi3クラスのオブジェクト生成
BP = brickpi3.BrickPi3() 

# ロボットクラス
# 
# ロボットの動作を提供
#
class Unicorn:
	
	# Configuration
	
	# モーターレベル
	MIN_SPEED       = 100
	MID_SPEED       = 200
	MAX_SPEED       = 400
	SPEED_INCREMENT = 100

	# モーターアクション時間
	TURN_INTERVAL   = 0.5
	U_TURN_INTERVAL = 2.0
	WALK_INTERVAL   = 0.2
	STOP_INTERVAL   = 0.5
	
	# 操舵範囲
	NXT_MOTOR_RANGE = 30
	LR3_LEVER_RANGE = 32767

	# ステアリング方向
	CENTER = 0
	LEFT   = -1
	RIGHT  = 1
	
	# ステアリング用NXTモーター角度
	TO_CENTER = 0
	TO_LEFT   = NXT_MOTOR_RANGE
	TO_RIGHT  = NXT_MOTOR_RANGE * -1

	# 進行方向の監視範囲（単位：センチメートル）
	MONITOR_RANGE    = 30
	ACCESSIBLE_RANGE = 50
	TOO_NEAR_RANGE   = 20

	# バッテリ容量下限値　これ以下の場合は停止（最大：12）
	LOWEST_BAT_LEVEL = 6.5
	
	# LED制御
	LED_INTERVAL = 0.3
	LED_LEVEL    = 100

	# Globals
	Current_speed = 0
	Last_speed = Current_speed
	Save_speed = 0
	Last_distance_to_obstacle = 0
	Auto_pilot = False
	
	Direction = {-1:'LEFT', 0:'CENTER', 1:'RIGHT'}

	def __init__(self, walk_port, steer_port, eye_port):
		self.drive    = walk_port
		self.steer    = steer_port
		self.eye      = eye_port

		BP.set_sensor_type(self.eye, BP.SENSOR_TYPE.NXT_ULTRASONIC)

	# バッテリー容量は十分か？
	def is_enough_battery(self):
		voltage = BP.get_voltage_battery()
		return True if self.LOWEST_BAT_LEVEL <= voltage else False

	# 前進
	def forward(self):
		BP.set_motor_dps(self.drive, self.Current_speed)
		self.speed_monitor()
		return

	# 後進
	def back(self):
		BP.set_motor_dps(self.drive , self.Current_speed * -1 )
		self.speed_monitor()
		return

	# 加速
	def speed_up(self):
		if (self.Current_speed + self.SPEED_INCREMENT) < self.MAX_SPEED:
			self.Current_speed += self.SPEED_INCREMENT
		else:
			self.Current_speed = self.MAX_SPEED
		return

	# 減速
	def speed_down(self):
		if (self.Current_speed - self.SPEED_INCREMENT) >= 0:
			self.Current_speed -= self.SPEED_INCREMENT
		else:
			self.Current_speed = 0
		return

	# 停止
	def stop(self):
		self.Current_speed = 0
		return

	# ステアリング
	def steering(self, dir = CENTER, lr3_level = LR3_LEVER_RANGE):
		if dir == self.CENTER:
			pos = self.TO_CENTER
		else:
			pos = abs(lr3_level) / self.__LR3_step(self.TO_RIGHT * dir) + self.TO_CENTER
		BP.set_motor_position(self.steer, pos)
		return

	def __LR3_step(self, dir):
		return round(self.LR3_LEVER_RANGE / dir)

	# 左レバーの傾きから速度指示を割り出す
	def get_LR3_speed(self, speed):
		return round(self.LR3_LEVER_RANGE / speed)

	# LED点灯
	def LED_on(self):
		BP.set_led(self.LED_LEVEL)
		time.sleep(self.LED_INTERVAL)
		return

	# LED消灯
	def LED_off(self):
		BP.set_led(0)
		time.sleep(self.LED_INTERVAL)
		return

	# 障害物までの距離をはかる
	def get_distance_to_obstacle(self):
		try:
			return BP.get_sensor(self.eye)
		except brickpi3.SensorError as error:
			print(error)

	# 障害物はあるかチェック
	# オートパイロットの場合はスルー
	def is_detect_obstacle(self):
		if self.Auto_pilot == False:
			return False
		distance = self.get_distance_to_obstacle()
		if distance and self.MONITOR_RANGE >= distance:
			return True
		return False

	# 進行できそうな方向かチェック
	# オートパイロットの場合はスルー
	def is_accessible_direction(self):
		if self.Auto_pilot == False:
			return True
		distance = self.get_distance_to_obstacle()
		if distance and self.ACCESSIBLE_RANGE < distance:
			return True
		return False

	# 進んで、かつモーターが進んだ距離を取得
	def forward_and_get_distance(self, wait_time):
		start = BP.get_motor_encoder(self.drive)
		time.sleep(wait_time)
		end =  BP.get_motor_encoder(self.drive)
		return abs(end - start)

	# バッテリ容量が十分なら、一定時間進んでみる
	def move_and_see(self, move_time):
		if self.is_enough_battery() == False:
			print("Not Enough Battery!")
			return False

		Robot.forward_and_get_distance(move_time)  # 進行距離の評価はいったん無視
		return True

	# 少しきびすを返す
	# オートパイロットの場合はスルー
	def back_away(self):
		if Robot.Auto_pilot == False:
			return

		print('Too much near, evacuation')
		self.speed_monitor(0)
		self.back()
		self.move_and_see(self.U_TURN_INTERVAL)
		return

	# 切り返す
	# オートパイロットの場合はスルー
	def turn_back(self, dir = RIGHT):
		if Robot.Auto_pilot == False:
			return

		# 距離が近すぎる場合はいったん離れる
		distance = self.get_distance_to_obstacle()
		if( self.TOO_NEAR_RANGE > distance):
			self.back_away()

		print('Turn back %s' % self.Direction[dir])
		self.speed_monitor(0)

		# 逆ハンドル
		self.back()
		self.steering(dir * -1)
		self.move_and_see(self.TURN_INTERVAL)
		self.stop()
		time.sleep(self.STOP_INTERVAL)
		
		# 正ハンドル
		self.forward()
		self.steering(dir)
		self.move_and_see(self.TURN_INTERVAL)
		self.steering(self.CENTER)
		return

	# スピードモニター
	def speed_monitor(self, last_speed = None):
		if self.Current_speed != self.Last_speed or last_speed != None:
			print('Current speed is %d' % self.Current_speed)
			self.Last_speed = self.Current_speed
		return

	# 運用終了
	def terminate(self):
		self.stop()
		BP.reset_all()
		print('Robot terminated BYE!')
		exit(1)

# ロボットオブジェクト生成
Robot = Unicorn(BP.PORT_A, BP.PORT_B, BP.PORT_1)

# コントローラクラス
#
# 十字ボタンは速度制御のみで、イベントが立ったらオートパイロットモードへ移行
# 左右レバーはマニュアルモードが選択されたとみなし、イベントが立ったらマニュアルモードへ移行
#
class PS4Controller(Controller):

	def __init__(self, **kwargs):
		Controller.__init__(self, **kwargs)

	# オートパイロット
	
	# 十字ボタン前後に倒すことで定速オートパイロットモードに移行
	#
	def on_up_arrow_press(self):
		Robot.Auto_pilot = True
		Robot.speed_up()
		return

	def on_down_arrow_press(self):
		Robot.Auto_pilot = True
		Robot.speed_down()
		return
	
	# マニュアルモード
	
	# L3レバーによるアクセル操作でマニュアルモードに移行
	#
	def on_L3_up(self,value):
		Robot.Auto_pilot = False
		Robot.Current_speed = value / Robot.get_LR3_speed(Robot.MAX_SPEED * -1)
		return

	def on_L3_down(self,value):
		Robot.Auto_pilot = False
		Robot.Current_speed = value / Robot.get_LR3_speed(Robot.MAX_SPEED * -1)
		return

	def on_L3_at_rest(self):
		Robot.Auto_pilot = False
		Robot.stop();
		return

	def on_L3_press(self):
		Robot.Auto_pilot = False
		Robot.stop();
		return

	# モード共通かつモードの変更を行わない操作
	
	# R3レバーによるステアリング操作
	#
	def on_R3_right(self, value):
		Robot.steering(Robot.RIGHT, value)
		return

	def on_R3_left(self, value):
		Robot.steering(Robot.LEFT, value)
		return

	def on_R3_at_rest(self):
		Robot.steering(Robot.CENTER)
		return

	# モードによらずL1/R1ボタン押下時に適度な低速モードで巡航
	# ボタンを離したら押下前のスピードに復帰
	#
	def on_L1_press(self):
		Robot.Save_speed = Robot.Current_speed
		Robot.Current_speed = Robot.MID_SPEED
		return

	def on_R1_press(self):
		Robot.Save_speed = Robot.Current_speed
		Robot.Current_speed = Robot.MID_SPEED * -1
		return

	def on_L1_release(self):
		Robot.Current_speed = Robot.Save_speed
		return

	def on_R1_release(self):
		Robot.Current_speed = Robot.Save_speed
		return

	# PSボタンはキルスイッチ
	def on_playstation_button_press(self):
		Robot.terminate()
		exit(1)

# コントローラオブジェクト生成
Controller = PS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
Controller.debug = False

#
# ロボットを動かす
#

# 音声コマンド処理スレッド
# Node-REDのexecノードからのメッセージをプロセス間通信で受け取る
# 
def voice_commander():

	# 通信の確立
	sock = socket(AF_INET, SOCK_STREAM)
	sock.bind    ((COMMANDER_HOST, COMMANDER_PORT))
	sock.listen  (THREADS)
	print ('voice commander ready, THREADS = ' + str(THREADS))

	# コマンド受信ループ
	while True:
		try:
			conn, addr = sock.accept()
			cmd        = conn.recv(MAX_MESSAGE).decode('utf-8')
			conn.close()

			print ('Received command:' + cmd)
			
			# 音声制御では原則オートパイロットとする
			Robot.Auto_pilot = True

			if (cmd == 'forward'):
				Robot.Current_speed = Robot.MIN_SPEED

			if (cmd == 'back'):
				Robot.Auto_pilot = False
				Robot.Current_speed = Robot.MID_SPEED * -1
				Robot.move_and_see(Robot.U_TURN_INTERVAL)
				Robot.stop()
				Robot.Auto_pilot = True

			if (cmd == 'hurryup'):
				Robot.speed_up()

			if (cmd == 'slowdown'):
				Robot.speed_down()

			if (cmd == 'stop'):
				Robot.stop()

			if (cmd == 'center'):
				Robot.steering(Robot.CENTER)
		
			if (cmd == 'right'):
				Robot.steering(Robot.RIGHT)

			if (cmd == 'left'):
				Robot.steering(Robot.LEFT)

		except:
			print ('Command Error:' + cmd)

	# 通信の終了
	sock.close()
	print ('end of receiver')

# PS4コントローラ制御スレッド
# コントローラからのイベントを受け取る
# 
def ps4_controller_connect():
	print("controller connected!")	# 現時点では未使用
	
def ps4_controller_disconnect():
	print("controller disconnected")
	Robot.terminate()

def ps4_controller():
	Controller.listen(timeout=60, on_connect=ps4_controller_connect, on_disconnect=ps4_controller_disconnect)

# LEDステータス表示制御スレッド
# BrickPi3本体のステータスLEDを点滅させ、ロボット動作中であることを示す 
# 
def status_show():
	while True:
		Robot.LED_on()
		Robot.LED_off()

# メインスレッド
#
try:

	# サブスレッド起動

	com = threading.Thread(target=voice_commander)
	com.setDaemon(True)
	com.start()
	
	ctr = threading.Thread(target=ps4_controller)
	ctr.setDaemon(True)
	ctr.start()

	led = threading.Thread(target=status_show)
	led.setDaemon(True)
	led.start()

	# イベントループ

	Robot.steering(Robot.CENTER)
	
	while True:

		# 障害物検知
		if Robot.Auto_pilot and Robot.is_detect_obstacle() == True:
			print("Detect obstacle!")
		
			Robot.back()
			Robot.move_and_see(Robot.TURN_INTERVAL)

			# 最初の切り返し方向はランダムに選ぶ
			turn_dir = random.randrange(-1,2,2)
			Robot.turn_back(turn_dir)

			latest_distance = Robot.get_distance_to_obstacle()
			last_distance   = Robot.Last_distance_to_obstacle

			# 初回の切り返しから障害物との距離が長くなればそのまま
			# 短くなれば方向が間違っていると判断して逆方向に切り返す
			if( latest_distance > 0 and latest_distance > last_distance ):
				while Robot.is_accessible_direction() == False:
					Robot.turn_back(turn_dir)
			else:
				while Robot.is_accessible_direction() == False:
					Robot.turn_back(turn_dir * -1)

			Robot.Last_distance_to_obstacle = latest_distance

		# バッテリ容量を見ながらとりあえず進む
		# 進めなくなったらループ離脱
		Robot.forward()
		if Robot.move_and_see(Robot.WALK_INTERVAL) == False:
			break
	
	# イベントループを抜けたら終了
	Robot.terminate()

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
	Robot.terminate()
