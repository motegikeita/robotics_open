#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# 音声コマンド受け渡しプロセス
# Node-REDのexecノードに仕込み、メインプログラムに制御コマンドを送る
# 

from socket import socket, AF_INET, SOCK_STREAM
import sys

COMMANDER_HOST = 'localhost'
COMMANDER_PORT = 51000

def voice_command_send(cmd):

    while True:
        try:
            sock = socket(AF_INET, SOCK_STREAM)
            sock.connect((COMMANDER_HOST, COMMANDER_PORT))
            sock.send(cmd.encode('utf-8'))
            sock.close()
            break
        except:
            print ('Retry: ' + cmd)

if __name__ == '__main__':
    voice_command_send(sys.argv[1])