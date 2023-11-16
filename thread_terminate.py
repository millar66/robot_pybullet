#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 18:50:00 2023

@author: lihui.liu
"""

import threading
import time
import os

def print_thread():
    pid = os.getpid()
    counts = 0
    while True:
        print(f'threading pid: {pid} ran: {counts:04d} s')
        counts += 1
        time.sleep(1)
        
class StoppableThread(threading.Thread):
    
    def __init__(self, daemon=None):
        super(StoppableThread, self).__init__(daemon=daemon)
        self.__is_running = True
        self.daemon = daemon
        
    def terminate(self):
        self.__is_running = False
        
    def run(self):
        pid = os.getpid()
        counts = 0
        while self.__is_running:
            print(f'threading running: {pid} ran: {counts:04d} s')
            counts += 1
            time.sleep(1)
            
def call_thread():
    thread = StoppableThread()
    thread.daemon = True
    thread.start()
        
    pid = os.getpid()
    counts = 0
    for i in range(5):
        print(f'0 call threading pid: {pid} ran: {counts:04d} s')
        counts += 2
        time.sleep(2)
    thread.terminate()
        
if __name__ == '__main__':
    call_thread()
    print(f'========call_thread finish========')
    counts = 0
    for i in range(5):
        counts += 1
        time.sleep(1)
        print(f'main thread: {counts:04d} s')
    
    
    