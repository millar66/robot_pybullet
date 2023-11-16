#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 19:35:38 2023

@author: lihui.liu
"""

import threading
from queue import Queue
from multiprocessing import Process, Pipe
import time
import os

class thread_print(threading.Thread):
    
    def __init__(self, print_text, print_content, daemon=None):
        super(thread_print, self).__init__(args=print_content,daemon=daemon)
        self.print_text = print_text
        self.print_content = print_content
        self.__is_running = True
        self.daemon = daemon
        
    def terminate(self):
        self.__is_running = False
        
    def run(self):
        global print_counts
        while self.__is_running:
            # print(f'threading running: {pid} ran: {counts:04d} s')
            print(f'{self.print_text} {print_counts}')
            time.sleep(1)
            
def call_thread():
    # counts = Queue(maxsize=100)
    global print_counts
    print_counts = 0
    thread = thread_print('test_print= ',print_counts,True)
    # thread.daemon = True
    thread.start()
        
    pid = os.getpid()
    for i in range(5):
        print(f'0 call threading pid: {pid} ran: {print_counts:04d} s')
        print_counts += 2
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
    
    