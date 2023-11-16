#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 17:01:31 2023

@author: lihui.liu
"""

import threading
import time

# class threading.Thread(group=None, target=None, name=None, args=(), kwargs={}, *, daemon=None):
    
class TestThread1(threading.Thread):
    
    def __init__(self,name=None,daemon=None):
        threading.Thread.__init__(self,name=name,daemon=daemon)
        
    def run(self):
        for i in range(30):
            print(threading.current_thread().name + ' test111 ', i)
            time.sleep(1./3.)

class TestThread2(threading.Thread):
    
    def __init__(self,name=None):
        threading.Thread.__init__(self,name=name)
        
    def run(self):
        for i in range(30):
            print(threading.current_thread().name + ' test222222 ', i)
            time.sleep(1./3.)
           
# thread = threading.Thread(target=test1)
thread1 = TestThread1(daemon=True) # daemon : test and main at the same time end
thread2 = TestThread2(name='TestThread')
# thread.setDaemon(True)
thread1.start()
thread2.start()
thread2.join(1.0)
# thread.join()

for i in range(6):
    print(threading.current_thread().name+' main****************',i)
    print(thread1.name+' is alive ',thread1.is_alive())
    time.sleep(1)

print(thread2.name+' is alive ===============> ',thread2.is_alive())


# %%
def test1():
    for i in range(30):
        print(threading.current_thread().name+' test111 ',i)
        time.sleep(1./3.)
def test2():
    for i in range(30):
        print(threading.current_thread().name+' test222222 ',i)
        time.sleep(1./3.)

# thread = threading.Thread(target=test1)
thread1 = threading.Thread(target=test1,daemon=True) # daemon : test and main at the same time end
thread2 = threading.Thread(target=test2,name='TestThread') # daemon : test and main at the same time end
# thread.setDaemon(True)
thread1.start()
thread2.start()
thread2.join(1.0)
# thread.join()

for i in range(6):
    print(threading.current_thread().name+' main****************',i)
    print(thread1.name+' is alive ',thread1.is_alive())
    time.sleep(1)

print(thread2.name+' is alive ',thread2.is_alive())




