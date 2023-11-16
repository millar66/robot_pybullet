# -*- coding: utf-8 -*-
"""
Created on Wed Nov 15 09:43:21 2023

@author: LH
"""

from threading import Thread
from time import sleep, ctime, time
from queue import Queue
from threading import Event

# 创建 Thread 的子类
class Thread_print(Thread):
#    def __init__(self, func, args):
    def __init__(self, print_text, print_content_queue, event):
        '''
        :param func: 可调用的对象
        :param args: 可调用对象的参数
        '''
        Thread.__init__(self)
#        self.func = func
        self.print_text = print_text
        self.print_content_queue = print_content_queue
        self.event = event
#        self.result = None

    def run(self):
#       event = Event()
#       self.result = self.func(*self.args)
#       print_content = self.print_content_queue.get()
        print_text = self.print_text
        print_content = 10000000000
        time_start = time()
        time_end = time()
        while True:
            if not self.print_content_queue.empty():
                print_content = self.print_content_queue.get()
            else:
                sleep(1./20.)
#        for i in range(100):
            # block for a moment
#            print_text = self.print_text_queue.get()
            # check for stop
#            print_content = self.print_content_queue.get_nowait()
            if self.event.is_set():
                # 在此添加退出前要做的工作，如保存文件等
                print('Worker is ended')
                break
            # report a message
            if (time_end - time_start) > 1:
                time_start = time()
                time_end = time()
                print(print_text, print_content, self.print_content_queue.qsize())
            else:
                time_end = time()
        print('Thread is ended')

#    def getResult(self):
#        return self.result

def main():
    # 创建 Thread 实例
    event = Event()
#    print_text_queue = Queue(50)
    print_content_queue = Queue(50)
    # create a thread 
    a = 1700
    print_text = 'hello world'
#    thread = Thread(target=task, args=(event,b))
    thread = Thread_print(print_text, print_content_queue, event)
    # start the new thread
    thread.start()
    print(time())
    for i in range(200):
        a += 100
        if not print_content_queue.full():
            print_content_queue.put(a)
        else:
            print('queue is full')
#        print_text_queue.put(b)
        sleep(1/20)
    print(time())
    # block for a while
    #sleep(5)
    # stop the worker thread
    print('Main stopping thread')
    event.set()
    # 这里是为了演示，实际开发时，主进程有事件循环，耗时函数不需要调用join()方法
    thread.join()
    print("end")

if __name__ == '__main__':
    main()













