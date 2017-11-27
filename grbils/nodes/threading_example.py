#!/usr/bin/env python

import threading

def test_thread():

    input("Press enter when you hear the sound.")

def test_func():

    while thread.is_alive():
        pass

    print("Thread is over. User heard a sound.")

thread = threading.Thread(target=test_thread)
thread.daemon = True
thread.start()
test_func()