from threading import Thread
from threading import Timer
import time


def Main():

	t1 = time.clock()
	while True:
		t2 = time.clock()
		if t2 - t1 > 5.0:
			break
	print t2 - t1
	
if __name__ == '__main__':
	Main()