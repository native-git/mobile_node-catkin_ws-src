from multiprocessing import Queue
from Queue import *

q = Queue(maxsize=3)


q.put(1)
q.put(2)
q.put(3)

one = q.get()
print "one"+str(one)

two = q.get()
print "two"+str(two)

three = q.get()
print "three"+str(three)