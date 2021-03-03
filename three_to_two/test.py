import numpy as np
import time


e = 10.6

# List = []
# t0 = time.time()
# List.append(e)
# print(time.time()-t0)
# print(List)

# Array = np.array([])
# t1 = time.time()
# Array = np.append(Array, e)
# print(time.time()-t1)
# print(Array)

# List = [1, 2, 3]
# t0 = time.time()
# List.pop(0)
# print(time.time()-t0)

# List = [1, 2, 3]
# t1 = time.time()
# List = List[1:]
# print(time.time()-t1)

List = [1, 2, 3]
t0 = time.time()
List.pop(1)
print(time.time()-t0)
print(List)

List = [1, 2, 3]
t1 = time.time()
del List[1]
print(time.time()-t1)
print(List)

