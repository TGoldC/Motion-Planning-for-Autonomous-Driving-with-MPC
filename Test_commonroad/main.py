# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import numpy as np

A = np.arange(40).reshape(2,20)
print(A)
idx = 0
N = 4
B = np.zeros((2, N))
for i in range(1, N+1):
    B[:, i-1] = A[:, idx + 2*i]

print(B)