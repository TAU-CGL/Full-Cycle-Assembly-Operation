import random
import time

def majority(arr):
    dict = {}
    for i in arr:
        if i not in dict:
            dict[i] = 1
        else:
            dict[i] += 1

    for i in dict:
        if dict[i] > len(arr)/2:
            return i

    return None

size = int(1e7)

start = time.time()
l = [i for i in range(size)]
end = time.time()
print(majority(l), 'time=', end - start)

def string_to_int(s):
    """
    Converts a string to an integer.
    :param s: characters are 'a'-'e'
    :return: int, the value of the string
    """
    res = 0
    for place in range(len(s)):
        res += (ord(s[place]) - ord('a')) * 5 ** place

    return res

print(string_to_int('aa'))
print(string_to_int('ee'))
