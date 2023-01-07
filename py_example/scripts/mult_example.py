#! /usr/bin/python3

import sys

print('Argument list:')
for arg in sys.argv:
    print(arg)

if len(sys.argv) != 3:
    print('Incorrect number of arguments')
    sys.exit(1)

a = int(sys.argv[1])
b = int(sys.argv[2])

c = a * b
print(f'Result: {c}')
