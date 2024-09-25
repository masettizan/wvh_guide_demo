#! /usr/bin/env python3


def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    i, p, d = s.partition('.')
    return '.'.join([i, (d)[:n]])

print(truncate(1.0000000002, 2))
x = round(-1.0000000002, 2)
print(x) 
