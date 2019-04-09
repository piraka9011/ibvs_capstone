#!/usr/bin/env python

from geometry_msgs.msg import Point
p = Point()

WIDTH = 640
HEIGHT = 480

i0 = Point(418, 85, 0)
i1 = Point(524, 85, 0)
i2 = Point(418, 357, 0)
i3 = Point(524, 357, 0)

r0 = Point(0, 0, 0)
r1 = Point(0, 0, 1)
r2 = Point(1, 0, 0)
r3 = Point(1, 0, 1)


def project(p, mat):
    x = mat[0][0] * p.x + mat[0][1] * p.y + mat[0][2] * p.z + mat[0][3] * 1
    y = mat[1][0] * p.x + mat[1][1] * p.y + mat[1][2] * p.z + mat[1][3] * 1
    w = mat[3][0] * p.x + mat[3][1] * p.y + mat[3][2] * p.z + mat[3][3] * 1
    return Point(WIDTH * (x / w + 1) / 2., HEIGHT - HEIGHT * (y / w + 1) / 2., 0)


def norm2(a, b):
    dx = b.x - a.x
    dy = b.y - a.y
    return dx * dx + dy * dy


def evaluate(mat):
    c0 = project(r0, mat)
    c1 = project(r1, mat)
    c2 = project(r2, mat)
    c3 = project(r3, mat)
    return norm2(i0, c0) + norm2(i1, c1) + norm2(i2, c2) + norm2(i3, c3)


def perturb(mat, amount):
    from copy import deepcopy
    from random import randrange, uniform
    mat2 = deepcopy(mat)
    mat2[randrange(4)][randrange(4)] += uniform(-amount, amount)
    return mat2


def approximate(mat, amount, n=10000):
    est = evaluate(mat)

    for i in xrange(n):
        mat2 = perturb(mat, amount)
        est2 = evaluate(mat2)
        if est2 < est:
            mat = mat2
            est = est2

    return mat, est


def transpose(m):
    return [
        [m[0][0], m[1][0], m[2][0], m[3][0]],
        [m[0][1], m[1][1], m[2][1], m[3][1]],
        [m[0][2], m[1][2], m[2][2], m[3][2]],
        [m[0][3], m[1][3], m[2][3], m[3][3]],
    ]


if __name__ == '__main__':
    mat = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
    for i in xrange(100):
        mat = approximate(mat, 1)
        mat = approximate(mat, .1)

    print(mat)