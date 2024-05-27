# cython: language_level=3

from libc.stdlib cimport free
import numpy as cnp

cdef class DWAPath:
    cdef public list x, y, yaw
    cdef public double v, omega, cost

cpdef tuple generate_target_course(list wx, list wy)
