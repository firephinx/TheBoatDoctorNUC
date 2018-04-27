import numpy

def cam_to_ik(self, cam_coord):
    r_y_90 = numpy.array([ [0, 0, 1], [0, 1, 0], [-1, 0, 0] ])
    return r_y_90.dot(cam_coord)