def resolv(angle, offset=0):
    while offset - angle > 180:
        angle += 360
    while offset - angle <= -180:
        angle -= 360
    return angle
