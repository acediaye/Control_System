import pid

END_TIME = 100
TIME_STEP = 0.1
REFERENCE = 5


if __name__ == '__main__':
    mypid = pid.PID(1, 0, 0, REFERENCE)
    for t in range(0, END_TIME, TIME_STEP):
        pass
        # u = mypid.controller(y, t)
