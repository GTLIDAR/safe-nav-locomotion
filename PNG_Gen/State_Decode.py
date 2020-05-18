# input state here:

state_input = [1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0]
state_input = [1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1]

i = 0
state = 0
Obs_state = 0
orientation = 0
step_length = 0
if len(state_input) == 26:
    for boul in state_input:
        print 'i = ' + str(i)
        if i == 0:
            Obs_state += boul
            i += 1
        elif i<8:
            Obs_state += boul*(2**(i))
            i += 1
        elif i ==8:
            orientation += boul
            i += 1
        elif i<12:
            orientation += boul*(2**(i-8))
            i += 1
        elif i == 12:
            state += boul
            i += 1
        elif i<20:
            state += boul*(2**(i-12))
            i += 1
        elif i == 20:
            if boul == 1:
                forward = 'forward'
            else:
                forward = 'not forward'
            i += 1
        elif i == 21:
            if boul == 1:
                turn_left = 'turnLeft'
            else:
                turn_left = 'not turnLeft'
            i += 1
        elif i == 22:
            if boul == 1:
                turn_right = 'turnRight'
            else:
                turn_right = 'not turnRight'
            i +=1
        elif i == 23:
            step_length += boul
            i += 1
        elif i<25:
            step_length += boul*(2**(i-23))
            i += 1
        elif i == 25:
            if boul == 1:
                stop = 'stop'
            else:
                stop = 'not stop'
                i += 1

if len(state_input) == 24:
    for boul in state_input:
        print 'i = ' + str(i)
        if i == 0:
            Obs_state += boul
            i += 1
        elif i<7:
            Obs_state += boul*(2**(i))
            i += 1
        elif i ==7:
            orientation += boul
            i += 1
        elif i<11:
            orientation += boul*(2**(i-7))
            i += 1
        elif i == 11:
            state += boul
            i += 1
        elif i<18:
            state += boul*(2**(i-11))
            i += 1
        elif i == 18:
            if boul == 1:
                forward = 'forward'
            else:
                forward = 'not forward'
            i += 1
        elif i == 19:
            if boul == 1:
                turn_left = 'turnLeft'
            else:
                turn_left = 'not turnLeft'
            i += 1
        elif i == 20:
            if boul == 1:
                turn_right = 'turnRight'
            else:
                turn_right = 'not turnRight'
            i +=1
        elif i == 21:
            step_length += boul
            i += 1
        elif i<22:
            step_length += boul*(2**(i-21))
            i += 1
        elif i == 22:
            if boul == 1:
                stop = 'stop'
            else:
                stop = 'not stop'
                i += 1

print 'state: ' + str(state)
print 'Orientation: ' + str(orientation)
print 'Obstacle state: ' + str(Obs_state)
print forward
print turn_left
print turn_right
print 'Step Length: ' + str(step_length)
print stop


