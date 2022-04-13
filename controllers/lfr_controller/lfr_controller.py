from controller import Robot

robot = Robot()
max_speed = 3.0

previous_error=I=D=P=error=0
kp= 2.0
ki= 0.1
kd= 0.4

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_list = ['front_right_wheel', 'rear_right_wheel',
              'front_left_wheel', 'rear_left_wheel']
sensor_list = ['ir_right', 'ir_left', 'ir_mid']

motor = dict()
sensor = dict()

for m in motor_list:
    motor[m] = robot.getDevice(m)
    motor[m].setPosition(float('inf'))
    motor[m].setVelocity(0.0)

for s in sensor_list:
    sensor[s] = robot.getDevice(s)
    sensor[s].enable(timestep)
    
def SetMotorSpeed(r_speed, l_speed):
    motor['front_right_wheel'].setVelocity(r_speed)
    motor['rear_right_wheel'].setVelocity(r_speed)
    motor['front_left_wheel'].setVelocity(l_speed)
    motor['rear_left_wheel'].setVelocity(l_speed)

while robot.step(timestep) != -1:
    right_value = sensor['ir_right'].getValue()
    left_value = sensor['ir_left'].getValue()
    mid_value = sensor['ir_mid'].getValue()

    print("Left: {} || Middle: {} || Right: {}".format(left_value,mid_value,right_value))
    
    if right_value < 900 and left_value > 900 and mid_value >= 900:
        error=2.5
    elif right_value > 900 and left_value < 900 and mid_value >= 900:
        error=-2.5
    else:
        error=0
    
    P = error
    I = error + I
    D = error - previous_error
    error_balance = int((kp*P)+(ki*I)+(kd*D))
    previous_error = error
    
    left_speed= max_speed - error_balance
    
    right_speed = max_speed + error_balance
    SetMotorSpeed(right_speed,left_speed)
    pass