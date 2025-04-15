#motor_thrust_constant and motor_torque_constant represent the proportionality 
#of the controllable value applied to the motor (avg voltage applied by PWM) to the 
#output thrust and torques. Here we roughly assume they are 1.0, but a detailed analysis 
#to provide more accurate values will eventually be necessary

def main():
    file_path = "my_text_file.txt"
    m = 0.1                             # kg
    g = 9.81                            # m/s2
    z = 0.0                             # m
    v = 0.0                             # m/s
    a = 0.0                             # m/s2
    motor_speed = 0.0                   # rpm
    max_motor_speed = 10000             # rpm
    rpm_to_thrust = 0.03*9.81/10000     # kgm/s2/rpm | based on 30g thrust per motor at max speed
    runtime = 100                       # s
    step = 0.001                        # s
    t = 0.0                             # s
    set_z = 20.0                        # m
    p = 1.0                             # pid control parameters
    i = 1.0
    d = 1.0
    
    with open(file_path, "w") as file:
        #begin logging
        file.write("FLIGHT LOG\n")
        file.write("time(ms), height(mm), m1, m2, m3, m4\n")
        while t < runtime:
            #determine acting force
            if z <= 0.0 and m*g >= 4*motor_speed*rpm_to_thrust:
                f = 0.0
            else:
                f = -m*g + 4*motor_speed*rpm_to_thrust
            #determine resulting acceleration
            a = f/m
            #determine resulting velocity
            v += a*step
            #determine resulting position
            z += v*step
            
            #controller section
            error = set_z - z
            
        
            
if __name__ == "__main__":
    main()
    