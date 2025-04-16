import matplotlib.pyplot as plt

def main():
    file_path = "my_text_file.txt"
    m = 0.1                             # kg
    g = 9.81                            # m/s2
    z = 0.0                             # m
    v = 0.0                             # m/s
    v_term = 12                         # m/s
    a = 0.0                             # m/s2
    motor_speed = 0.0                   # rpm
    max_motor_speed = 10000             # rpm
    rpm_to_thrust = 0.03*9.81/10000     # kgm/s2/rpm | based on 30g thrust per motor at max speed
    runtime = 100                       # s
    step = 0.001                        # s
    t = 0.0                             # s
    set_z = 20.0                        # m
    p = 0.1                             # pid control parameters
    i = 0.01
    d = 50.0
    error = 0.0
    cum_error = 0.0

    t_plot = []
    z_plot = []
    with open(file_path, "w") as file:
        #begin logging
        file.write("FLIGHT LOG\n")
        file.write("time(ms), height(mm), motor speed (rpm), net force (N), velocity (m/s) total error\n")
        while t < runtime:
            #determine acting force
            f = -m*g + 4*motor_speed*rpm_to_thrust
            #determine resulting acceleration
            a = f/m
            #determine resulting velocity
            v += a*step
            if v > v_term:
                v = v_term
            elif v < -v_term:
                v = -v_term
            #determine resulting position
            z += v*step
                        
            #controller section
            last_error = error
            cum_error += error*step
            error = (set_z - z)/step
            total_error =  p*error + i*cum_error + d*(error-last_error)
            #motor speed update
            motor_speed += total_error            
            #ensure motor speed is within limits
            if motor_speed >= max_motor_speed:
                motor_speed = max_motor_speed
            elif motor_speed <= 0:
                motor_speed = 0
            #logging
            file.write(f"{t}, {z}, {motor_speed}, {f}, {v}, {total_error}\n")
            
            t_plot.append(t)
            z_plot.append(z)
            t += step
            
    plt.plot(t_plot, z_plot)
    plt.xlim(0, 50)
    plt.ylim(-5, 30)
    plt.show()
        
            
if __name__ == "__main__":
    main()
    
