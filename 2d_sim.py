import math
import matplotlib.pyplot as plt

x_cum = 0.0
x_last = 0.0
roll_cum = 0.0
roll_last = 0.0
z_cum = 0.0
z_last = 0.0


def controller(set_x, set_z, x, z, roll, step):
    #some vars are global to allow manipulation from main and controller methods
    global x_cum
    global x_last
    global roll_cum
    global roll_last
    global z_cum
    global z_last
    
    p_x = 0.0
    i_x = 0.0
    d_x = 0.0

    p_roll = 0.0
    i_roll = 0.0
    d_roll = 0.0

    p_thr = 0.0001
    i_thr = 0.0
    d_thr = 0.0

    #motor roll outer loop
    x_err = set_x - x

    #flipping because positive x error should lead to negative roll impulse and vice versa,
    #alternative is having negative PID values, but this seems more appropriate
    x_err *= -1
    
    x_cum += x_err
    roll_1 = p_x*x_err + i_x*x_cum*step + d_x*(x_err - x_last)/step
    x_last = x_err
    
    #motor roll inner loop
    roll_err = roll_1 - roll
    roll_cum += roll_err
    roll_2 = p_roll*roll_err + i_roll*roll_cum*step + d_roll*(roll_err - roll_last)/step
    roll_last = roll_err 
    
    #motor thrust
    z_err = set_z - z
    z_cum += z_err
    thrust = p_thr*z_err + i_thr*z_cum*step + d_thr*(z_err - z_last)/step
    z_last = z_err

    return [roll_2, thrust]

def main():
    file_path = "my_text_file.csv"
    m = 0.1
    I = 0.1
    x = 0.0     #location of drone CoG
    z = 0.0
    g = 9.81
    theta = 0.0
    rpm1 = 0.0
    rpm2 = 0.0
    k = 0.3*9.81/30000  #conversion factor of rpm to thrust
    b = 0.1
    h = 0.005
    t = 0.0
    v_x = 0.0
    v_z = 0.0
    a_x = 0.0
    a_z = 0.0
    alpha = 0.0
    omega = 0.0
    step = 0.001          #1ms steps
    runtime = 300.0     #run for 5min
    del_t = 4.5 #constraint overrun protection prediction time 
    set_z = 20.0
    set_x = 100.0
    theta_t2 = 0.0
    del_rpm = 0.0

    z_plot = []
    x_plot = []
    theta_plot = []
    t_plot = []
    z_error_plot = []
    x_error_plot = []
    rpm1_plot = []
    rpm2_plot = []
    v_z_plot = []
    v_x_plot = []
    pred_theta = []

    with open(file_path, "w") as file:
        #begin logging
        file.write("FLIGHT LOG\n")
        file.write("t, z, x, theta, rpm1, rpm2, v_z, v_x, omega, pred_theta, del_rpm\n")
    
        #simulation based on the idea that over a short time period, 'step', force/acceleration is constant. thus, over this period:
        #force = constant
        #accel = constant
        #velo_f = velo_i + accel*t
        #pos_f = pos_i + velo_i*t + 1/2*accel*t^2
        #also
        #torque = constant
        #rotational accel = const
        #omega_f = omega_i + accel*t
        #theta_f = theta_i + omega_i*t + 1/2*accel*t^2
        while(t < runtime):
            #determine the forces and torques
            #base forces
            f_1 = rpm1*k
            f_2 = rpm2*k
            #z components
            f_1_z = f_1*math.sin(math.radians(90+theta))
            f_2_z = f_2*math.sin(math.radians(90+theta))
            #net z force
            f_z = -m*g + f_1_z + f_2_z
            #x components
            f_1_x = f_1*math.cos(math.radians(90+theta))
            f_2_x = f_2*math.cos(math.radians(90+theta))
            #net x force
            f_x = f_1_x + f_2_x
            #torques
            t_o = f_1*b/2 - f_2*b/2
            
            #determine the resultant acceleration and angular acceleration
            a_x = f_x/m
            a_z = f_z/m
            alpha = t_o/I
            
            #position update (position update before velo update because position is based on initial velo)
            x += (v_x*step * 0.5*a_x*(step**2))
            z += (v_z*step + 0.5*a_z*(step**2))
            theta += (omega*step + 0.5*alpha*(step**2))

            #velocity update
            v_x += a_x*step
            v_z += a_z*step
            omega += alpha*step
            
            #enforcement of terminal velocity, using a simplified model here, assuming v_term = 15.0 
            if v_x > 15.0:
                v_x = 15.0
            elif v_x < -15.0:
                v_x = -15.0

            if v_z > 15.0:
                v_z = 15.0
            elif v_z < -15.0:
                v_z = -15.0

            #enforcement of terminal angular velocity, also a simplified model
            if omega > 20.0:
                omega = 20.0
            elif omega < -20.0:
                omega = -30.0
            
            #save data for plotting
            x_plot.append(x)
            z_plot.append(z)
            theta_plot.append(theta)
            t_plot.append(t)
        
            #write data to logfile
            file.write(f"{t:.3f}, {z:.3f}, {x:.3f}, {theta:.3f}, {rpm1:.3f}, {rpm2:.3f}, {v_z:.2f}, {v_x:.2f}, {omega:.2f}, {theta_t2:.3f}, {del_rpm:.3f}\n")





            #controller action
            output = controller(set_x, set_z, x, z, theta, step)
            rpm1 += (output[0] + output[1])
            rpm2 += (-output[0] + output[1])
        
            
            #rpm saturation enforcement on controller output
            if rpm1 > 30000.0:
                rpm1 = 30000.0
            elif rpm1 < 0.0:
                rpm1 = 0.0

            if rpm2 > 30000.0:
                rpm2 = 30000.0
            elif rpm2 < 0.0:
                rpm2 = 0.0

            #rpm adjustment based on constraint protection 
            #if overrun at t += del_t is predicted, then an adjustment will be made to rpm1, rpm2 considering the value of
            #del_rpm and the physical limits of motor rpm. If no overrun is predicted, del_rpm = 0.0, and nothing happens here.
            #constraint overrun prediction and enforcement
            del_rpm = 0.0
            #predicted value of theta at some future time, t + del_t, assuming controller output is not modified any further over that time period
            theta_t2 = theta + omega*del_t + k*b*(del_t**2)/(2*I)*(rpm1 - rpm2)
            #if overrun in positive theta
            if theta_t2 > 30.0:
                #minimum rpm delta needed to prevent overrun at t +=  t_del
                del_rpm = (30.0 - theta - omega*del_t)*2*I/(k*b*(del_t**2))
                rpm1 += del_rpm/2
                rpm2 -= del_rpm/2
            #if overrun in negative theta
            elif theta_t2 < -30.0:
                del_rpm = (-30.0 - theta - omega*del_t)*2*I/(k*b*(del_t**2))
                rpm1 -= del_rpm/2
                rpm2 += del_rpm/2

            pred_theta.append(theta_t2)
            
            #try to distribute del_rpm
            if rpm1 > 30000.0:
                rpm2 -= (rpm1 - 30000.0)
            elif rpm1 < 0.0:
                rpm2 -= rpm1
            if rpm2 > 30000.0:
                rpm1 -= (rpm2 - 30000.0)
            elif rpm2 < 0.0:
                rpm1 -= rpm2

            #one more rpm saturation enforcement in case del_rpm distribution caused us to overrun:
            if rpm1 > 30000.0:
                rpm1 = 30000.0
            elif rpm1 < 0.0:
                rpm1 = 0.0

            if rpm2 > 30000.0:
                rpm2 = 30000.0
            elif rpm2 < 0.0:
                rpm2 = 0.0


            #time steps forward
            t += step
        
        fig, axs = plt.subplots(3)
        axs[0].plot(t_plot, x_plot)
        axs[0].set_title('x')
        axs[1].plot(t_plot, z_plot)
        axs[1].set_title('z')
        axs[2].plot(t_plot, theta_plot)
        axs[2].set_title('theta')
        fig.show()

        out = input('press any key to quit')

if __name__ == "__main__":
    main()
