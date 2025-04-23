import math
import matplotlib.pyplot as plt
import classes

x_cum = 0.0
x_last = 0.0
x_err = 0.0
roll_cum = 0.0
roll_last = 0.0
roll_err = 0.0
z_cum = 0.0
z_last = 0.0
z_err = 0.0
roll_1 = 0.0


def controller(set_x, set_z, drone, step, del_t, t):
    #some vars are global to allow manipulation from main and controller methods
    global x_cum
    global x_last
    global x_err
    global roll_cum
    global roll_last
    global roll_err
    global z_cum
    global z_last
    global z_err
    global roll_1
    
    inner_loop_rr = 1       #how many steps between updates of the inner loop?
    outer_loop_rr = 4       #how many steps between updates of the outer loop?
    
    p_x = 1.0
    i_x = 300.0
    d_x = 0.05

    p_roll = 0.001
    i_roll = 0.0
    d_roll = 0.011

    p_thr = 0.01
    i_thr = 0.0
    d_thr = 0.5
    
    #if t is a multiple of outer_loop_rr*step
    if t%(outer_loop_rr*step) == 0:
        #motor roll outer loop
        x_err = set_x - drone.pos[0]

        #flipping because positive x error should lead to negative roll impulse and vice versa,
        #alternative is having negative PID values, but this seems more appropriate
        x_err *= -1
        
        x_cum += x_err
        roll_1 = p_x*x_err + i_x*x_cum*step + d_x*(x_err - x_last)/step
        x_last = x_err
        #saturation filter for roll set point
        if roll_1 > 30.0:
            roll_1 = 30.0
        elif roll_1 < -30.0:
            roll_1 = -30.0

    #motor roll inner loop
    roll_err = roll_1  - drone.rot[0]
    roll_cum += roll_err
    roll_2 = p_roll*roll_err + i_roll*roll_cum*step + d_roll*(roll_err - roll_last)/step
    roll_last = roll_err 

    #motor thrust
    z_err = set_z - drone.pos[1]
    z_cum += z_err
    thrust = p_thr*z_err + i_thr*z_cum*step + d_thr*(z_err - z_last)/step
    z_last = z_err

    drone.motor_output[0] += (roll_2 + thrust)
    drone.motor_output[1] += (-roll_2 + thrust)

    #rpm saturation enforcement on controller output
    if drone.motor_output[0] > 30000.0:
        drone.motor_output[0] = 30000.0
    elif drone.motor_output[0] < 0.0:
        drone.motor_output[0] = 0.0

    if drone.motor_output[1] > 30000.0:
        drone.motor_output[1] = 30000.0
    elif drone.motor_output[1] < 0.0:
        drone.motor_output[1] = 0.0

    #rpm adjustment based on constraint protection 
    #if overrun at t += del_t is predicted, then an adjustment will be made to rpm1, rpm2 considering the value of
    #del_rpm and the physical limits of motor rpm. If no overrun is predicted, del_rpm = 0.0, and nothing happens here.
    #constraint overrun prediction and enforcement
    del_rpm = 0.0
    #predicted value of theta at some future time, t + del_t, assuming controller output is not modified any further over that time period
    theta_t2 = drone.rot[0] + drone.omega[0]*del_t + drone.k*drone.b*(del_t**2)/(2*drone.I)*((drone.motor_output[0]**2) - (drone.motor_output[1]**2))
    #if overrun in positive theta
    if theta_t2 > 30.0:
        #minimum rpm delta needed to prevent overrun at t +=  t_del
        del_rpm = (30.0 - drone.rot[0] - drone.omega[0]*del_t)*2*drone.I/(drone.k*drone.b*(del_t**2))
        drone.motor_output[1] = 30000
        drone.motor_output[0] = math.sqrt(del_rpm + (30000**2))
    #if overrun in negative theta
    elif theta_t2 < -30.0:
        del_rpm = (-30.0 - drone.rot[0] - drone.omega[0]*del_t)*2*drone.I/(drone.k*drone.b*(del_t**2))
        drone.motor_output[0] = 30000
        drone.motor_output[1] = math.sqrt((30000**2) - del_rpm)

    #pred_theta.append(theta_t2)
    
    #!!! no longer necessary, this is now done in the if else if block above
    #try to distribute del_rpm
    """
    if drone.motor_output[0] > 30000.0:
        drone.motor_output[1] -= (drone.motor_output[0] - 30000.0)
    elif drone.motor_output[0] < 0.0:
        drone.motor_output[1] -= drone.motor_output[0]
    if drone.motor_output[1] > 30000.0:
        drone.motor_output[0] -= (drone.motor_output[1] - 30000.0)
    elif drone.motor_output[1] < 0.0:
        drone.motor_output[0] -= drone.motor_output[1]
    """

    #one more rpm saturation enforcement in case del_rpm distribution caused us to overrun:
    if drone.motor_output[0] > 30000.0:
        drone.motor_output[0] = 30000.0
    elif drone.motor_output[0] < 0.0:
        drone.motor_output[0] = 0.0

    if drone.motor_output[1] > 30000.0:
        drone.motor_output[1] = 30000.0
    elif drone.motor_output[1] < 0.0:
        drone.motor_output[1] = 0.0
   
# In the refactoring of this code, I would like the main method to be responsible only for advancing time,
# and the modules (drone physics sim, controller, etc.) should be responsible for updating themselves
# Because the modules don't always update at the same time, the main method should also be responsible
# for calling the modules to update themselves at the appropriate intervals and supplying the duration
# of the interval which the update is to cover
def main():
    #simulation variables
    t = 0.0
    runtime = 300.0
    global_step = 0.001         #global time resolution is 1ms
    controller_step = 0.001     #controller updates at 1000Hz, same as global
    del_t = 0.5
    file_path = "my_text_file.csv"
    
    #variables for plotting
    z_plot = []
    x_plot = []
    theta_plot = []
    t_plot = []
    
    #create drone object
    drone = classes.Drone_2D(0.1, 0.1, 0.1, 0.005, 0.3*9.81/30000)
    drone.pos[0] = 0.5              #slight error in initial x value
    drone.rot[0] = 3.0              #slight error in initial theta value
    motor_output = [0.0, 0.0]
    rpm1 = motor_output[0]
    rpm2 = motor_output[1]
    
    #create controller object
    #...for now, just use existing functional controller, class-based controller in the works...
    
    with open(file_path, "w") as file:
        #begin logging
        file.write("FLIGHT LOG\n")
        file.write("t, z, x, theta, rpm1, rpm2, v_z, v_x, omega, pred_theta, del_rpm\n")
        while(t < runtime):
            #update physics
            drone.update(global_step)
            
            #logging
            #file.write(f"{t:.3f}, {z:.3f}, {x:.3f}, {theta:.3f}, {rpm1:.3f}, {rpm2:.3f}, {v_z:.2f}, {v_x:.2f}, {omega:.2f}, {theta_t2:.3f}, {del_rpm:.3f}\n")
            x_plot.append(drone.pos[0])
            z_plot.append(drone.pos[1])
            theta_plot.append(drone.rot[0])
            t_plot.append(t)
            
            #controller reaction
            controller(0.0, 3.0, drone, controller_step, del_t, t)


            #time steps forward
            #t += step
            #convert controller output to motor input
            #global time step
            t += global_step

    #plotting
    fig, axs = plt.subplots(3)
    axs[0].plot(t_plot, x_plot)
    axs[0].set_title('x')
    axs[1].plot(t_plot, z_plot)
    axs[1].set_title('z')
    axs[2].plot(t_plot, theta_plot)
    axs[2].set_title('theta')
    fig.show()
    #terminal interface
    out = input('press ENTER key to quit')

if __name__ == "__main__":
    main()
