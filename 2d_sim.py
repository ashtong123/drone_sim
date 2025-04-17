import math
import matplotlib.pyplot as plt

def main():
    file_path = "my_text_file.csv"
    m = 0.1
    I = 0.1
    x = 0.0     #location of drone CoG
    z = 0.0
    g = 9.81
    theta = 30.0
    rpm1 = 0.0
    rpm2 = 0.0
    k = 1.0*9.81/30000  #conversion factor of rpm to thrust
    b = 0.1
    h = 0.005
    p = 10.0
    i = 0.0
    d = 100.0
    p_x = 0.0
    i_x = 0.0
    d_x = 0.0
    p_th = 1000.0
    i_th = 100.0
    d_th = 2000.0
    t = 0.0
    v_x = 0.0
    v_z = 0.0
    a_x = 0.0
    a_z = 0.0
    alpha = 0.0
    omega = 0.0
    step = 0.1
    runtime = 1000.0
    cum_z_error = 0.0
    cum_x_error = 0.0
    cum_theta_error = 0.0
    
    set_z = 20.0
    set_x = 5.0
    set_theta = 0.0
    
    z_error = set_z - z
    x_error = set_x - x
    theta_error = -theta
    
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

    with open(file_path, "w") as file:
        #begin logging
        file.write("FLIGHT LOG\n")
        file.write("t, z, x, theta, z_error, x_error, theta_error, rpm1, rpm2, v_z, v_x, omega, ss_theta_error\n")
    
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
            
            #velocity update
            v_x += a_x*step
            v_z += a_z*step
            omega += alpha*step
            
            if v_x > 15.0:
                v_x = 15.0
            elif v_x < -15.0:
                v_x = -15.0

            if v_z > 15.0:
                v_z = 15.0
            elif v_z < -15.0:
                v_z = -15.0

            if omega > 20.0:
                omega = 20.0
            elif omega < -20.0:
                omega = -20.0

            #position update
            x += v_x*step
            z += v_z*step
            theta += omega*step
            
            #save data for plotting
            x_plot.append(x)
            z_plot.append(x)
            theta_plot.append(theta)
            t_plot.append(t)
        
            #write data to logfile
            file.write(f"{t:.3f}, {z:.3f}, {x:.3f}, {theta:.3f}, {z_error:.3f}, {x_error:.3f}, {theta_error:.3f}, {rpm1:.3f}, {rpm2:.3f}, {v_z:.2f}, {v_x:.2f}, {omega:.2f}, {cum_theta_error/(t+.0001):.2f}\n")

            #controller
            #my original thought was to try to control the motor rpms based on the x, z, and theta
            #errors. I have a new idea, that is, to control the motor rpms directly. 
            #this will require determining the relationship between where the drone is and
            #where we want it to be, then converting that information directly into 
            #rpms for each motor
            
            
            # We want the drone to move from its current position to some other specificed position
            # The current position is defined as
            curr_pos = [x, z]
            
            #whereas the desired position is defined as
            set_pos = [set_x, set_z]
            
            #thus, there is a vector which defines the shortest path between curr_pos and set_pos,
            #which we call pos_error, defined as the difference set_pos - curr_pos
            #pos_error is the answer to the question: "how do I travel from
            #curr_pos to set_pos?
            pos_error = [a - b for a,b in zip(set_pos, curr_pos)]
            
            #we can control two values: rpm1 and rpm2. How do we control these values in order
            #to minimize the magnitude of pos_error?
            #in any given instant, the action which most efficiently reduces the position error is
            #to travel in the direction of the position error vector as fast as possible,
            #we can call this it optimal_travel_vector
            #the optimal travel vector will be in the direction of the pos_error vector, but 
            #will have the highest possible magnitude. In the case of our drone, the
            #magnitude will depend on directio, for simplicity's sake, we will assume that it is
            #direction independent, and has a value of 15m/s
            #we can achieve this by turning pos_error into a unit vector and multiplying by 15:
            mag = math.sqrt(sum([(a**2) for a in pos_error]))
            optimal_travel_vector = [15*a/mag for a in pos_error] # for stability reasons, we may want 
                                                                  # the optimal travel vector to depend on the
                                                                  # size of the pos_error, so that we 'slow down'
                                                                  # as we get close to our target to avoid overshoot
            
            
            # ! Since we can only control RPMs, and RPMs affect thrust/acceleration directly, and 
            # velocity indirectly, we want to find the optimal change in velocity (acceleration) 
            # that will modify our current trajectory to match the optimal trajectory
            # In addition to the position error vector, there also exists a velocity error vector
            # the velocity error vector is the vector difference between the optimal_travel_vector
            # and the current_travel_vector
            # the current_travel_vector can be determined numerically (See above):
            current_travel_vector = [v_x, v_z]
            # where v_x and v_z are continuously computed based on the initial velocities plus the 
            #total accumulated change in velocity.
            #Thus, the velocity error vector can be calculated
            vel_error = [a - b for a,b in zip(optimal_travel_vector, current_travel_vector)]
            
            #the vel_error represents the change in velocity (acceleration) necessary to change 
            #our actual velocity into the optimal velocity
            
            #also, keep in mind that there are limitations on safe values of theta, 
            #and limitations on achievable values of v_x, v_z, and omega
            
            """
            

            last_z_error = z_error
            last_x_error = x_error
            last_theta_error = theta_error
            z_error = set_z - z
            x_error = set_x - x
            
            if x_error >= 1.0:
                set_theta = 15.0
            else:
                set_theta = 0.0
            
            theta_error = set_theta - theta
            cum_z_error += z_error
            cum_x_error += x_error
            cum_theta_error += theta_error
            
            
            #error correction attempt
            
            z_correction = p*z_error + i*cum_z_error*step + d*(z_error-last_z_error)/step
            rpm1 += z_correction
            rpm2 += z_correction

            x_correction = p_x*x_error + i_x*cum_x_error*step + d_x*(x_error-last_x_error)/step
            rpm1 -= x_correction 
            rpm2 += x_correction
            
            theta_correction = p_th*theta_error + i_th*cum_theta_error*step + d_th*(theta_error-last_theta_error)/step
            rpm1 += theta_correction
            rpm2 -= theta_correction
            """
            
            if rpm1 > 30000.0:
                rpm1 = 30000.0
            elif rpm1 < 0.0:
                rpm1 = 0.0
            if rpm2 > 30000.0:
                rpm2 = 30000.0
            elif rpm2 < 0.0:
                rpm2 = 0.0
                
            #time marches on...
            t += step

        fig, axs = plt.subplots(3)
        fig.suptitle('Flight Sim Log')
        axs[0].set_title('z vs. t')
        axs[0].plot(t_plot, z_plot)
        axs[1].set_title('x vs. t')
        axs[1].plot(t_plot, x_plot)
        axs[2].set_title('theta vs. t')
        axs[2].plot(t_plot, theta_plot)
        fig.show()
        key = input('Press any key to close')
        
        

if __name__ == "__main__":
    main()
