import math
import matplotlib.pyplot as plt

def main():
    file_path = "my_text_file.txt"
    m = 0.1
    I = 0.1
    x = 0.0     #location of drone CoG
    z = 0.0
    g = 9.81
    theta = 0.0
    rpm1 = 0.0
    rpm2 = 0.0
    k = 0.2*9.81/30000  #conversion factor of rpm to thrust
    b = 0.1
    h = 0.005
    p = 1.0
    i = 0.0
    d = 50.0
    p_x = 1.0
    i_x = 0.0
    d_x = 50.0
    t = 0.0
    v_x = 0.0
    v_z = 0.0
    a_x = 0.0
    a_z = 0.0
    alpha = 0.0
    omega = 0.0
    step = 0.001
    runtime = 100.0
    cum_z_error = 0.0
    cum_x_error = 0.0
    
    set_z = 20.0
    set_x = 5.0
    
    z_error = set_z - z
    x_error = set_x - x
    
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
        file.write("t, z, x, theta, z_error, x_error, rpm1, rpm2, v_z, v_x\n")
    
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
            file.write(f"{t:.3f}, {z:.3f}, {x:.3f}, {theta:.3f}, {z_error:.3f}, {x_error:.3f}, {rpm1:.3f}, {rpm2:.3f}, {v_z:.2f} {v_x:.2f}\n")


            #error calculation
            last_z_error = z_error
            last_x_error = x_error
            z_error = set_z - z
            x_error = set_x - x
            cum_z_error += z_error
            cum_x_error += x_error
            
            #error correction attempt
            z_correction = p*z_error + i*cum_z_error*step + d*(z_error-last_z_error)/step
            rpm1 += z_correction
            rpm2 += z_correction

            x_correction = p_x*x_error + i_x*cum_x_error*step + d_x*(x_error-last_x_error)/step
            rpm1 -= x_correction 
            rpm2 += x_correction
            
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
        axs[0].plot(t_plot, z_plot)
        axs[0].plot(t_plot, x_plot)
        axs[0].plot(t_plot, theta_plot)
        
        

if __name__ == "__main__":
    main()
