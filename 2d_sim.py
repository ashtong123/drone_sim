import math

def main():
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
    p = 0.0
    i = 0.0
    d = 0.0
    t = 0.0
    a_x = 0.0
    a_z = 0.0
    alpha = 0.0
    omega = 0.0
    step = 0.001
    runtime = 100.0
    cum_z_error = 0.0
    cum_x_error = 0.0
    z_error = 0.0
    x_error = 0.0
    
    set_z = 20.0
    set_x = 5.0
    
    z_plot = []
    x_plot = []
    theta_plot = []
    t_plot = []
    
    while(t < runtime):
        #determine the forces and torques
        #base forces
        f_1 = rpm1*k
        f_2 = rpm2*K
        #z components
        f_1_z = f_1*math.sin(math.radians(90+theta))
        f_2_z = f_2*math.sin(math.radians(90+theta))
        #net z force
        f_z = -m*g + f_1_z + f_2_z
        #x components
        f_1_x = f1*math.cos(math.radians(90+theta))
        f_2_x = f2*math.cos(math.radians(90+theta))
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
        
        #position update
        x += v_x*step
        z += v_z*step
        theta += omega*step
        
        #save data for plotting
        x_plot.append(x)
        z_plot.append(x)
        theta_plot.append(theta)
        t_plot.append(t)
        
        #error calculation
        last_z_error = z_error
        last_x_error = x_error
        z_error = set_z - z
        x_error = set_x - x
        cum_z_error += z_error
        cum_x_error += x_error
        
        #error correction attempt
        thrust_correction = p*z_error + i*cum_z_error*step + d*(z_error-last_z_error)/step
        
        #time marches on...
        t += step
        
if __name__ == "__main__":
    main()