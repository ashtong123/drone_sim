import numpy as np

#motor_thrust_constant and motor_torque_constant represent the proportionality 
#of the controllable value applied to the motor (avg voltage applied by PWM) to the 
#output thrust and torques. Here we roughly assume they are 1.0, but a detailed analysis 
#to provide more accurate values will eventually be necessary
class Drone:
    def __init__(self, motor_offset, thickness, density, motor_thrust_constant, motor_torque_constant): #(m, m, kg/m3, unitless, unitless)
        self.motor_offset = motor_offset            #vectors describing the relative positions of the rotors to the drone center\
        self.thickness = thickness
        self.density = density
        self.motor_thrust_constant = motor_thrust_constant
        self.motor_torque_constant = motor_torque_constant
        self.transform = None #6 element array describing the x, y, z, u, v, w offsets of local csys wrt global
        self.controller = None #initially just a proportional controller (cf = error)
        self.velocity = None #m/s
        self.acceleration = None #m/s2
        self.mass = None
        self.motor_speed = None
        self.net_force = None
        

def main():
    file_path = "my_text_file.txt"
    with open(file_path, "w") as file:
        #begin logging
        file.write("FLIGHT LOG\n")
        file.write("time(ms), height(mm)\n")
        
        #init drone object
        drone = Drone((0.1, 0.1), 0.002, 1250, 1.0, 1.0)             
        
        #set initial conditions of drone
        drone.lposition = np.array([0.0, 0.0, 0.0])
        drone.controller = 1.0
        drone.lvelocity = np.array([0.0, 0.0, 0.0])
        drone.lacceleration = np.array([0.0, 0.0, 0.0])
        drone.motor_speed = np.array([0.0, 0.0, 0.0, 0.0])
        drone.mass = ((drone.motor_offset * 2) ** 2) * drone.thickness *  drone.density     #kg
        drone.net_force = np.array([0.0, 0.0, 0.0])
        
        #set simulation parameters
        set_height = 10.0       # try to hover drone at 10.0m
        time = 0.0              # start at t = 0
        step = 0.001            # step in 1ms increments
        runtime = 10000.0       #run sim for 10,000s
        g = 9.81
        #begin simulation
        while(time <= runtime):
            #determine forces and torques acting on drone
            #net vertical forces
            if drone.position[2] <= 0.0 and drone.mass*g > np.sum(drone.motor_speed*drone.motor_thrust_constant):
                net_force[2] = 0.0         #for now assuming force will only exist in +-z direction
            else:
                net_force[2] = - drone.mass*g + np.sum(drone.motor_speed*drone.motor_thrust_constant)
            #use forces to determine acceleration
            drone.lacceleration = drone.net_force/drone.mass
            #update drone state according to thrust and torques                                              #kg*m/s2
            drone.lvelocity += step*drone.lacceleration
            drone.lposition += step*drone.lvelocity
            
            #CONTROLLER DETERMINES ERROR
            error = set_height - drone.lposition[2]
            #CONTROLLER UPDATES MOTOR SPEED TO CREATE IMPULSE
            drone.motor_speed = np.add(drone.motor_speed, error*drone.controller)

            #log current drone state (currently only z position)
            file.write(str(time) + ", " + str(drone.position[2]))
            time += step
            
if __name__ == "__main__":
    main()
    