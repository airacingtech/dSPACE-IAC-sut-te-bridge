std_msgs/Header header
uint8           car_num # number of fellow-vehicle
float64         lat     # [deg] absolute position of fellow-vehicle in global coords
float64         lon     # [deg] absolute position of fellow-vehicle in global coords
float64         hgt     # [deg] absolute position of fellow-vehicle in global coords
float64         vx      # [m/s] relative velocity of fellow-vehicle in ego-vehicle local coords
float64         vy      # [m/s] relative velocity of fellow-vehicle in ego-vehicle local coords
float64         vz      # [m/s] relative velocity of fellow-vehicle in ego-vehicle local coords
float64         yaw     # [rad] from North, CCW positive
float64         pitch   # [rad]
float64         roll    # [rad]
float64         del_x   # [m] relative position of fellow-vehicle in ego-vehicle local coords
float64         del_y   # [m] relative position of fellow-vehicle in ego-vehicle local coords
float64         del_z   # [m] relative position of fellow-vehicle in ego-vehicle local coords