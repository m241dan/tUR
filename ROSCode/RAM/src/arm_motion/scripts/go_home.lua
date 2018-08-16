--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 7/22/18
-- Time: 8:39 PM
-- To change this template use File | Settings | File Templates.
--

require('types')

-- Return to Stump
-- Ending position X: 20.42, Y: 14.33, Z: 5.73

return {
    [1] =  { x =  18.95, y =  13.0, z = 13.19, eeo = 0.292, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness=10, tolerance=2 };
     [2] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = GRIPPER_OPEN },
     [3] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = -200, s3 = 0, s4 = 0, s5 = 0, s6 = 0 },
     [4] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 2470 },
}
