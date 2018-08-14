--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 5/16/18
-- Time: 2:49 PM
-- To change this template use File | Settings | File Templates.
--

require('types')

-- Trial 1: Leave Stump
-- Starting position X: 20.42, Y: 14.33, Z: 5.73

return {
    [1] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 2048, s6 = GRIPPER_OPEN },
    [2] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 200, s3 = 0, s4 = 0, s5 = 0, s6 = 0 },
    [3] = { x =  17.0, y =  0.0, z = 12, eeo = 0, velocity = 5, type = DISCRETE_W,precision = 1, shape = "linear", smoothness=10, tolerance=2 };
    [4] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 2048, s6 = GRIPPER_CLOSED},

}