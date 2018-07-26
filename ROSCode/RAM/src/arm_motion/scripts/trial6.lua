--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 5/16/18
-- Time: 2:49 PM
-- To change this template use File | Settings | File Templates.
--

require('types')

--trial 6 "Blue Button from Home"
return {
    [1] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = GRIPPER_CLOSED},
    [2] = { x = 22.00, y = -5.00, z = 20.7, eeo = 0, velocity = 5, type = DISCRETE_W, precision = .5, shape = "linear", smoothness = 2, tolerance = 2},
    [3] = { x = 23.7, y = -5.1, z = 20.7, eeo = 0, velocity = 5, type = DISCRETE_W, precision = .1, shape = "linear", smoothness = 2, tolerance = 2 },
    [4] = { x = 22.00, y =-5.00, z = 20.7, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
}