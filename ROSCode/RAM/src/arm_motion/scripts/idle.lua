--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 6/24/18
-- Time: 7:22 PM
-- To change this template use File | Settings | File Templates.
--

--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 5/16/18
-- Time: 2:49 PM
-- To change this template use File | Settings | File Templates.
--

require('types')

-- Home Position
return {
    [1] =  { x =  17.0, y =  0.0, z = 12, eeo = 0, velocity = 5, type = DISCRETE_W,precision = 1, shape = "linear", smoothness=10, tolerance=2 };
    [2] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 2048, s6 = GRIPPER_CLOSED},
}