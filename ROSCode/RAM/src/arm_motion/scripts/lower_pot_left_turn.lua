--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 8/14/18
-- Time: 3:08 PM
-- To change this template use File | Settings | File Templates.
--
require( 'types' )

--lower pot right turn
return {
    [1] = { x = 21.0, y = 0.5, z = 9.9, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 5, tolerance = 2},
    [2] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 2048, s6 = GRIPPER_OPEN },
    [3] = { x = 24.1, y = 0.5, z = 9.9, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 5, tolerance = 2},
    [4] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = GRIPPER_POT }, -- GRIPPER_POT },
    [5] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = -800, s6 = 0 },
    [6] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 800, s6 = 0 },
    [7] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = GRIPPER_OPEN },
    [8] = { x = 21.0, y = 0.5, z = 9.9, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 5, tolerance = 2},
    [9] = { velocity = 5, type = SERVO_ABSOLUTE, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = GRIPPER_CLOSED},
}

