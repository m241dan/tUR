--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 7/22/18
-- Time: 8:50 PM
-- To change this template use File | Settings | File Templates.
--
require('types')

-- "Toggle Down to Up"
return {
    [1] = { x = 20.00, y = 3.0, z = 13.0, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
    [2] = { x = 24.10, y = 2.9, z = 13.0, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
    [3] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 0, s3 = 0, s4 = -140, s5 = 0, s6 = 0 },
    [4] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 0, s3 = 0, s4 = 140, s5 = 0, s6 = 0 },
    [5] = { x = 20.00, y = 2.9, z = 13.0, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
}
