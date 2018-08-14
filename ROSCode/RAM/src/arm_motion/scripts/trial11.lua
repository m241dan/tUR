--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 8/14/18
-- Time: 2:38 PM
-- To change this template use File | Settings | File Templates.
--
require('types')

--trial 11 "Left Toggle Down to Up"
return {
    [1] = { x = 20.00, y = 7.5, z = 20.0, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
    [2] = { x = 23.50, y = 7.5, z = 20.0, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
    [3] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 0, s3 = 0, s4 = 110, s5 = 0, s6 = 0 },
    [4] = { velocity = 5, type = SERVO_R, s1 = 0, s2 = 0, s3 = 0, s4 = -110, s5 = 0, s6 = 0 },
    [5] = { x = 20.00, y = 7.5, z = 20.0, eeo = 0, velocity = 5, type = DISCRETE_W, precision = 1, shape = "linear", smoothness = 2, tolerance = 2 },
}