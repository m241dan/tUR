--
-- Created by IntelliJ IDEA.
-- User: korisd
-- Date: 5/16/18
-- Time: 3:34 PM
-- To change this template use File | Settings | File Templates.
--


SERVO_ABSOLUTE = 4
SERVO_R = 3;
VISION = 2;
DISCRETE_R = 1;
DISCRETE_W = 0;

GRIPPER_OPEN = 1701
GRIPPER_POT = 2000
GRIPPER_CLOSED = 2510

function addObjective( trial, objective )
    objective = dofile( objective .. ".lua" )
    if objective ~= nil and type( objective ) == "table" then
        for _, v in ipairs( objective ) do
           table.insert( trial, #trial+1, v )
        end
    end
end