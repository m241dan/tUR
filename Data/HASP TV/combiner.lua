#!/usr/local/bin/lua

local lfs = require( "lfs" )
local dir_name = arg[1]
local file_names = {}

for file_name in lfs.dirs( dir_name ) do
    local iter, match = file_name:gmatch()
    if( match ) then
    end
end

