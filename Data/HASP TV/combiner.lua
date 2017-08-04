#!/usr/local/bin/lua

local lfs = require( "lfs" )
local dir_name = arg[1]
local combined_file_name = arg[2] or "combined.out"
local file_names = {}

for file_name in lfs.dir( dir_name ) do
    local identifier

    file_name:gsub( ".*-(%d+).txt", function( a ) identifier = a; end )
    if( identifier ) then
        file_names[tonumber(identifier)] = dir_name .. "/" .. file_name;
    end
end

local combined_file = io.open( combined_file_name, "a+" )

for i, file_name in ipairs( file_names ) do
    local curr_file = io.open( file_name, "r" )
    combined_file:write( curr_file:read( "*a" ) )
end

combined_file:close()
