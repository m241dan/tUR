#!/usr/local/bin/lua

local lfs = require( "lfs" )
local dir_name = arg[1]
local combined_file_name = arg[2] or "c.out"
local file_names = {}


function compare( a, b )
  return a < b
end

count = 1
for file_name in lfs.dir( dir_name ) do
    if( file_name ~= "." and file_name ~= ".." ) then
       print( file_name )
       file_names[count] = dir_name .. "/" .. file_name
       count = count + 1
    end
end

sorted = table.sort( file_names, compare )
print( tostring( sorted ) )

-- local combined_file = io.open( combined_file_name, "w" )

--for i, file_name in ipairs( file_names ) do
--    local curr_file = io.open( file_name, "r" )
--    for line in curr_file:lines() do
        --line = string.gsub( line, "^(\x01)(\x21)", function( a, b ) return ""; end )
        --combined_file:write( line )
    --end
--end

--combined_file:close()


