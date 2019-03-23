function [ user, location, use_mode ] = get_file_info( filename )
%% get the user
if ~isempty(strfind(filename,'dwf'))
    user = 'dwf';
elseif ~isempty(strfind(filename,'mjb'))
    user = 'mjb';
elseif ~isempty(strfind(filename,'lzq'))
    user = 'lzq';
elseif~isempty(strfind(filename,'qjc'))
    user = 'qjc';
else
    user = 'lcx';
end
%% get the location
if ~isempty(strfind(filename,'nanti'))
    location = 'nanti';
elseif ~isempty(strfind(filename,'lab'))
    location = 'lab';
elseif ~isempty(strfind(filename,'outdoor'))
    location = 'outdoor';
elseif ~isempty(strfind(filename,'indoor'))
    location = 'indoor';    
else
    location = 'somewhere';
end
%% get the use_mode
if ~isempty(strfind(filename,'compass'))
    use_mode = 'compass';
elseif ~isempty(strfind(filename,'hand'))
    use_mode = 'in hand';
elseif ~isempty(strfind(filename,'listen'))
    use_mode = 'listening';
elseif ~isempty(strfind(filename,'pocket'))
    use_mode = 'in pocket';   
elseif ~isempty(strfind(filename,'foot'))
    use_mode = 'on foot';  
else
    use_mode = 'somehow';
end

end

