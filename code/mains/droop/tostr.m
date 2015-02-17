function [ s ] = tostr(array)

    s = '{';
    for i=1:(length(array)-1),
        s = strcat(s,num2str(array(i)),',');
    end
    s = strcat(s,num2str(array(length(array))),'}');
        
end