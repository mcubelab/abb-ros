function [ ds ] = evaluate( p )

    global curr_s;

    s = model(p);
    ds = abs(curr_s - s);

end

