function [ eax,eay,efx,efy,times,dthetas ] = searchParams(  )

global ax; global ay; global fx; global fy;
sax = [7:12];
say = [7:12];
sfx = [1:4];
sfy = [1:4];

eax = [];
eay = [];
efx = [];
efy = [];
times = [];
dthetas = [];

for i=1:length(sax)
    for j=1:length(say)
        for k=1:length(sfx)
            for h=1:length(sfy)
                [f, time, dtheta] = run(sax(i),say(j),sfx(k),sfy(h));
                
                if (f)
                    'success',time, dtheta, sax(i),say(j),sfx(k),sfy(h)
                    eax = [eax;sax(i)];
                    eay = [eay;say(j)];
                    efx = [efx;sfx(k)];
                    efy = [efy;sfy(h)];
                    times = [times:time];
                    dthetas = [dthetas;dtheta];
                end
            end
        end
    end
end

    [val, ind] = min(times)
    [tval, tind] = min(abs(dthetas))
    
end

