xmesh = linspace(0,3,10);
for k =-10:0.1:10
    solinit = bvpinit(xmesh,@guess, k);
    % solinit = bvpinit(x,v,parameters)
    try
        sol = bvp4c(@bvpfun, @bcfun, solinit);
    catch 
        warning('No value found');
    end
end
