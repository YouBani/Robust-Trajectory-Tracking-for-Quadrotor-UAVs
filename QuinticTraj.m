% Generation of a cubic trajectory
function a = QuinticTraj(tspan, x0, xf)
    to = tspan(1);
    tf = tspan(2);
    
    A = [1 to to^2 to^3 to^4 to^5;
        0 1 2*to 3*(to^2) 4*(to^3) 5*(to^4);
        0 0 2 6*to 12*to^2 20*to^3;
        1 tf tf^2 tf^3 tf^4 tf^5;
        0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
        0 0 2 6*tf 12*tf^2 20*tf^3];
    
    b = [x0; xf];
    
    a = A\b;
end