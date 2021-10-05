%% Transform a 3D vector into a skew symmetric matrix.
function skew = vec2skew(x)
    skew = [   0 -x(3)  x(2) ;
         x(3)     0 -x(1) ;
        -x(2)  x(1)     0 ];
end
