% Funcion que devuelve la matriz de rotacion en el eje X y tiene como
% parametro de entrada el ángulo en grados

function A=rot_axis(ang,axis)
    s = sin(ang);
    c = cos(ang);
    switch axis
        case 1 
            A=[ 1,  0,  0;
                0,  s,  -s;
                0,  s,  c];

        case 2
            A=[ c,  0,  s;
                0,	1,  0;
                -s, 0,  c];
            
        case 3
            A=[ c,  -s, 0;
                s,  c,  0;
                0,	0,	1];
        otherwise
            error("Select correct Axis (1, 2, 3)")
    end
end