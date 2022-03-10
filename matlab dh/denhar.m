% Funcion que recibe una fila de la tabla de parametros de
% Denavit-Hartemberg y retorna la matriz de transformacion homogenea de la
% articulacion ingresada

function MTH=denhar(q,d,a,alpha)

cq = cos(q);
sq = sin(q);
ca = cos(alpha);
sa = sin(alpha);

MTH = [ cq    -sq*ca   sq*sa     a*cq ; ...
        sq    cq*ca    -cq*sa    a*sq ; ...
        0     sa       ca        d    ; ...
        0     0        0         1   ];

end