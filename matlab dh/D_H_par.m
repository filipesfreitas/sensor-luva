function matrix_A = D_H_par(O,A,d,a)

co = cos(O);
so = sin(O);
ca = cos(A);
sa = sin(A);

   matrix_A = [ co -so*ca so*sa a*co;
                so co*ca -co*sa a*so;
                0  sa     ca    d;
                0  0      0     1];
end
