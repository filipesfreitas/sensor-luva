function matrix_A = D_H_par(O,A,d,a)
   
   matrix_A = [ cos(O) -sin(O) 0 a;
                sin(O)*cos(A) cos(O)*cos(A) -sin(A) d*sin(O);
                sin(O)*sin(A) cos(A)*cos(O)  cos(A) cos(A)*d;
                0       0             0             1];
end
