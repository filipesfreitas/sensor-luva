% función que dibuja un sistema de coordenadas de tamaño t definido por la
% Matriz de transformación homogénea A y con un titulo en cada uno de los ejes
% coordenados

function draw_cord_sys(rot,p,t,sub)
    
    
    u=rot(1:3,1)*t + p;
    v=rot(1:3,2)*t + p;
    w=rot(1:3,3)*t + p;
    
    dibujar_linea(p,u,[1 0 0],2);
    dibujar_linea(p,v,[0 1 0],2);
    dibujar_linea(p,w,[0 0 1],2);
    
    text( u(1),u(2),u(3),strcat(['x_{',sub,'}']) );
    text( v(1),v(2),v(3),strcat(['y_{',sub,'}']) );
    text( w(1),w(2),w(3),strcat(['z_{',sub,'}']) );
end