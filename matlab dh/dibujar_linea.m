% Dibujar una linea a partir de dos puntos
% ejemplo:
% p0 = [0, 0, 0]; % punto inicial
% p1 = [1, 1, 1]; % punto final
% color = [1, 0, 0]; % rojo: [1, 0, 0], verde: [0, 1, 0], Azul: [0, 0, 1]
% grosor = 2;
% dibujar_linea(p0, p1, color, grosor);
function dibujar_linea( p0, p1, color, grosor )
    % vectores que contienen las componentes de los puntos a dibujar
    x = [ p0(1), p1(1)];
    y = [ p0(2), p1(2)];
    z = [ p0(3), p1(3)];
    % Dibujo de las lineas
    plot3(x,y,z, 'Color', color ,'LineWidth',grosor);
end