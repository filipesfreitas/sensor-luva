function cord_system(A,sub)
  t=1;
  p=A(1:3,4);
  
  u=A(1:3,1)*t + p;
  v=A(1:3,2)*t + p;
  w=A(1:3,3)*t + p;
  
hold on;
  grid on;
  scatter3(p(1),p(2),p(3),"r","filled");
  xlabel('x')
  ylabel('y')
  zlabel('z')
  x = [p(1);u(1)];
  y = [p(2);u(2)];
  z = [p(3);u(3)];
  plot3(x,y,z ,'Color', [1 0 0],'LineWidth',3);
  x = [p(1);v(1)];
  y = [p(2);v(2)];
  z = [p(3);v(3)];
  plot3(x,y,z ,'Color', [0 1 0],'LineWidth',3);
  x = [p(1);w(1)];
  y = [p(2);w(2)];
  z = [p(3);w(3)];
  plot3(x,y,z ,'Color', [0 0 1],'LineWidth',3);

  text( u(1),u(2),u(3),strcat(['x_{',sub,'}']) );
  text( v(1),v(2),v(3),strcat(['y_{',sub,'}']) );
  text( w(1),w(2),w(3),strcat(['z_{',sub,'}']) );

end
