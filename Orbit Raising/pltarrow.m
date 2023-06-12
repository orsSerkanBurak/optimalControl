function pltarrow(x,y,scale,col)
% PLTARROW Plot arrow from (x(1),y(1)) to (x(2),y(2)).
%    PLTARROW(X,Y) plots an arrow from (X(1),Y(1)) to (X(2),Y(2)).
%    PLTARROW(X,Y,SCALE) scales the arrow by SCALE (default SCALE=1).
%    PLTARROW(X,Y,SCALE,COL) uses color COL (default COL='k').

if nargin<3
    scale=1;
end
if nargin<4
    col='k';
end

% Compute arrow tip coordinates
deltax = x(2) - x(1);
deltay = y(2) - y(1);
theta = atan2(deltay, deltax);
L = sqrt(deltax^2 + deltay^2) * scale;
tipx = x(2) - 0.05*L*cos(theta);
tipy = y(2) - 0.05*L*sin(theta);

% Plot line and arrow
line(x, y, 'Color', col);
line([x(2) tipx], [y(2) tipy], 'Color', col);
line([tipx x(2)-0.05*L*cos(theta+pi/16)], [tipy y(2)-0.05*L*sin(theta+pi/16)], 'Color', col);
line([tipx x(2)-0.05*L*cos(theta-pi/16)], [tipy y(2)-0.05*L*sin(theta-pi/16)], 'Color', col);
end
