close all; clc
s = tf('s');
%% FdT de la planta a lazo abierto %%

Gpol = tf(89.56, [1 5.145 16.88]);
step(Gpol);

%% Controller requeriments
% Overshoot <= 5% ; ts <= 3 seg
% Luego, calculo el polo de trabajo
mp = 0.05;
ts = 3;

zeta = sqrt((log(mp)^2)/(pi^2+log(mp)^2));  % zeta = 0.7448
wn = 5/(ts*zeta);
pt = -wn*zeta + 1i*wn*sqrt(1-zeta^2);

%% Calculo de PID
% Dado el polo de trabajo, graficamos el LdR de Gpol a lazo abierto
% Quereos diseñar un PID, por lo cual primero agregamos un integrador (polo
% en el origen). Luego, con la ayuda de la herramienta
% 'controlSystemDesigner' ubicamos un par de ceros complejos conjugados tal
% que Pt pertenezca al LdR.
% Por ultimo, ajusto la ganancia para cumplir con los requerimientos.

controlSystemDesigner('rlocus', Gpol);

Gpid = (5.9676+3.3481*s+0.5*s^2)/s;

Gol = Gpid*Gpol;
Gcl = feedback(Gol, 1);
step(Gcl);  
rlocus(Gol); sgrid



