%% Estimacion de la FdT de la planta
% 1- Importo datos de la hoja de calculos y los separo
% 2- Estimar la FdT (tfest)
% 3- Comparar los datos experimentales con la estimacion de matlab

%% 1
clear
clc 

file = 'datos_tf_sist_lz.xlsx';         % arhivo con los datos
regionC = xlsread(file, 'regionC');     % importo los datos de hoja 'regionC'
timeC = regionC(:,2);                   % guardo el tiempo (en s)
tsC = (timeC(3)-timeC(2));              % tiempo de muestreo (en s)
heightC = regionC(:,1);                 % guardo la altura (en cm)
inputC = regionC(:,3);
dataC = iddata(heightC, inputC, tsC);   % salida, entrada, Tmuestreo
plot(dataC);

%% 2
tf20 = tfest(dataC, 2, 0);              % Fit to estimation data: 92.28% 
step(tf20);

%% 3
compare(dataC, tf20);

%% Controller requeriments
mp = 0.05;
ts = 3;

zeta = sqrt((log(mp)^2)/(pi^2+log(mp)^2));  % zeta = 0.7448
od = 5/ts;
wn = 5/(ts*zeta);
wd = od*tan(acos(zeta));

pt = -od + wd*1i;
pw = -wn*zeta + 1i*wn*sqrt(1-zeta^2);
controlSystemDesigner('rlocus', tf20);

Cpid = 0.316*((1+0.21*s+(0.43*s)^2)/s;    % tf PID designed

%% PID TUNER
s = tf('s');
[kp, ki, kd] = deal(0.405, 4.083, 0.03787);
PID = kp + ki/s + kd*s;

%% Root locus tf20 to fit with PID tuner
controlSystemDesigner('rlocus', tf20);

C = 0.403*((1+0.099*s+ (0.096*s)^2)/s);

