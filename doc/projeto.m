%{

FILTRO LCR por Vitor Paese De Carli

O codigo tem o objetivo de demonstrar o projeto de um controlador do tipo
PID para controlar a tensao sobre o capacitor de um Filtro LC(R), onde a
resistencia R corresponde a resistencia de um carga generica.

%}

% limpando o ambiente
clc
clear all
close all

%% definicoes

% definicao de s
s = tf('s');
% frequencia de discretizacao do controlador
fc = 2000;
tsc = 1/fc;
% frequencia de discretizacao da planta
f  = 1000*fc;
ts = 1/f;
% tempo para simulacao
t_inicial = 0;
t_final = 1; 
vetor_tempo = t_inicial:ts:t_final;

%% definicao do Filtro LCR em espaço de estados

% parametros do sistema
L1 = 1e-3;
R1 = 0.5;
C1 = 44e-6;
R2 = 100;
% matriz de estados
A  = [-R1/L1    -1/L1;
      1/C1      -1/(R2*C1)];
% matriz de entrada
B  = [ 1/L1;
       0 ];
% matriz de saida
C = [0 1]; %escolhendo "ver" a tensao sobre o capacitor C1
% matriz de transicao direta
D  = 0;
% definicao do espaco de estados
FILTRO_LCR_ss       = ss(A,B,C,D);
% definicao do espaco de estados no dominio do tempo discreto
[Ad,Bd,Cd,Dd,tz]    = ssdata(c2d(FILTRO_LCR_ss,ts)); 
FILTRO_LCR_ssd      = ss(Ad,Bd,Cd,Dd);

%% extracao das funcoes de transferencia

%{
as funcoes de transferencia permitem usar o sisotool, a ferramenta acelera
o projeto quando o sistema possui apenas 1 entrada.
%}

%tensao do capacitor em relacao a tensao de entrada
[num, den]  = ss2tf(A,B,C,D);
VC_tf       = minreal(tf(num, den));

%% definicao do controlador PID
kp = 5.05;
ki = 5;
kd = 0.05;
PID1 = kp + ki/s + kd * s;
% controlador sintonizado no sisotool
PID  = 0.05 * (s+1)*(s+100)*(1/s);
PIDd = minreal(c2d(PID, tsc, 'tustin'));
[num, den] = tfdata(PIDd, 'v');

%% definicao da malha de controle
VC_malha_aberta = PID * VC_tf;
VC_malha_fechada = minreal(VC_malha_aberta / (1 + VC_malha_aberta)); 
VC_malha_fechadad = minreal(c2d(VC_malha_fechada, tsc, 'tustin'));

figure(1)
pzmap(VC_malha_fechadad);
figure(2)
step(VC_malha_fechadad);
figure(3)
step(VC_malha_fechada);

ts = 1/2000;
kp1 = 5.05;
ki1 = 5 * ts * 1/2;
kd1 = 0.05 * 2 * 1/ts;

k1 = kp1 + ki1 + kd1
k2 = 2 * ki1 - 2 * kd1
k3 = -kp1+ ki1 + kd1