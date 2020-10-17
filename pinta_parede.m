close all
clear
clc

%Queremos que o centro do paraleleṕıpedo
%esteja em [0;0.6;0.75]
MTH=Robo.desl([0;0.8;0.75]);
%Queremos as dimens~oes 1, 0.1 e 1.5
lados = [1 0.1 1.5];
%Escolhemos as cores (RGB)
cor = [0.9 0.9 0.9];
%Queremos densidade 1
densidade = 1;
P=Paralelepipedo(MTH,lados,cor,densidade);

%Criando obstaculo (Janela)
%definindo o centro
MTHJ=Robo.desl([0;0.75;0.85]);
%Queremos as dimensoes 0.3, 0.1 e 0.4
ladosJ = [0.3 0.1 0.4];
%Escolhemos as cores (RGB)
corJ = [0.3 0.3 0.7];
J=Paralelepipedo(MTHJ,ladosJ,corJ,densidade);

%cria robo
R = Robo.Cria_KukaKR5();
%Cria um cenário
C = Cenario(R);
C.adicionaobjetos(P);
C.adicionaobjetos(J);
C.adicionaobjetos(R);

C.desenha();
