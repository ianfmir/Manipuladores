close all
clear
clc

%Cria um cenário
Cen=Cenario([]);
%Cria um robo
R = Robo.Cria_KukaKR5();
%Adiciona o robo
Cen.adicionaobjetos(R);
%Faz a animaçao
dt=0.01;
for i = 1: 100
    %Atualiza a mth do Robo
    MTH=Robo.desl([-0.01*i;0;0]);
    R.mth = MTH;
    R.config(R.q);
    Cen.desenha();
    drawnow;
end
