close all
clear
clc

%Cria um cenário
Cen=Cenario([]);
%Cria um robo
R = Robo.Cria_KukaKR5();
%Encontra posicao para o pincel
MTHC = R.cinematicadir(R.q,'efetuador')*Robo.rot('x',pi/2)*Robo.desl([0;0.05;-0.1]);
%Parametros do pincel
raio = 0.05;
altura = 0.2;
cor = [1 0 0];
densidade = 1;
%Cria pincel
pincel = Cilindro(MTHC, raio, altura, cor, densidade);
pincel.desenha();
R.grudaobjeto(pincel);
%Adiciona o robo
Cen.adicionaobjetos(R);
Cen.adicionaobjetos(pincel);
Cen.desenha();
drawnow;
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
