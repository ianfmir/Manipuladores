close all
clear
clc

%Queremos que o centro do paraleleṕıpedo
%esteja em [0;0.6;0.75]
MTH=Robo.desl([0;0.8;0.75]);
%Queremos as dimens~oes 1, 0.1 e 1.5
lados = [1.5 0.1 1.5];
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

%criando o Tdes
Tef0= R.cinematicadir(R.q, 'efetuador');
Tdes = Robo.desl([-0.75; 0.20; 0.745])*Tef0; %posicao desejada do efetuador em metros o ; para transformar em uma matriz coluna
xdes = Tdes(1:3,1);
ydes = Tdes(1:3,2);
zdes = Tdes(1:3,3);
pdes = Tdes(1:3,4);

%eixo da posicao desejada
E = Eixo(Tdes, 0.2,{'xdes', 'ydes', 'zdes'});
C.adicionaobjetos(E);

%constantes
dt = 0.01
K = 7;

%historicos
q_hist=[];
r_hist=[];
u_hist=[];
t=[];
C.desenha();

pause();

for k = 1: 2/dt %tempo em s/dt
    q_hist(:,k) = R.q;
    t(k) = (k-1)*dt;
    
    Tef = R.cinematicadir(R.q,'efetuador');
    Jgeo = R.jacobianageo(R.q,'efetuador');
    
    pef = Tef(1:3,4);
    xef = Tef(1:3,1);
    yef = Tef(1:3,2);
    zef = Tef(1:3,3);
    
    Jp = Jgeo(1:3,:);
    Jw = Jgeo(4:6,:);
    
    rpos = pef-pdes;
    rorix = 1-xdes'*xef;
    roriy = 1-ydes'*yef;
    roriz = 1-zdes'*zef;
    
    r = [rpos;rorix;roriy;roriz];
    
    Jrpos = Jp;
    Jrorix = xdes'*Robo.matrizprodv(xef)*Jw;  
    Jroriy = ydes'*Robo.matrizprodv(yef)*Jw;
    Jroriz = zdes'*Robo.matrizprodv(zef)*Jw;
    
    Jr = [Jrpos;Jrorix;Jroriy;Jroriz];
    
    u = Robo.pinvam(Jr,0.001)*(-K*r);
    
    u_hist(:,k) = u;
    r_hist(:,k) = r;
    
    qprox = R.q + u*dt;
    R.config(qprox);
    C.desenha();
    drawnow;
    
end
