%% Trabalho Final - Manipuladores Robóticos
% Simulação de um robô que pinta paredes desviando de obstáculos                       
% Autores:
% André Freire Vidigal - 2014144448
% Ian Fernandes Miranda - 2015113295
% Vitor Ribas Moura - 2015129094

%% Preparação do ambiente

close all
clear
clc

%% Criação do cenário

% Posicionamento do centro da parede
MTH=Robo.desl([0;0.7;0.4]);
% Dimensionamento da parede
lados = [3 0.1 0.8];
% Coloração da parede (RGB)
cor = [0.9 0.9 0.9];
% Densidade da parede
densidade = 1;
% Criação da parede
P=Paralelepipedo(MTH,lados,cor,densidade);

% Posicionamento do centro da janela
MTHJ=Robo.desl([0;0.7;0.3]);
% Dimensionamento da janela
ladosJ = [1 0.1 0.6];
% Coloração da janela (RGB)
corJ = [0.3 0.3 0.7];
% Criação da janela
J=Paralelepipedo(MTHJ,ladosJ,corJ,densidade);

% Criação do robô
R = Robo.Cria_KukaKR5();

% Criação do cenário
C = Cenario(R);
% Adição da parede
C.adicionaobjetos(P);
% Adição da janela
C.adicionaobjetos(J);
% Adição do robô
C.adicionaobjetos(R);

% Criação da posição inicial desejada do efetuador
Tef0= R.cinematicadir(R.q, 'efetuador');
Tdes = Robo.desl([-1.5; 0.6; 0]); %*Tef0
xdes = Tdes(1:3,1);
ydes = Tdes(1:3,2);
zdes = Tdes(1:3,3);
pdes = Tdes(1:3,4);

%Criação dos eixos da posicao desejada
E = Eixo(Tdes, 0.2,{'xdes', 'ydes', 'zdes'});
C.adicionaobjetos(E);

%Desenho dos elementos do cenário
C.desenha();

pause();

%% Movimentação da posição desejada do efetuador
% Percorre o eixo X da parede
for x = 0:60
    % Novo valor para o X desejado
    newxdes = -1.5+x/20;
    % Percorre o eixo Y da parede
    for z = 0:80
        % Novo valor para o Z desejado
        newzdes = z/100;
        % Verificando se a posição em questão não é a janela
        if ((newxdes < -0.5 || newxdes > 0.5) || newzdes > 0.6)
            % Tdes assume novos valores
            Tdes = Robo.desl([newxdes; 0.6; newzdes]);
            % Criação do eixo da posição desejada com os novos valores
            E = Eixo(Tdes, 0.2,{'xdes', 'ydes', 'zdes'});
            % As próximas três linhas devem ser substituídas pelo movimento
            % do robô, mas neste momento estão aqui para mostrar na figura
            % que o frame desejado percorre toda a parede, com exceção da
            % porta.
            C.adicionaobjetos(E);
            C.desenha();
            drawnow;
        end
    end
end
%% Movimentação do robô

% Constantes
dt = 0.01;
K = 7;

% Historicos
q_hist=[];
r_hist=[];
u_hist=[];
t=[];

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
