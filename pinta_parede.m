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
MTH=Robo.desl([0;0.75;0.4]);
% Dimensionamento da parede
lados = [3 0.1 0.8];
% Coloração da parede (RGB)
cor = [0.9 0.9 0.9];
% Densidade da parede
densidade = 1;
% Criação da parede
P=Paralelepipedo(MTH,lados,cor,densidade);

% Posicionamento do centro da janela
MTHJ=Robo.desl([0;0.73;0.4]);
% Dimensionamento da janela
ladosJ = [1 0.1 0.8];
% Coloração da janela (RGB)
corJ = [0.3 0.3 0.7];
% Criação da janela
J=Paralelepipedo(MTHJ,ladosJ,corJ,densidade);

% Criação do robô
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

% Criação do cenário
C = Cenario(R);
% Adição da  parede
C.adicionaobjetos(P);
% Adição da janela
C.adicionaobjetos(J);
% Adição do robô
C.adicionaobjetos(R);
C.adicionaobjetos(pincel);

% Criação da posição inicial desejada do efetuador
Tef0= R.cinematicadir(R.q, 'efetuador');
Tdes = Robo.desl([-1.4; 0.6; 0])*Robo.rot('y',-pi/2)*Robo.rot('x',-pi/2);
xdes = Tdes(1:3,1);
ydes = Tdes(1:3,2);
zdes = Tdes(1:3,3);
pdes = Tdes(1:3,4);

%Criação dos eixos da posicao desejada
E = Eixo(Tdes, 0.2,{'xdes', 'ydes', 'zdes'});
%C.adicionaobjetos(E);

%Desenho dos elementos do cenário
C.desenha();

pause();

%% Parâmetros de controle %%

K=10;
deltaT=0.01;
q=R.q;
alpha=0.001;
rh=[];
k=1;

%% Posicionamento inicial do robô %%

movimenta base do robo para x=-1.2
for i = 1: 120
    %Atualiza a mth do Robo
    MTH=Robo.desl([-0.01*i;0;0]);
    R.mth = MTH;
    R.config(R.q);
    C.desenha();
    drawnow;
end

%posiciona o pincel na parte inferior esquerda da parede
for k = 1: 2/deltaT %tempo em s/dt
    q_hist(:,k) = R.q;
	t(k) = (k-1)*deltaT;

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

    u = Robo.pinvam(Jr,0.01)*(-K*r);

    u_hist(:,k) = u;
    r_hist(:,k) = r;

    qprox = R.q + u*deltaT;
    R.config(qprox);
    C.desenha();
    drawnow;    
end

%% Loop principal do algorítmo %%

q=q_hist;

for x = 2: 30
    for z = 0: 15
        %Evitando a porta
        if x<10 || x>20
            %Calculando o vetor de tarefa e a Jacobiana
            if mod(x,2)==0
                Tdes= Robo.desl([-1.5+0.1*x;0.6;z/10])*Robo.rot('y',-pi/2)*Robo.rot('x',-pi/2);
            else
                Tdes= Robo.desl([-1.5+0.1*x;0.6;(8-z)/10])*Robo.rot('y',-pi/2)*Robo.rot('x',-pi/2);
            end
            [r,Jr] = functarefa2(R,q(:,k),Tdes);
            %Guarda o hist´orico de vetor de erros
            rh=[rh r];
            %Calculando a velocidade que deve ser integrada
            qdot = Robo.pinvam(Jr,alpha)*(-K*r);
            %Criando o pr´oximo k por integra¸c~ao
            q(:,k+1) = q(:,k)+deltaT*qdot;
            %Coloca no rob^o
            R.config(q(:,k+1));
            %Desenha
            C.desenha();
            drawnow;
            k=k+1;
        end
    end
    
    %movimentação da base do robo no eixo x
    if x == 10      %quando chega proximo à porta, movimenta até sair da porta
        if(R.mth(1,4)<0.8)
            %Atualiza a mth do Robo
            MTH=Robo.desl([R.mth(1,4)+0.01;0;0]);
            R.mth = MTH;
            R.config(R.q);
            C.desenha();
            drawnow;
        end
    else    %quando não esta na área da porta, movimenta 10cm por vez
        for i = 1: 10
            %Atualiza a mth do Robo
            MTH=Robo.desl([R.mth(1,4)+0.01;0;0]);
            R.mth = MTH;
            R.config(R.q);
            C.desenha();
            drawnow;
        end
    end
end

%%

function [r,Jr] = functarefa2(R,q,Tdes)
%Vamos montar a referencia desejada no tempo
px_des = Tdes(1,4);
py_des = Tdes(2,4);
pz_des = Tdes(3,4);
ex_des = Tdes(1:3,1);
ey_des = Tdes(1:3,2);
ez_des = Tdes(1:3,3);

%Vamos extrair os valores atuais para essas vari´aveis baseados
%na cinematica direta. Vamos tambem calcular a Jacobiana, que ser´a
%necessaria
[J,CD]=R.jacobianageo(q,'efetuador');
px = CD(1,4);
py = CD(2,4);
pz = CD(3,4);
ex = CD(1:3,1);
ey = CD(1:3,2);
ez = CD(1:3,3);
%Vamos calcular o vetor de tarefa, um vetor COLUNA
r = [px-px_des; py-py_des; pz-pz_des; 1-(ex_des)'*ex; 1-(ey_des)'*ey; 1-(ez_des)'*ez];
%Vamos calcular a Jacobiana da tarefa:
Jr = [J(1:3,:); (ex_des)'*Robo.matrizprodv(ex)*J(4:6,:);(ey_des)'*Robo.matrizprodv(ey)*J(4:6,:);(ez_des)'*Robo.matrizprodv(ez)*J(4:6,:)];
end
