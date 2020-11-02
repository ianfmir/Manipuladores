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
