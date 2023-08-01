% X toward front; Z toward the right
clearvars
clc


sys = armParameters;

ampAcc = 5;
ampF = 0;

SHjoint = [0,0];
param =  [0; 0]; % [Elbow damping; Shoulder damping]


thetaSHset = (0:10:120)/180*pi;
thetaELset = (10:10:150)/180*pi;
thetaAccSet = 0:pi/12:2*pi-pi/12;
thetaFset = 0:pi/12:2*pi-pi/12;



%% Activation optimization

activationSet = zeros(6,length(thetaFset),length(thetaSHset),length(thetaELset));

Hand = [];
tic
for i = 1:length(thetaSHset)
    for j = 1:length(thetaELset)
        thetaSH = thetaSHset(i)
        thetaEL = thetaELset(j)
        U = [];
        for k = 1:length(thetaFset)
            thetaAcc = thetaAccSet(k);
            thetaF = thetaFset(k);
            refAcc = ampAcc*[cos(thetaAcc); sin(thetaAcc)];
            handForce = ampF*[cos(thetaF); sin(thetaF)];
            uOpt = SolveOptimalActivation(thetaSH, thetaEL, handForce, refAcc);
            U = [U, uOpt];
        end
        activationSet(:,:,i,j) = U;

        ELjoint = SHjoint + [sys.a1.*cos(thetaSH), sys.a1.*sin(thetaSH)];
        Hand = [Hand; ELjoint + [sys.a2.*cos(thetaSH+thetaEL), sys.a2.*sin(thetaSH+thetaEL)]];

    end
end
toc

Hand = [0,0; Hand];

figure(10); plot(Hand(:,1),Hand(:,2),'o'); axis equal
figure(20); polarplot(thetaFset,U,'linewidth',2); title('optimal activations in the last pose'); 

%% find the synergies
NoSyn = 4;
useConcatenated = true;

synergySet = zeros(6,NoSyn,length(thetaSHset),length(thetaELset));

if useConcatenated
    A_cnnmf = [];
    for i = 1:length(thetaSHset)
        for j = 1:length(thetaELset)
            A_cnnmf = [A_cnnmf; activationSet(:,:,i,j)];
        end
    end
    [Syn_tmp,~] = nnmf(A_cnnmf,NoSyn);
    for i = 1:length(thetaSHset)
        for j = 1:length(thetaELset)
            synergySet(:,:,i,j) = Syn_tmp(1:6,:);
            Syn_tmp(1:6,:) = [];
        end
    end

else
    for i = 1:length(thetaSHset)
        for j = 1:length(thetaELset)
            [Syn,Coeff] = robustNNMF(activationSet(:,:,i,j),NoSyn);
            synergySet(:,:,i,j) = Syn;
            A_cnnmf = [A_cnnmf;activationSet(:,:,i,j)];
        end
    end
end


save("synergies","synergySet",'thetaELset','thetaSHset')


%% %%% interpolate the synergies
tic
thetaSH = 1.2;
thetaEL = 1.2;
omegaSH = 1;
omegaEL = 0;

F_pert = 0*[5;5];

Syn = interpn(1:6,1:NoSyn,thetaSHset,thetaELset,  synergySet  ,1:6,1:NoSyn,thetaSH,thetaEL);

[basis_acc,basis_F] = calcBasis(Syn,thetaSH,thetaEL,param);


%%% Regenerate the reference acceleration



U=[];
uOpt=[];
Acc_norm=[];
Acc_angle=[];
F_norm=[];
F_angle=[];



desiredDirectionSet = (-180:5:180)*pi/180;
for desiredDirection = desiredDirectionSet
    
    refernceVector = [cos(desiredDirection);sin(desiredDirection)];
    refAcc = ampAcc*refernceVector;
    refAcc = CorrectAccRef(refAcc,thetaSH,thetaEL,omegaSH,omegaEL,sys,F_pert);
    handForce = ampF*refernceVector;

%     coeff = lsqnonneg(basis_F,handForce);
    coeff = lsqnonneg(basis_acc,refAcc);

    
    u = Syn*coeff;
    U = [U, u];
    IC = getPlanarArmIC(thetaEL, omegaEL, thetaSH, omegaSH, u, param);
    [~, ~, y] = PlanarArm(0,IC,[-handForce(2)+F_pert(2); -handForce(1)+F_pert(1);u],param);
    RealAcc = getPlanarArmOutputs(y).hand_a;
    Acc_norm = [Acc_norm; norm(RealAcc)];
    Acc_angle = [Acc_angle; atan2(RealAcc(2), RealAcc(1))];
    RealF = calculateTaskSpaceForce(u,[thetaSH;thetaEL],[omegaSH;omegaEL],[-handForce(2)+F_pert(2); -handForce(1)+F_pert(1)],param);
    F_norm = [F_norm; norm(RealF)];
    F_angle = [F_angle; atan2(RealF(2), RealF(1))];

    
    
    uOpt = [uOpt,SolveOptimalActivation(thetaSH, thetaEL, -handForce, refAcc)];
    
end
toc
figure(30); clf;
subplot(2,1,1); 
polarplot(desiredDirectionSet,uOpt','linewidth',2); 
title('optimal'); 
rlim([0,.4])
legend('Biarticular extensor','Biarticular flexor','Elbow extensor','Elbow flexor','Shoulder extensor','Shoulder flexor','location','best');

subplot(2,1,2); 
polarplot(desiredDirectionSet,U','linewidth',2); hold on
polarplot((basis_acc(1,:)+1i*basis_acc(2,:))*0.01','o')
polarplot((basis_F(1,:)+1i*basis_F(2,:))*0.01','*')
% compass(atan2(basis(2,:),basis(1,:)),[1,1,1])
title('synergy'); 
rlim([0,.4])
set(findall(gcf,'type','text'),'FontSize',9)
set(findall(gcf,'type','axes'),'fontsize',9)
% figure(2); plot(thetaAccSet,A);
% figure(1); plot(thetaAccSet,angle);


figure(40); clf
polarplot(Acc_norm.*[cos(Acc_angle)+1i*sin(Acc_angle)])
hold all
polarplot(F_norm.*[cos(F_angle)+1i*sin(F_angle)])
title('actual produced vectors')
legend('acc','F')
