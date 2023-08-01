function [K,JC] = getGainsKalmanFilter(A,B,C,D,H,Omega_xi,Omega_omega,Omega_eta,L,Sigma_x,Sigma_e)

%   Writtent by F. Crevecoeur - Spet. 6, 2019
%   Used in: Robust control in human reaching movements: a model free
%   strategy to compensate for unpredictable disturbances. 
%   Crevecoeur F., Scott S. H., Cluff T. 
%   DOI: https://doi.org/10.1523/JNEUROSCI.0770-19.2019

n = size(A,1);
k = size(H,1);
d = size(D,3);
c = size(C,3);
nStep = size(L,3);
Sigma_ex = zeros(n);
if size(A,3)==1
    A = repmat(A,1,1,nStep);
end
K = zeros(n,k,nStep);
JC = zeros(nStep,3);

for i = 1:nStep
    
    sTemp = (Sigma_e + Sigma_x + Sigma_ex + Sigma_ex');
    
    sdn = 0; % state-dependent term
    for j = 1:d
        sdn = sdn + D(:,:,j)*sTemp*D(:,:,j)';
    end
    
    cdn = 0; % control-dependent term
    for j = 1:c
        cdn = cdn + C(:,:,j)*L(:,:,i)*Sigma_x*L(:,:,i)'*C(:,:,j)';
    end 
    
    K(:,:,i) = A(:,:,i)*Sigma_e*H'/(H*Sigma_e*H'+Omega_omega+sdn);

    Sigma_e_temp = Sigma_e;
    Sigma_e = Omega_xi+Omega_eta+(A(:,:,i)-K(:,:,i)*H)*Sigma_e*A(:,:,i)'+cdn;
    term = (A(:,:,i)-B*L(:,:,i))*Sigma_ex*H'*K(:,:,i)';
    Sigma_x = Omega_eta + K(:,:,i)*H*Sigma_e_temp*A(:,:,i)'+(A(:,:,i)...
        - B*L(:,:,i))*Sigma_x*(A(:,:,i)-B*L(:,:,i))'...
        + term + term';
    Sigma_ex = (A(:,:,i)-B*L(:,:,i))*Sigma_ex*(A(:,:,i)-K(:,:,i)*H)'-Omega_eta;
    
    JC(i,:) = [Sigma_e(1,1),sTemp(1,1),0];
    
end


    

