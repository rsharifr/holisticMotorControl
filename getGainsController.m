function [L,Sx,Se,s] = getGainsController(A,B,C,D,H,Q,R,K,Omega_xi,Omega_omega,Omega_eta)

%   Writtent by F. Crevecoeur - Spet. 6, 2019
%   Used in: Robust control in human reaching movements: a model free
%   strategy to compensate for unpredictable disturbances. 
%   Crevecoeur F., Scott S. H., Cluff T. 
%   DOI: https://doi.org/10.1523/JNEUROSCI.0770-19.2019

n = size(A,1);
m = size(B,2);
c = size(C,3);
d = size(D,3);
nStep = size(R,3);
L = zeros(m,n,nStep);
if size(A,3)==1
    A = repmat(A,1,1,nStep);
end

currSx = Q(:,:,end);
currSe = 0;
currs = 0;

for i = nStep:-1:1
    
    cdn = 0; % control-dependent term
    for j = 1:c
        cdn = cdn + C(:,:,j)'*(currSx + currSe)*C(:,:,j);
    end
    
    sdn = 0; % state-dependent term
    for j = 1:d
        sdn = sdn + D(:,:,j)'*K(:,:,i)'*currSe*K(:,:,i)*D(:,:,j);
    end

    L(:,:,i) = (R(:,:,i) + B'*currSx*B + cdn)\(B'*currSx*A(:,:,i));
    currSxTemp = currSx;
    currSeTemp = currSe;
    
    currSx = Q(:,:,i) + A(:,:,i)'*currSx*(A(:,:,i)-B*L(:,:,i)) + sdn;
    currSe = A(:,:,i)'*currSxTemp*B*L(:,:,i)+...
        (A(:,:,i)-K(:,:,i)*H)'*currSeTemp*(A(:,:,i)-K(:,:,i)*H);
    currs = trace(currSxTemp*Omega_xi+currSeTemp*(...
        Omega_xi+Omega_eta+K(:,:,i)*Omega_omega*K(:,:,i)'))+currs;

end

Sx = currSx;
Se = currSe;
s = currs;