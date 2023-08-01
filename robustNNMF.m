function [W,H,d] = robustNNMF(A,k)


opt = statset('MaxIter',50,'Display','off');
[W,H] = nnmf(A,k,'replicates',5,...
    'options',opt,...
    'algorithm','mult');

options = optimset('display','off','Algorithm','sqp','useparallel','never');

for iter=1:3
    for i=1:size(A,2), H(:,i) = lsqnonneg(W,A(:,i)); end
    W = fmincon(@(W)(norm(A-W*H,'fro')),W,[],[],[],[],zeros(size(A,1),k),1e3*ones(size(A,1),k),[],options);
    opt = statset('Maxiter',1000,'Display','off', 'TolFun',1e-9,'TolX',1e-9);
        [W,H,d] = nnmf(A,k,'w0',W,'h0',H,...
          'options',opt,...
          'algorithm','als');
end

display(['NNMF solution reached. Error = ' num2str(d)])