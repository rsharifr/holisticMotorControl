classdef OFC < handle
    properties
        simSetting
        costFunction
        systemEq
        optimalGains
        simulationResults
        noiseConstructors
    end
    properties(GetAccess = private)
        
    end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    methods
        %% CONSTRUCTOR
        function ofc = OFC(A,B,H,Q,R,simSetting,noiseStructure)
            ofc.simSetting = simSetting;
            deltaT = simSetting.deltaT;
            if isempty(ofc.simSetting.A_sim)
                A_sim = A;
            else
                A_sim = ofc.simSetting.A_sim;
            end
            
            if any(size(A)~=size(A_sim))
                error('Dimension mismatch between "A" and "A_sim"')
            end
            A_discrete = eye(size(A,2))+deltaT*A;
            A_sim_discrete = eye(size(A_sim,2)) + deltaT*A_sim;
            B_discrete = deltaT*B;
            
            ofc.systemEq.A = A_discrete;
            ofc.systemEq.A_sim = A_sim_discrete;
            ofc.systemEq.B = B_discrete;
            ofc.systemEq.H = H;
            ofc.systemEq.numberOfStates = size(A_discrete,2);
            ofc.systemEq.numberOfControls = size(B_discrete,2);
            ofc.systemEq.numberOfOutputs = size(H,1);
            ofc.costFunction.Q = Q;
            ofc.costFunction.R = R;
            
            ofc.augmentSystemWithSensoryDelay();
            ofc.setupNoiseConstructors(noiseStructure);
        end


        %% AUGMENT SYSTEM EQ WITH SENSORY DELAY
        function [A,B,Q,H] = augmentSystemWithSensoryDelay(ofc)

            A0 = ofc.systemEq.A;
            A0_sim = ofc.systemEq.A_sim;
            B0 = ofc.systemEq.B;
            H0 = ofc.systemEq.H;
            if ~isempty(ofc.costFunction)
                Q0 = ofc.costFunction.Q;
            else
                error('Q matrix is empty')
            end
            h = ofc.simSetting.delay;

            n = size(A0,1);
            nSteps_A = size(A0,3);
            m = size(B0,2);
            nStep = size(Q0,3);
            p = size(H0,1);

            A = zeros((h+1)*n,(h+1)*n,nSteps_A);
            A_sim = zeros((h+1)*n,(h+1)*n,nSteps_A);
            B = zeros((h+1)*n,m);
            Q = zeros((h+1)*n,(h+1)*n,nStep);
            H = zeros(p,(h+1)*n);

            for i = 1:nSteps_A
                A(1:n,1:n,i) = A0(:,:,i);
                A(n+1:end,1:end-n,i) = eye(h*n);

                A_sim(1:n,1:n,i) = A0_sim(:,:,i);
                A_sim(n+1:end,1:end-n,i) = eye(h*n);
            end
            B(1:n,:) = B0;
            H(:,end-n+1:end) = H0;

            % Adding h times the constraint Q1:
            Qaug = zeros(n,n,nStep+h);
            for i = 1:h
                Qaug(:,:,i) = Q0(:,:,1);
            end
            for t = 1:nStep
                Qaug(:,:,t+h) = Q0(:,:,t);
            end

            %Filling the diagonal Q matrices
            for t = 1:nStep
                for i = 0:h
                    Q(i*n+1:(i+1)*n,i*n+1:(i+1)*n,t) = Qaug(:,:,t+h-i)/(h+1);
                end
            end

            ofc.systemEq.A = A;
            ofc.systemEq.A_sim = A_sim;
            ofc.systemEq.B = B;
            ofc.systemEq.H = H;
            ofc.costFunction.Q = Q;
            ofc.systemEq.numberOfStates = size(A,2);
            ofc.systemEq.numberOfControls = size(B,2);
            ofc.systemEq.numberOfOriginalStates = size(A0,2);

        end


        %% CALCULATE OPTIMAL GAINS
        function [L,K,newCost,Sx,Se,s,JC] = getOptimalGains(ofc)

            % The output matrices are:
            %   L                   time series of optimal feedback gains
            %   K                   time series of non adaptive Kalmen gains
            %   NEWCOST             expected cost after stopping the iterations
            %   SX, SE              series of cost matrices determined by the backwards
            %                       recurrences for the state (SX) and error (SE) terms
            %   S                   scalar component of the total expected cost
            %   JC                  covariance matrix


            A = ofc.systemEq.A;
            B = ofc.systemEq.B;
            H = ofc.systemEq.H;
            Q = ofc.costFunction.Q;
            R = ofc.costFunction.R;

            n = ofc.systemEq.numberOfStates;
            p = ofc.systemEq.numberOfOutputs;
            nStep = ofc.simSetting.nStep;
            xInit = ofc.simSetting.xInit;

            C = ofc.noiseConstructors.controlDependentConstructor;
            D = ofc.noiseConstructors.stateDependentConstructor;
            Omega_xi = ofc.noiseConstructors.additiveProcessNoiseCovar;
            Omega_omega = ofc.noiseConstructors.sensoryNoiseCovar;
            Omega_eta = ofc.noiseConstructors.internalNoiseCovar;
            Sigma_e = ofc.noiseConstructors.errorEstimateCovar;
            Sigma_x = ofc.noiseConstructors.stateEstimateCovar;


            h = ofc.simSetting.delay;
            xInit = repmat(xInit,h+1,1);

            K = zeros(n,p,nStep);

            tol = 1e-14;
            current = 1e6;
            itmax = 500;
            dCost = 1e6;
            count = 0;

            fprintf("Solving for OFC's optimal gains...")
            tic

            % The optimal control and Kalman gains are calculated iteratively (no more
            % than itmax times if it does not converge)
            while (dCost>tol) && (count<itmax)

                [L,Sx,Se,s] = getGainsController(ofc,A,B,C,D,H,Q,R,K,Omega_xi,Omega_omega,Omega_eta);
                [K,JC] = getGainsKalmanFilter(ofc,A,B,C,D,H,Omega_xi,Omega_omega,Omega_eta,L,Sigma_x,Sigma_e);

                % Expected cost
                newCost = xInit'*Sx*xInit + trace((Sx+Se)*Sigma_e)+ s;

                % Relative improvement
                dCost = abs(current - newCost)/newCost;
                current = newCost;

                count = count + 1;

            end
            elapsedTime = toc;
            fprintf('Done. Number of iterations: %d, elapsed time %.3f \n', count, elapsedTime); % Number of iterations

            ofc.optimalGains.L = L;
            ofc.optimalGains.K = K;
        end
        
        %% GET CONTROLLER GAIN, L
        function [L,Sx,Se,s] = getGainsController(ofc,A,B,C,D,H,Q,R,K,Omega_xi,Omega_omega,Omega_eta)

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
        end

        %% GET KALMAN FILTER GAIN K
        function [K,JC] = getGainsKalmanFilter(ofc,A,B,C,D,H,Omega_xi,Omega_omega,Omega_eta,L,Sigma_x,Sigma_e)

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
        end

        %% SETUP NOISE STRUCTURES
        function [C,D,Omega_xi,Omega_omega,Omega_eta,Sigma_e,Sigma_x] = setupNoiseConstructors(ofc,noiseStructure)
            c = noiseStructure.c;
            d = noiseStructure.d;
            xi = noiseStructure.xi;
            omega = noiseStructure.omega;
            eta = noiseStructure.eta;

            B = ofc.systemEq.B;
            n = ofc.systemEq.numberOfStates;
            m = ofc.systemEq.numberOfControls;
            p = ofc.systemEq.numberOfOutputs;
            C = zeros(n,m,m);% control-dependent noise constructor
            for i = 1:m
                Cprime = zeros(m);
                Cprime(i,i) = c;
                C(:,:,i) = B*Cprime;
            end
            D = d*zeros(p,n,1); % state-dependent noise constructor

            Omega_xi = zeros(size(B*B'));
            Omega_xi((B*B')~=0) = xi; % covariance of additive process noise (only for control-affected states)
            % Omega_xi(6,6) = xi; % FOR 1D cup task. this is important to allow the observer to expect disturbances
            % Omega_xi(7:8,7:8) = Omega_xi(5:6,5:6); % FOR 2D REACH MODEL. this is important to allow the observer to expect disturbances
            Omega_omega = omega*eye(p); % covariance of sensory noise
            Omega_eta = eta*eye(n); % covariance of internal noise

            Sigma_e = eye(n)*10^-4; % initial unconditional error estimation covariance
            Sigma_x = eye(n)*10^-2; % initial unconditional state estimation covariance


            ofc.noiseConstructors.controlDependentConstructor = C;
            ofc.noiseConstructors.stateDependentConstructor = D;
            ofc.noiseConstructors.additiveProcessNoiseCovar = Omega_xi;
            ofc.noiseConstructors.sensoryNoiseCovar = Omega_omega;
            ofc.noiseConstructors.internalNoiseCovar = Omega_eta;
            ofc.noiseConstructors.errorEstimateCovar = Sigma_e;
            ofc.noiseConstructors.stateEstimateCovar = Sigma_x;

        end



        %% SIMULATE INTERNAL MODEL
        function [x_out,xest_out,u_out,Sigma_x_out,Sigma_e_out,Sigma_ex_out] = simulateMotion(ofc)

            nStep = ofc.simSetting.nStep;
            n = ofc.systemEq.numberOfStates;
            m = ofc.systemEq.numberOfControls;
            p = ofc.systemEq.numberOfOutputs;
            xInit = ofc.simSetting.xInit;

            h = ofc.simSetting.delay;
            A = ofc.systemEq.A;
            A_sim = ofc.systemEq.A_sim;
            B = ofc.systemEq.B;
            H = ofc.systemEq.H;

            if size(A,3) == 1
                A = repmat(A,1,1,nStep);
                A_sim = repmat(A_sim,1,1,nStep);
            end

            C = ofc.noiseConstructors.controlDependentConstructor;
            D = ofc.noiseConstructors.stateDependentConstructor;
            Omega_xi = ofc.noiseConstructors.additiveProcessNoiseCovar;
            Omega_omega = ofc.noiseConstructors.sensoryNoiseCovar;
            Omega_eta = ofc.noiseConstructors.internalNoiseCovar;



            Q = ofc.costFunction.Q;
            R = ofc.costFunction.R;
            K = ofc.optimalGains.K;
            L = ofc.optimalGains.L;

            for iter = 1:ofc.simSetting.nsimu
                cost = 0;
                currentX = repmat(xInit,h+1,1);
                currentXEst = currentX;
                x = zeros(nStep,n);
                x(1,:) = currentX(1:n)';
                xest = x;
                u = zeros(nStep-1,m);
                Sigma_ex = zeros(n,n,nStep);
                Sigma_e = repmat(ofc.noiseConstructors.errorEstimateCovar,1,1,nStep);
                Sigma_x = repmat(ofc.noiseConstructors.stateEstimateCovar,1,1,nStep);

                for i = 1:nStep-1

                    sensoryNoise = mvnrnd(zeros(p,1),Omega_omega)';
                    processNoise = mvnrnd(zeros(n,1),Omega_xi)';
                    %         processNoise(7:8) = zeros(2,1); % FOR 2D REACH MODEL. The perturbation is deterministic. Does not matter much
                    internalNoise = mvnrnd(zeros(n,1),Omega_eta)';

                    stateDependentNoise = 0;
                    for isdn = 1:size(D,3)
                        stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*x(i,:)';
                    end

                    yz = H*currentX + sensoryNoise + stateDependentNoise;
                    u(i,:) = (-L(:,:,i)*currentXEst)';

                    currentXEst = A(:,:,i)*currentXEst + B*u(i,:)' + K(:,:,i)*(yz-H*currentXEst) + internalNoise;

                    % Cost LQG
                    cost = cost + currentX'*Q(:,:,i)*currentX + u(i,:)*R(:,:,i)*u(i,:)';

                    %control-dependent noise
                    controlDependentNoise = 0;
                    for icdn = 1:m
                        controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*u(i,:)';
                    end

                    currentX = (A_sim(:,:,i))*currentX + B*u(i,:)' + processNoise + controlDependentNoise;



                    x(i+1,:) = currentX(1:n)';
                    xest(i+1,:) = currentXEst(1:n)';

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    sTemp = (Sigma_e(:,:,i) + Sigma_x(:,:,i) + Sigma_ex(:,:,i) + Sigma_ex(:,:,i)');

                    sdn = 0; % state-dependent term
                    for j = 1:size(D,3)
                        sdn = sdn + D(:,:,j)*sTemp*D(:,:,j)';
                    end

                    cdn = 0; % control-dependent term
                    for j = 1:size(C,3)
                        cdn = cdn + C(:,:,j)*L(:,:,i)*Sigma_x(:,:,i)*L(:,:,i)'*C(:,:,j)';
                    end

                    Sigma_e(:,:,i+1) = Omega_xi+Omega_eta+(A(:,:,i)-K(:,:,i)*H)*Sigma_e(:,:,i)*A(:,:,i)'+cdn;
                    term = (A(:,:,i)-B*L(:,:,i))*Sigma_ex(:,:,i)*H'*K(:,:,i)';
                    Sigma_x(:,:,i+1) = Omega_eta + K(:,:,i)*H*Sigma_e(:,:,i)*A(:,:,i)'+(A(:,:,i)-B*L(:,:,i))*Sigma_x(:,:,i)*(A(:,:,i)-B*L(:,:,i))'...
                        + term + term';
                    Sigma_ex(:,:,i+1) = (A(:,:,i)-B*L(:,:,i))*Sigma_ex(:,:,i)*(A(:,:,i)-K(:,:,i)*H)'-Omega_eta;


                end

                x = x(:,1:ofc.systemEq.numberOfOriginalStates);
                xest = xest(:,1:ofc.systemEq.numberOfOriginalStates);

                x_out{iter} = x;
                xest_out{iter} = xest;
                u_out{iter} = u;
                Sigma_e_out{iter} = Sigma_e;
                Sigma_x_out{iter} = Sigma_x;
                Sigma_ex_out{iter} = Sigma_e;
                cost_out{iter} = cost;

            end
            ofc.simulationResults.x = x_out;
            ofc.simulationResults.xest = xest_out;
            ofc.simulationResults.u = u_out;
        end
        

    end
end