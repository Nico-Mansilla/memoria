classdef mheOptiLoss3 < handle
    properties (GetAccess = public, SetAccess = private)
        % system's properties
        Ne, nq, nu, ny, Ts, Nt;
        % Numerical values of the optimal solution
        Qtraj, Utraj, Ytraj, Wtraj, Vtraj, q0bar, qkAux, qk, mtxW, mtxWPrm, mtxR, mtxP, v_kminusNe, sigmaP, cP; % mtxR was used for adapting the AC with Kalman
        % Solver's parameters
        q_lb, q_ub, u_lb, u_ub; 
        % 
        Alin, Blin, Clin, S, K, jac_fx, jac_fu, jac_hx, lastInput;
        %
        Fsym, hsym;
        %
        optSolver, optiMHE, solutionMHE;
        % MHE parameters:
        Y, U, qbar, P, Lh, alphaPrm, alphaPrmVal, cPrm, cPrmVal, fLossV1, jacV;
        JmheQ, JmheQVal;
        % MHE variables:
        Q, w, v, X;
        % Quadratic (standard) MHE
        solutionMHEalpha, optiMHEalpha;
    end
    
    methods
        function obj = mheOptiLoss3(Ne,Nt,F,h,dims,boxConst,Ts)
            obj.Ne              = Ne;         
            obj.Nt              = Nt;
            obj.nq              = dims.nq;
            obj.nu              = dims.nu;
            obj.ny              = dims.ny;
            obj.Ts              = Ts;           
            obj.Fsym            = F;
            obj.hsym            = h;
            obj.Qtraj           = zeros(obj.nq,obj.Ne+1);
            obj.Utraj           = zeros(obj.nu,obj.Ne);
            obj.Ytraj           = zeros(obj.ny,obj.Ne+1);
            obj.q0bar           = zeros(obj.nq,1);
            obj.qk              = zeros(obj.nq,1);
            obj.mtxP            = 1e10.*eye(obj.nq);
            obj.v_kminusNe      = ones(obj.ny,1);
            obj.lastInput       = zeros(obj.nu,1);
            %
            obj.optSolver       = 'ipopt'; % qpoases, ipopt
            % ***********************************************************            
            obj.optiMHE         = casadi.Opti();
            obj.Y               = obj.optiMHE.parameter(obj.ny,obj.Ne+1);  % measurements
            obj.U               = obj.optiMHE.parameter(obj.nu,obj.Ne);    % past inputs
            obj.qbar            = obj.optiMHE.parameter(obj.nq);           % prior knowledge at the beginning of the estimation window
            obj.P               = obj.optiMHE.parameter(obj.nq,obj.nq);    % arrival-cost matrix
            obj.Lh              = obj.optiMHE.parameter(obj.Nt,2);
            obj.alphaPrm        = obj.optiMHE.parameter;
            obj.cPrm            = obj.optiMHE.parameter;
            obj.mtxWPrm         = obj.optiMHE.parameter(obj.nq,obj.nq);
            % Init paarmetrs with tribial values:
            obj.optiMHE.set_value(obj.Y,obj.Ytraj);
            obj.optiMHE.set_value(obj.U,obj.Utraj);
            obj.optiMHE.set_value(obj.qbar,obj.q0bar);
            obj.optiMHE.set_value(obj.P,inv(obj.mtxP));
            % OPtimisation variables of MHE1_____________________________
            obj.Q               = obj.optiMHE.variable(obj.nq,obj.Ne+1);        % state trajectory
            obj.w               = obj.optiMHE.variable(obj.nq,obj.Ne);          % disturbances trajectory
            obj.v               = obj.optiMHE.variable(obj.ny,obj.Ne+1);        % residuals of the estimation
            obj.X               = obj.optiMHE.variable(obj.nq);                 % arrival-cost          
            %
            vSym                = casadi.MX.sym('x');
            aSym                = casadi.MX.sym('alpha');
            cSym                = casadi.MX.sym('c');            
            rhsf                = (abs(aSym-2)/(aSym)) * ( ((((vSym/cSym)^2/(abs(aSym-2)))+1)^(aSym/2)) - 1);
            jacRhsf             = rhsf.jacobian(vSym);
            obj.jacV            = casadi.Function('jacV', {vSym,aSym,cSym}, {jacRhsf});
            obj.fLossV1         = casadi.Function('fLossV', {vSym,aSym,cSym}, {rhsf});
            %
            obj.optiMHE.subject_to([-ones(obj.Nt+1,1);-inf(2,1)] <= obj.v <= [ones(obj.Nt+1,1);inf(2,1)]);
            obj.optiMHE.subject_to(-0.01.*ones(obj.nq,1) <= obj.w <= 0.01.*ones(obj.nq,1));
            %
            obj.JmheQ = (obj.X') * obj.P * obj.X;                                                                               % arrival-cost
            obj.optiMHE.subject_to(obj.X == obj.Q(:,1)-obj.qbar);                                                           % arrival-cost constraint
            for i=1:obj.Ne+1
                if i<obj.Ne+1
                    obj.JmheQ = obj.JmheQ + ( (obj.w(:,i)' * (obj.mtxWPrm)) * obj.w(:,i) );
                    obj.optiMHE.subject_to( obj.Q(:,i+1) == obj.Fsym(obj.Q(:,i),obj.U(:,i)) + obj.w(:,i) );                    
                end
                %
                for j=1:obj.Nt
                    obj.optiMHE.subject_to( obj.Q(j,i) == obj.Q(obj.Nt+j,i)-obj.Q(obj.Nt+j+1,i) );
                end
                %
                for j=1:obj.ny
                    obj.JmheQ = obj.JmheQ + obj.jacV( obj.v(j,i), obj.alphaPrm, obj.cPrm )^2;
                end
                obj.optiMHE.subject_to( obj.Y(:,i) == obj.hsym(obj.Q(:,i)) + obj.v(:,i) );                                  % measurements contrtraints
                % Geometrical constraints of the chained vehicle:
                xy_acum = [0;0];
                if obj.Nt>0
                    for k=0:obj.Nt-1
                        xy_acum = xy_acum + [obj.Lh(end-k,1)*cos(obj.Q(obj.Nt+k+1,i)) + obj.Lh(end-k,2)*cos(obj.Q(obj.Nt+k+2,i));... 
                                             obj.Lh(end-k,1)*sin(obj.Q(obj.Nt+k+1,i)) + obj.Lh(end-k,2)*sin(obj.Q(obj.Nt+k+2,i))];
                        aux_var = obj.Q(2*obj.Nt+2:2*obj.Nt+3,i) - xy_acum;
                        obj.optiMHE.subject_to( obj.Q(2*obj.Nt+1+(k+1)*2+1:2*obj.Nt+1+(k+2)*2,i) == aux_var );
                    end       
                end
            end
            obj.optiMHE.minimize(obj.JmheQ);
            p_optsMHE1 = struct('expand',true);
            s_optsMHE1 = struct('sb','no','print_level',0,'gamma_theta',1e-12,'jacobian_approximation','exact','max_iter',10000,'tol',1e-8,'warm_start_init_point','no');%, ,'gamma_theta',1e-2, 'fast_step_computation','yes','jacobian_approximation','exact','warm_start_init_point','yes'); % 
            obj.optiMHE.solver('ipopt',p_optsMHE1,s_optsMHE1);
            % *************************************************************
            % list of parameters to be tuned: https://coin-or.github.io/Ipopt/OPTIONS.html
        end       

        function setVehicleDims(obj,dims)
            obj.optiMHE.set_value(obj.Lh,dims);
        end

        function setJacobians(obj,jacFx,jacFu,jacHx)
            obj.jac_fx = jacFx;
            obj.jac_fu = jacFu;
            obj.jac_hx = jacHx;
        end

        function setSigmaP(obj,sigma)
            obj.sigmaP = sigma;
        end

        function setCP(obj,c)
            obj.cP = c;
        end

        function updateACP(obj)
            % % Adaptive arrival-cost****************************************           
            % NkNum1      = 1 + (obj.q0bar'*obj.mtxP)*(obj.q0bar);
            % NkNum2      = obj.sigmaP / norm(obj.v_kminusNe)^2;
            % Nk          = full(NkNum1 * NkNum2);
            % if Nk > 1
            %     alpha       = 1 - (1/Nk);
            %     W           = obj.mtxP - (obj.mtxP*(obj.q0bar*obj.q0bar')*obj.mtxP) ./ (1+(obj.q0bar'*obj.mtxP)*obj.q0bar);
            %     traceW      = trace(full(W));
            %     if(traceW/alpha <= obj.cP)
            %         obj.mtxP = W./casadi.DM(alpha);
            %     else
            %         obj.mtxP = W;
            %     end      
            %     obj.optiMHE.set_value(obj.P,inv(obj.mtxP));% tic; eye(obj.nq)/S.mheLoss.nlmheLoss.mtxP; toc; tic; inv(S.mheLoss.nlmheLoss.mtxP); toc
            % end
            % Kalman update************************************************
            obj.Alin = obj.jac_fx(obj.qk,obj.lastInput);
            obj.Blin = obj.jac_fu(obj.qk,obj.lastInput);
            obj.Clin = obj.jac_hx(obj.qk);
            % discretise the matrices
            m_aux    = expm([full(obj.Alin),full(obj.Blin);zeros(obj.nu,obj.nq+obj.nu)].*obj.Ts);                                
            obj.Alin = m_aux(1:obj.nq,1:obj.nq);
            obj.Blin = m_aux(1:obj.nq,obj.nq+1:end);
            %
            obj.mtxP = obj.Alin*obj.mtxP*obj.Alin' + obj.mtxW;
            obj.S    = obj.Clin*obj.mtxP*obj.Clin' + obj.mtxR;
            obj.K    = obj.mtxP*obj.Clin'/(obj.S);% + 1e-6.*eye(obj.ny));
            obj.mtxP = (eye(obj.nq) - obj.K*obj.Clin)*obj.mtxP;        
        end 

        function updateMeasurement(obj,y)
            obj.Ytraj = [obj.Ytraj(:,2:end),y];
        end

        function updateInput(obj,u)
            obj.Utraj       = [obj.Utraj(:,2:end),u];
            obj.lastInput   = u;
        end

        function setMtxR(obj,mtxR)
            obj.mtxR = casadi.DM(mtxR);
        end

        function setMtxW(obj,mtxW)
            obj.mtxW = casadi.DM(mtxW);
            obj.optiMHE.set_value(obj.mtxWPrm,mtxW);
        end

        function set_x0bar(obj,q0)
            obj.q0bar = q0;
            obj.optiMHE.set_value(obj.qbar,q0);
        end

        function set_alpha(obj,alpha)
            obj.alphaPrmVal = alpha;
            obj.optiMHE.set_value(obj.alphaPrm,alpha);
        end

        function set_cPrm(obj,c)
            obj.cPrmVal = c;
            obj.optiMHE.set_value(obj.cPrm,c);
        end
        
        function solve(obj)
            % *************************************************************
            % Find the coefficients of the loss functions
            % *************************************************************
            obj.optiMHE.set_value(obj.Y,obj.Ytraj);
            obj.optiMHE.set_value(obj.U,obj.Utraj);
            obj.optiMHE.set_value(obj.qbar,obj.q0bar);            
            %
            try
                obj.solutionMHE = obj.optiMHE.solve();
                % obj.optiMHE.set_initial(obj.solutionMHE.value_variables());            
                obj.Vtraj       = obj.solutionMHE.value(obj.v);
                obj.Wtraj       = obj.solutionMHE.value(obj.w);
                obj.JmheQVal    = obj.solutionMHE.value(obj.JmheQ);
                %
                obj.Qtraj       = obj.solutionMHE.value(obj.Q);
                obj.qk          = obj.Qtraj(:,end);
                obj.q0bar       = obj.Qtraj(:,2);
                obj.qkAux       = full(obj.Fsym(obj.qk,obj.lastInput));
                obj.v_kminusNe  = obj.Vtraj(:,1);
                updateACP(obj);
                obj.optiMHE.set_value(obj.P,inv(obj.mtxP));
            catch
                obj.qk      = obj.qkAux;
                obj.qkAux   = full(obj.Fsym(obj.qk,obj.lastInput));
            end
        end                        

    end
    
end

