classdef mheMemNic < handle
    properties (GetAccess = public, SetAccess = private)
        % system's properties
        Ne, nq, nu, ny, Ts, Nt;
        % Numerical values of the optimal solution
        Qtraj, Utraj, Ytraj, Wtraj, Vtraj, q0bar, qkAux, qk, mtxW, mtxWPrm, mtxVPrm, mtxR, mtxP, v_kminusNe, sigmaP, cP; % mtxR was used for adapting the AC with Kalman
        x0Val, y0Val, x1Val, y1Val, x2Val, y2Val, x3Val, y3Val;
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
        Q, w, vAtt, vGps0, vGps1, vGps2, vGps3, X;
        vXYGps0, vXYGps1, vXYGps2, vXYGps3;
        x0, y0, x1, y1, x2, y2, x3, y3;
        % Parameters related to the GPS's positions
        wf, wr, l, h0, h1, h2, h3, th0, th1, th2, th3, normV1, normV2, thv;
        normV1Prm, normV2Prm, vNorm;
        th0Prm, th1Prm, th2Prm, th3Prm, h0Prm, h1Prm, h2Prm, h3Prm;
        % Quadratic (standard) MHE
        solutionMHEalpha, optiMHEalpha;
    end
    
    methods
        function obj = mheMemNic(Ne,Nt,F,h,dims,boxConst,Ts)
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
            obj.mtxP            = 1e6.*eye(obj.nq);
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
            obj.mtxWPrm         = obj.optiMHE.parameter(obj.nq,obj.nq);
            obj.mtxVPrm         = obj.optiMHE.parameter(1+2*4,1+2*4);
            obj.normV1Prm       = obj.optiMHE.parameter;
            obj.normV2Prm       = obj.optiMHE.parameter;
            obj.th0Prm          = obj.optiMHE.parameter;
            obj.th1Prm          = obj.optiMHE.parameter;
            obj.th2Prm          = obj.optiMHE.parameter;
            obj.th3Prm          = obj.optiMHE.parameter;
            obj.h0Prm           = obj.optiMHE.parameter;
            obj.h1Prm           = obj.optiMHE.parameter;
            obj.h2Prm           = obj.optiMHE.parameter;
            obj.h3Prm           = obj.optiMHE.parameter;
            % Init paarmetrs with trivial values:
            obj.optiMHE.set_value(obj.Y,obj.Ytraj);
            obj.optiMHE.set_value(obj.U,obj.Utraj);
            obj.optiMHE.set_value(obj.qbar,obj.q0bar);
            obj.optiMHE.set_value(obj.P,inv(obj.mtxP));
            % OPtimisation variables of MHE1_____________________________
            obj.Q               = obj.optiMHE.variable(obj.nq,obj.Ne+1);        % state trajectory
            obj.w               = obj.optiMHE.variable(obj.nq,obj.Ne);          % disturbances trajectory
            obj.vAtt            = obj.optiMHE.variable(1,obj.Ne+1);        % residuals of the estimation
            obj.vGps0           = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vGps1           = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vGps2           = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vGps3           = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vXYGps0         = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vXYGps1         = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vXYGps2         = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vXYGps3         = obj.optiMHE.variable(2,obj.Ne+1);
            obj.vNorm           = obj.optiMHE.variable(2,obj.Ne+1);
            obj.X               = obj.optiMHE.variable(obj.nq);                 % arrival-cost          
            obj.x0              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.y0              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.x1              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.y1              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.x2              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.y2              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.x3              = obj.optiMHE.variable(1,obj.Ne+1);
            obj.y3              = obj.optiMHE.variable(1,obj.Ne+1);
            %
            obj.JmheQ = (obj.X') * obj.P * obj.X;                                                                               % arrival-cost
            obj.optiMHE.subject_to(obj.X == obj.Q(:,1)-obj.qbar);                                                           % arrival-cost constraint
            for i=1:obj.Ne+1
                % System constraints --------------------------------------
                if i<obj.Ne+1
                    obj.JmheQ = obj.JmheQ + ( ((obj.w(:,i)') * (obj.mtxWPrm)) * (obj.w(:,i)) );
                    obj.optiMHE.subject_to( obj.Q(:,i+1) == obj.Fsym(obj.Q(:,i),obj.U(:,i)) + obj.w(:,i) );                    
                end                
                % Multiple measurement constraints ------------------------
                % Measurement constraints
                obj.optiMHE.subject_to( obj.Y(1,i)   == obj.Q(1,i) + obj.vAtt(i) );
                obj.optiMHE.subject_to( obj.Y(2:3,i) == [obj.x0(i); obj.y0(i)] + obj.vGps0(:,i) );
                obj.optiMHE.subject_to( obj.Y(4:5,i) == [obj.x1(i); obj.y1(i)] + obj.vGps1(:,i) );
                obj.optiMHE.subject_to( obj.Y(6:7,i) == [obj.x2(i); obj.y2(i)] + obj.vGps2(:,i) );
                obj.optiMHE.subject_to( obj.Y(8:9,i) == [obj.x3(i); obj.y3(i)] + obj.vGps3(:,i) );
                % GPS's position constraints
                obj.optiMHE.subject_to( (obj.x0(i)-obj.x2(i))^2 + (obj.y0(i)-obj.y2(i))^2 == obj.normV1Prm^2 + obj.vNorm(1,i) );
                obj.optiMHE.subject_to( (obj.x1(i)-obj.x3(i))^2 + (obj.y1(i)-obj.y3(i))^2 == obj.normV2Prm^2 + obj.vNorm(2,i) );                
                % GPS's measurements and center of mass constraints
                obj.optiMHE.subject_to( obj.x0(i) == obj.Q(2,i) + obj.h0Prm*cos(obj.Q(1,i) + obj.th0Prm) + obj.vXYGps0(1,i) );
                obj.optiMHE.subject_to( obj.y0(i) == obj.Q(3,i) + obj.h0Prm*sin(obj.Q(1,i) + obj.th0Prm) + obj.vXYGps0(2,i) );
                obj.optiMHE.subject_to( obj.x1(i) == obj.Q(2,i) + obj.h1Prm*cos(obj.Q(1,i) - obj.th1Prm) + obj.vXYGps1(1,i) );
                obj.optiMHE.subject_to( obj.y1(i) == obj.Q(3,i) + obj.h1Prm*sin(obj.Q(1,i) - obj.th1Prm) + obj.vXYGps1(2,i) );
                obj.optiMHE.subject_to( obj.x2(i) == obj.Q(2,i) + obj.h2Prm*cos(-pi/2 - obj.Q(1,i) + obj.th2Prm) + obj.vXYGps2(1,i) );
                obj.optiMHE.subject_to( obj.y2(i) == obj.Q(3,i) + obj.h2Prm*sin(-pi/2 - obj.Q(1,i) + obj.th2Prm) + obj.vXYGps2(2,i) );
                obj.optiMHE.subject_to( obj.x3(i) == obj.Q(2,i) + obj.h3Prm*cos(-pi/2 - obj.Q(1,i) - obj.th3Prm) + obj.vXYGps3(1,i) );
                obj.optiMHE.subject_to( obj.y3(i) == obj.Q(3,i) + obj.h3Prm*sin(-pi/2 - obj.Q(1,i) - obj.th3Prm) + obj.vXYGps3(2,i) );
                % Residual costs ------------------------------------------
                obj.JmheQ = obj.JmheQ + obj.vAtt(i) * obj.mtxVPrm(1) * obj.vAtt(i);
                obj.JmheQ = obj.JmheQ + obj.vGps0(:,i)' *obj.mtxVPrm(2:3,2:3) * obj.vGps0(:,i);
                obj.JmheQ = obj.JmheQ + obj.vGps1(:,i)' *obj.mtxVPrm(4:5,4:5) * obj.vGps1(:,i);
                obj.JmheQ = obj.JmheQ + obj.vGps2(:,i)' *obj.mtxVPrm(6:7,6:7) * obj.vGps2(:,i);
                obj.JmheQ = obj.JmheQ + obj.vGps3(:,i)' *obj.mtxVPrm(8:9,8:9) * obj.vGps3(:,i);
                
                obj.JmheQ = obj.JmheQ + 1e6 * (obj.vNorm(:,i)' * obj.vNorm(:,i));

                obj.JmheQ = obj.JmheQ + 1e6 * obj.vXYGps0(:,i)' * obj.vXYGps0(:,i);
                obj.JmheQ = obj.JmheQ + 1e6 * obj.vXYGps1(:,i)' * obj.vXYGps1(:,i);
                obj.JmheQ = obj.JmheQ + 1e6 * obj.vXYGps2(:,i)' * obj.vXYGps2(:,i);
                obj.JmheQ = obj.JmheQ + 1e6 * obj.vXYGps3(:,i)' * obj.vXYGps3(:,i);

                % N-trailer vehicle constraints ---------------------------
                % Joint constraints
                for j=1:obj.Nt
                    obj.optiMHE.subject_to( obj.Q(j,i) == obj.Q(obj.Nt+j,i)-obj.Q(obj.Nt+j+1,i) );
                end
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
            s_optsMHE1 = struct('sb','no','print_level',0,'gamma_theta',1e-12,'jacobian_approximation','exact','max_iter',10000,'tol',1e-8,'warm_start_init_point','no','timing_statistics','no','print_timing_statistics','no','print_user_options', 'no','print_options_documentation', 'no');%, ,'gamma_theta',1e-2, 'fast_step_computation','yes','jacobian_approximation','exact','warm_start_init_point','yes'); % 
            obj.optiMHE.solver('ipopt',p_optsMHE1,s_optsMHE1);
            % *************************************************************
            % list of parameters to be tuned: https://coin-or.github.io/Ipopt/OPTIONS.html
        end       

        function setPrmsGPSpos(obj,prmVals)
            obj.wf  = prmVals(1);
            obj.wr  = prmVals(2);
            obj.l   = prmVals(3);
            %
            obj.h0      = sqrt((obj.wf/2)^2+(obj.l/2)^2);
            obj.h1      = obj.h0;
            obj.h2      = sqrt((obj.wr/2)^2+(obj.l/2)^2);
            obj.h3      = obj.h2;
            obj.th0     = atan(obj.wf/obj.l);
            obj.th1     = obj.th0;
            obj.th2     = atan(obj.wr/obj.l);
            obj.th3     = obj.th2;
            obj.normV1  = sqrt(((obj.wf+obj.wr)/2)^2+(obj.l/2)^2);
            obj.normV2  = obj.normV1;
            obj.thv     = 2*atan(obj.wf/obj.l);
            % Set parameter values
            obj.optiMHE.set_value(obj.h0Prm,obj.h0);
            obj.optiMHE.set_value(obj.h1Prm,obj.h1);
            obj.optiMHE.set_value(obj.h2Prm,obj.h2);
            obj.optiMHE.set_value(obj.h3Prm,obj.h3);
            obj.optiMHE.set_value(obj.th0Prm,obj.th0);
            obj.optiMHE.set_value(obj.th1Prm,obj.th1);
            obj.optiMHE.set_value(obj.th2Prm,obj.th2);
            obj.optiMHE.set_value(obj.th3Prm,obj.th3);
            obj.optiMHE.set_value(obj.normV1Prm,obj.normV1);
            obj.optiMHE.set_value(obj.normV2Prm,obj.normV2);
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
            obj.optiMHE.set_value(obj.mtxVPrm,mtxR);
        end

        function setMtxW(obj,mtxW)
            obj.mtxW = casadi.DM(mtxW);
            obj.optiMHE.set_value(obj.mtxWPrm,mtxW);
        end

        function set_x0bar(obj,q0)
            obj.q0bar = q0;
            obj.optiMHE.set_value(obj.qbar,q0);
            obj.qkAux = q0;
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
                % obj.Vtraj       = obj.solutionMHE.value(obj.v);
                % obj.Wtraj       = obj.solutionMHE.value(obj.w);
                obj.JmheQVal    = obj.solutionMHE.value(obj.JmheQ);
                %
                obj.Qtraj       = obj.solutionMHE.value(obj.Q);
                obj.x0Val       = obj.solutionMHE.value(obj.x0);
                obj.y0Val       = obj.solutionMHE.value(obj.y0);
                obj.x1Val       = obj.solutionMHE.value(obj.x1);
                obj.y1Val       = obj.solutionMHE.value(obj.y1);
                obj.x2Val       = obj.solutionMHE.value(obj.x2);
                obj.y2Val       = obj.solutionMHE.value(obj.y2);
                obj.x3Val       = obj.solutionMHE.value(obj.x3);
                obj.y3Val       = obj.solutionMHE.value(obj.y3);
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

