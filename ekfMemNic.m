classdef ekfMemNic < handle
    properties (GetAccess = public, SetAccess = private)
        nq, ny, Ts;
        Qtraj, Ptraj, Ytraj;
        Qk, Rk;
        x0bar;
    end
    
    methods
        function obj = ekfMemNic(Ts, Qk, Rk, nq, ny)
            obj.Ts  = Ts;
            obj.nq  = nq;
            obj.ny  = ny;
            obj.Qk  = Qk;
            obj.Rk  = Rk;
            
            obj.Qtraj = zeros(nq, 1); % Estados inicializados con CasADi
            obj.Ptraj = eye(nq);      % Covarianza inicial (identidad)
            obj.Ytraj = zeros(ny, 1);
        end
        
        function predict(obj, F, F_jacobian, u)
            % Predicción del estado
            x_pred = F(obj.Qtraj(:, end), u);

            % Evaluar el Jacobiano de la dinámica antes de usarlo
            F_jac_eval = full(F_jacobian(obj.Qtraj(:, end), u));
            
            % Predicción de la covarianza
            P_pred = F_jac_eval * full(obj.Ptraj) * F_jac_eval' + full(obj.Qk);

            % Actualizar trayectorias
            obj.Qtraj = [obj.Qtraj, x_pred];
            obj.Ptraj = P_pred;
        end
        
        function update(obj, measurement, H_jacobian)
            % Calcular la innovación
            y_tilde = measurement - full(obj.Qtraj(:, end));

            % Evaluar el Jacobiano de la observación
            H_jac_eval = full(H_jacobian(obj.Qtraj(:, end)));
            
            % Calcular la covarianza de la innovación
            S = H_jac_eval * full(obj.Ptraj) * H_jac_eval' + full(obj.Rk);
            
            % Calcular la ganancia de Kalman
            K = full(obj.Ptraj) * H_jac_eval' / S;
            
            % Actualizar estado y covarianza
            x_kalman = full(obj.Qtraj(:, end)) + K * y_tilde;
            P_kalman = (eye(obj.nq) - K * H_jac_eval) * full(obj.Ptraj);
            
            obj.Qtraj(:, end) = full(x_kalman);
            obj.Ptraj = P_kalman;
        end

        function set_x0bar(obj, x0bar)
            obj.Qtraj(:, end) = x0bar;
        end

        function update_input(obj, u)
            obj.Ytraj = [obj.Ytraj, u];
        end

        function set_Qk(obj, Qk)
            obj.Qk = Qk;
        end

        function set_Rk(obj, Rk)
            obj.Rk = Rk;
        end
        
    end
end
