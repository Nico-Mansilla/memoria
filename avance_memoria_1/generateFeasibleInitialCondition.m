function q_init_feasible = generateFeasibleInitialCondition(optiMHE, y_meas_data, uPast, Ts, l, nq, Nh, nu)
    % Generates a feasible initial condition for the given CasADi optimization problem

    % Define the tolerance for feasibility check
    tolerance = 1e-6;

    % Create a temporary optimization problem to test feasibility
    tempOpti = casadi.Opti();

    % Define variables
    qMHE_temp = tempOpti.variable(nq, Nh+1);  % State estimates
    w_temp = tempOpti.variable(nq, Nh);       % Process noise
    v_temp = tempOpti.variable(nq, Nh);       % Measurement noise
    q_init_temp = tempOpti.parameter(nq);     % Initial state

    % Set initial state constraint
    tempOpti.subject_to(qMHE_temp(:, 1) == q_init_temp);

    % Bounds on process and measurement noise
    tempOpti.subject_to(-0.1 <= w_temp <= 0.1);
    tempOpti.subject_to(-0.1 <= v_temp <= 0.1);

    % Define the dynamics function
    function Fk = F(q, u, Ts, l, w)
        Fk = q + Ts * [cos(q(3)), 0; 
                       sin(q(3)), 0; 
                       0, 1] * [1/2, 1/2; 
                               1/l, -1/l] * u + w;
    end

    % Define the objective function
    J_temp = 0;
    for k = 1:Nh
        J_temp = J_temp + (y_meas_data(:, k) - qMHE_temp(:, k)).' * diag([1, 1, 1]) * (y_meas_data(:, k) - qMHE_temp(:, k));
        tempOpti.subject_to(qMHE_temp(:, k+1) == F(qMHE_temp(:, k), uPast(:, k), Ts, l, w_temp(:, k)));
        J_temp = J_temp + w_temp(:, k).' * diag([0.1, 0.1, 0.1]) * w_temp(:, k);
        tempOpti.subject_to(y_meas_data(:, k) == qMHE_temp(:, k) + v_temp(:, k));
    end
    J_temp = J_temp + (y_meas_data(:, Nh+1) - qMHE_temp(:, Nh+1)).' * diag([1, 1, 1]) * (y_meas_data(:, Nh+1) - qMHE_temp(:, Nh+1));

    % Set objective
    tempOpti.minimize(J_temp);

    % Solver options
    p_optsTemp = struct('expand', true);
    s_optsTemp = struct('sb', 'yes', 'print_level', 0, 'gamma_theta', 1e-2, 'jacobian_approximation', 'exact', 'fast_step_computation', 'yes', 'warm_start_init_point', 'yes');
    tempOpti.solver('ipopt', p_optsTemp, s_optsTemp);

    % Initialize a random initial condition
    q_init_feasible = randn(nq, 1);

    % Try solving the temporary problem until a feasible solution is found
    while true
        tempOpti.set_value(q_init_temp, q_init_feasible);

        try
            solutionTemp = tempOpti.solve();
            % Check if all constraints are satisfied within the tolerance
            feasibility = true;
            for k = 1:Nh
                if norm(solutionTemp.value(qMHE_temp(:, k+1)) - F(solutionTemp.value(qMHE_temp(:, k)), uPast(:, k), Ts, l, solutionTemp.value(w_temp(:, k)))) > tolerance
                    feasibility = false;
                    break;
                end
            end
            if feasibility
                disp('Found a feasible initial condition.');
                break;
            end
        catch
            disp('Initial condition not feasible. Generating a new one...');
        end
        % Generate a new random initial condition
        q_init_feasible = randn(nq, 1);
    end
end
