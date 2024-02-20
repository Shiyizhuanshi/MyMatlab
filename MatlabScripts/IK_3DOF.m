function [jointA_angle, jointB_angle, jointC_angle] = IK_3DOF(T, T_angle_initial_guess, AB, BC, CT, jointA_limit, jointB_limit, jointC_limit)
    fprintf('Target: %d, %d\n', T(1), T(2));
    iteration = 1;

    while iteration <= 4000
        %get the position of C
        pos_C = [T(1) - CT * cos(T_angle_initial_guess), T(2) - CT * sin(T_angle_initial_guess)];
        AC = sqrt((pos_C(1))^2 + (pos_C(2))^2);
        fprintf('pos_C: %d, %d\n', pos_C(1), pos_C(2));
        fprintf('The AC: %d, AB + BC: %d\n', AC, AB + BC);

        % Check AC First    
        if AC > AB + BC
            T_angle_initial_guess = T_angle_initial_guess + deg2rad(0.1);
            iteration = iteration + 1;
            fprintf('AC is greater than AB + BC in iteration: %d\n', iteration);
            continue;
        else
            % check no joint angle is out of limit
            AC_angle = atan2(pos_C(2), pos_C(1));
            S = AB + BC + CT;
            jointA_angle = asin(S/(AB*AC)) + AC_angle;
            jointB_angle = asin(S/(AB*BC)) + jointA_angle;
            jointC_angle = T_angle_initial_guess;
            
            if jointA_angle < jointA_limit(1) || jointA_angle > jointA_limit(2) || ...
                jointB_angle < jointB_limit(1) || jointB_angle > jointB_limit(2) || ...
                jointC_angle < jointC_limit(1) || jointC_angle > jointC_limit(2)
                fprintf('Joint angle out of limit in iteration: %d\n', iteration);
                T_angle_initial_guess = T_angle_initial_guess + deg2rad(1);
                iteration = iteration + 1;
            else
                break;
            end
        end
    end

    if iteration > 4000
        error('The iteration is greater than 4000');
    end

    fprintf('The jointA_angle: %f, jointB_angle: %f, jointC_angle: %f\n', ...
        (jointA_angle), (jointB_angle), (jointC_angle));
end
