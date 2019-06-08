function [ tfpu ] = MAGSolver(tpx0)

% MAGSolver - Implement optimal control problem with ICLOCS in
% closed-loop with Simulink
%
% Syntax:  [ tfpu ] = MAXSolver(tpx0)
%
% Input:
%    tpx0 - vector in format [current time; current state]
%
% Output:
%    tfpu - vector in format [terminal time; computation time; input to implement]
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% Dual-Agent CPT Problem
% Copyright 2019
%
% Adapted from the Supersonic Aircraft Minimum Fuel Climb example for
% ICLOCS Version 2 (2018).
% The contribution of Yuanbo Nie, Omar Faqir, and Eric Kerrigan for their
% work on ICLOCS Version 2 (2018) is kindly acknowledged.
% Department of Electrical and Electronic Engineering,
% Imperial College London, UK
%--------------------------------------------------------
global infoNLP data solution N opt_t_min opt_step 
persistent solution_new loopcount fintime u0 roz
if(isempty(loopcount))
    loopcount = 0;
    [problem,guess]=MAGProblem;    
    fintime = problem.time.tf_min;
    u0 = guess.inputs(1,:);
    roz=false;
end
disp('Stage 0 done')
simtime=tpx0(1); %Current simulation time (global)
x0=tpx0(2:end); %Current state

% Fetch the problem definition
[problem,guess]=MAGProblem;          
% Update solution when time stamp reached
if simtime~=0 && simtime>solution_new.t_ref
    solution=solution_new;
end
re_opt=1;
disp('Stage 1 done')
% Conditions for forced re-optimization
if any(any(x0'>(problem.states.xu-problem.states.xConstraintTol)))
    constVio=x0'-problem.states.xu;
    constVio(constVio<(-problem.states.xConstraintTol))=0;
    if any(constVio)
        re_opt=1;
    end
end
if any(any(x0'<(problem.states.xl+problem.states.xConstraintTol)))
    constVio=problem.states.xl-x0';
    constVio(constVio<(-problem.states.xConstraintTol))=0;
    if any(constVio)
        re_opt=1;
    end
end
 
% Conditions for forced non-re-optimization
if simtime>solution.t_ref && simtime<(solution.t_ref+opt_step)
    re_opt=0;
end
if fintime<opt_t_min
    re_opt=0;
end
disp('Stage 2 done')
if (simtime~=0 && ~re_opt || simtime==0&&roz) %|| mod(simtime,tstep)~=0 % For time instances without re-optimization
    crt_time=simtime-solution.t_ref; % Current time in the frame of previous solution
    fintime = solution.tf-crt_time;
    u0 = [speval(solution.Up,1,crt_time),speval(solution.Up,2,crt_time),speval(solution.Up,3,crt_time),speval(solution.Up,4,crt_time),speval(solution.Up,5,crt_time),speval(solution.Up,6,crt_time),speval(solution.Up,7,crt_time),speval(solution.Up,8,crt_time),speval(solution.Up,9,crt_time),speval(solution.Up,10,crt_time),speval(solution.Up,11,crt_time),speval(solution.Up,12,crt_time)];
    tfpu=[fintime;0;u0'];% Return the interpolated solution
    disp('Stage 7nro done')
else
    options= settings_hscubconst(N);                  % Get options and solver settings 
    if simtime~=0 % For later instances
%         opt_step=solution.computation_time;
        % If appromated value exceed the state bounds, set the value to
        % state bound limits if within user-defined tolerance, otherwise
        % terminate with an error
        if any(any(x0'>problem.states.xu))
            constVio=x0'-problem.states.xu;
            constVio(constVio<0)=0;
            if all(constVio<problem.states.xConstraintTol)
                x0(constVio>0)=problem.states.xu(constVio>0);
            else
                disp(x0'>problem.states.xu)
                error('Simulation going out of bound 1');
            end
        end
        if any(any(x0'<problem.states.xl))
            constVio=problem.states.xl-x0';
            constVio(constVio<0)=0;
            if all(constVio<problem.states.xConstraintTol)
                x0(constVio>0)=problem.states.xl(constVio>0);
            else
                disp(x0'<problem.states.xl)
                error('Simulation going out of bound 2');
            end
        end
        disp('Stage 3b done')       
        % Update the initial condition %%%
        problem.time.tf_min = fintime;
        problem.time.tf_max = fintime;
        problem.states.x0=x0';
        problem.states.x0l=x0';
        problem.states.x0u=x0';
        
        % Update the initial guess %%%
        guess.tf = fintime;
        guess.states(1,:)=x0';
        guess.inputs(1,:) = u0';
        
%         if fintime > opt_step
%             idx_new=solution.T>(opt_step);
%             T_new=[0;solution.T(idx_new)-(opt_step)];
%             guess.tf=fintime-(opt_step);
%             guess.time=T_new;
%             guess.states=[x0';solution.X(idx_new,:)];
%             guess.inputs=[speval(solution.Up,1,opt_step),speval(solution.Up,2,opt_step),speval(solution.Up,3,opt_step),speval(solution.Up,4,opt_step);solution.U(idx_new,:)];
%         else
%             guess.states(1,:)=x0';
%         end
        disp('Stage 4 done')
        % Update the new mesh
%         idx_new_2=solution.T>opt_step;
%         T_new_2=[0;solution.T(idx_new_2)-opt_step];
%         if solution.tf>opt_step && length(T_new_2)>=3
%             disp('here')
%             options.tau=diff(T_new_2./(guess.tf-opt_step));
%             options.nodes=length(options.tau)+1;
%         else
%             disp('there')
%             T_new_2=linspace(0,guess.tf,3);
%             T_new_2(end)=guess.tf;
%             options.tau=diff(T_new_2./guess.tf)';
%             options.nodes=length(options.tau)+1;  
%         end
        T_new_2=linspace(0,guess.tf,25);
        T_new_2(end)=guess.tf;
        options.tau=diff(T_new_2./(guess.tf))';
        options.nodes=length(options.tau)+1;
        disp('Stage 5 done')
    end

    % Solve the optimization problem
    maxAbsError=1e9;maxAbsConstraintError=1e9;
    i=1; imax=1;
    computation_time=0;
    while (any(maxAbsError>problem.states.xErrorTol) || any(maxAbsConstraintError>problem.constraintErrorTol)) && i<=imax && solution.status.status~=2
        [infoNLP,data,options]=transcribeOCP(problem,guess,options); % Format for NLP solver
        disp('transcription done')
        [solution_new,solution_new.status,data] = solveNLP(infoNLP,data);      % Solve the NLP
        [solution_new]=output(problem,solution_new,options,data,0);          % Output solutions
        computation_time=computation_time+solution_new.computation_time;
        maxAbsError=max(abs(solution_new.Error));
        maxAbsConstraintError=max(solution_new.ConstraintError);
        
%         [ options, guess ] = doMeshRefinement( options, problem, guess, data, solution_new, i );
        i=i+1;
    end
    if solution.status.status~=0&&solution.status.status~=1&&solution.status.status~=-1
        error('Ipopt unable to find optimal solution');
    elseif solution.status.status==-1
        warning('Max number of iterations exceeded')
    end
    
    disp('Stage 6 done')
    % Return the solution
    if simtime~=0
        solution_new.computation_time=computation_time;
        solution_new.t_ref=simtime;
        crt_time=simtime-solution.t_ref; % Current time in the frame of previous solution
        fintime = solution.tf-crt_time;
        u0 = [speval(solution.Up,1,crt_time),speval(solution.Up,2,crt_time),speval(solution.Up,3,crt_time),speval(solution.Up,4,crt_time),speval(solution.Up,5,crt_time),speval(solution.Up,6,crt_time),speval(solution.Up,7,crt_time),speval(solution.Up,8,crt_time),speval(solution.Up,9,crt_time),speval(solution.Up,10,crt_time),speval(solution.Up,11,crt_time),speval(solution.Up,12,crt_time)];
        tfpu=[fintime;computation_time;u0'] ; % Return the interpolated solution
        disp('Stage 7ro done')
    else
        solution_new.computation_time=0;
        solution_new.t_ref=0;
        fintime = solution_new.tf;
        u0 = solution_new.U(1,:);
        tfpu=[fintime;solution_new.computation_time;u0'];
        solution=solution_new;
        disp('Stage 7roz done')
        roz=true;
    end
end
% disp(solution.tf)
fprintf('\n\nFINISHED LOOP %d\n\n',loopcount)
loopcount = loopcount+1;

end