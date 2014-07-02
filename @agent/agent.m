% Point mass dynamics
classdef agent < handle
    methods(Static)
        function score = get_score(state,target_pk)
            %{ 
            pk = zeros(1,length(state));
             for i = 1:length(state)
                pk(i) = 1- 0.3625^state(i);
                if (pk(i) >= target_pk(i))
                end
            end
            score = sum(abs(pk-target_pk));
            %}
            score = 0;
            for i = 1:length(state)
                if ((1-0.3625^state(i))<target_pk(i))
                    score = score + target_pk(i) + 0.3625^state(i) - 1;
                end
            end
        end
    end
    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = agent(ID)
            % Indexing from 1
            obj.ID = ID;
            obj.Target = -1;
            % North, East, Down convention
            obj.State = [ceil(rand()*49);ceil(rand()*49);-50;0;0;0];
            obj.Mass = 5; % 5 kg, placeholder mostly
            obj.Force = [0;0;0];
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Dynamics and Controls Methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [] = get_trajectory(obj,target_loc)
            % Set magnitude of lateral velocity
            velocity_mag = 10; %m/s placeholder
            max_accel = 1; % m/s^2
            old_vel = obj.State(4:5);
            % Distance to current target projected on XY Plane
            new_vel = (target_loc(:,obj.Target)-obj.State(1:2));
            range = norm([new_vel;0]);
            if (range>velocity_mag)
                new_vel = velocity_mag*new_vel/range;
                % 10 = 1/dt
                accels = 10*(new_vel-old_vel);
                if (max(accels)>max_accel)
                    accels = accels*max_accel/max(accels);
                end
                new_vel = old_vel + accels/10;
                descent_rate = - obj.State(3)*norm([new_vel;0])/range;
            else
                descent_rate = -obj.State(3);
            end
            obj.State(4:6) = [new_vel;descent_rate];
            obj.Trajectory = [new_vel;descent_rate];

        end
        function [] = velocity_hack(obj)
            obj.State(4:6) = obj.Trajectory;
        end
        function dstate = stateDiff(obj, y)
            dstate(1:3) = y(4:6);
            dstate(4:6) = (1/obj.Mass)*obj.Force;
            dstate = dstate';
        end
        function [] = RK4(obj)
            dt = 0.1;
            k1 = obj.stateDiff(obj.State);
            k2 = obj.stateDiff(obj.State+(dt/2)*k1);
            k3 = obj.stateDiff(obj.State+(dt/2)*k2);
            k4 = obj.stateDiff(obj.State+dt*k3);
            obj.State = obj.State + (dt/6)*(k1+2*k2+2*k3+k4);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Planning Methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function retarget_sa(robot,target_loc,target_pk)
            for i = 1:length(target_loc)
                candidate_targets(i) = i;
                candidate_angle(i) = atan2(target_loc(2,i)-robot.location(2),target_loc(1,i)-robot.location(1))-robot.heading;
            end
            candidate_targets(robot.target) = [];
            candidate_angle(robot.target) = [];

            inverse_angle = zeros(1,length(candidate_angle));
            sum = 0;
            for d = 1:length(candidate_angle)
                inverse_angle(d) = 1/candidate_angle(d) + sum;
                sum = inverse_angle(d);
            end
            %probability = inverse_angle./sum;
            %
            for i =1:length(inverse_angle)
                probability(i) = (1-isreal(log(1-target_pk-0.3625^robot.model(i))))*inverse_angle(i)/sum;
            end
            %}
            Q = rand();
            for i=1:length(probability)
                if (Q<probability(i))
                    robot.target = candidate_targets(i);
                    return
                end
            end
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
        function retarget_bn(obj,target_loc,target_pk)
            for i = 1:length(target_loc)
                candidate_targets(i) = i;
            end
            
            %candidate_targets(robot.target) = [];
            %score = zeros(1,length(candidate_targets));
            max_score =Inf;
            new_target = obj.Target;
            for i = 1:length(candidate_targets)
                
                result = obj.Model;
                result(obj.Target) = result(obj.Target)-1;
                result(i) = result(i)+1;
                score = agent.get_score(result,target_pk);

                if (score < max_score)
                    max_score = score;
                    new_target = i;
                end
            end
            obj.Target = new_target;
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Communication/Sensing Methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function get_model_cheat(obj,targeted,numTargets)
            obj.Model = zeros(1,numTargets);
            for t = 1:numTargets
                for r = 1:length(targeted)
                    if targeted(r) == t
                        obj.Model(t) = obj.Model(t)+1;
                    end
                end
            end
        end
        function compose_message(obj,numTargets)
            obj.message_model = obj.Model;
            
        end
        function receive_message()
        end
        function get_distances(obj, target_loc)
            for i =1:length(target_loc)
                obj.Distance(:,i) = target_loc(:,i)-obj.State(1:2); 
            end
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Attrition Related Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function destroyed = check_attrition(obj,x1,x2,F)
            % If the robot is outside of the heatmap by chance
            % Assign a nominal attrition rate
            if ((obj.State(1)<x1(1))||(obj.State(1)>x1(numel(x1)))||(obj.State(2)<x2(1))||(obj.State(2)>x2(numel(x2))))
                attrition_rate = 0.001;
            else
                % Interpolate to get attrition rate
                x_min = -1;
                y_min = -1;
                for i = 1:length(x1)
                    if (obj.State(1)>x1(i))
                        x_min = i;
                    end
                end

                for i = 1:length(x2)
                    if (obj.State(2)>x2(i))
                        y_min = i;
                    end
                end
                attrition_rate = (1/(x1(x_min+1)-x1(x_min))*(x2(y_min+1)-x2(y_min)))*(F(x_min,y_min)*(x1(x_min+1)-obj.State(1))*(x2(y_min+1)-obj.State(2)) + F(x_min+1,y_min)*(obj.State(1)-x1(x_min))...
            *(x2(y_min+1)-obj.State(2)) + F(x_min,y_min+1)*(x1(x_min+1)-obj.State(1))*(obj.State(2)-x2(y_min)) + F(x_min+1,y_min+1)*(obj.State(1)-x1(x_min))*(obj.State(2)-x2(y_min)));
            end
            if rand()<attrition_rate
                destroyed = true;
                disp('Agent Destroyed');
            else
                destroyed = false;
            end
        end
    end
    properties
        ID
        Target
        State
        Mass
        Force
        Trajectory
        Model
        Distance
        message_model
        message_confidence
    end
end
            