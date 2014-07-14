% Point mass dynamics
classdef agent < handle
    methods(Static)
        function cost = get_cost(state,target_pk)
            % Still experimenting with different scoring functions
            % Current iteration is based on distance from desired Pk and a
            % weighting factor based on the prority of each target (as
            % expressed by desired Pk)
            cost = 0;
            %
            for i = 1:length(state);
                if (1-0.3625^state(i) < target_pk(i))
                    cost = cost + (target_pk(i)-1+0.3625^state(i))/(1-target_pk(i));
                end
            end
            %}
            %{
            for i = 1:length(state)
                % Assumes 25% attrition rate for all agents
                if ((1-0.3625^state(i))<target_pk(i))
                    cost = cost + target_pk(i) + 0.3625^state(i) - 1;
                end
            end
            %}
        end
    end
    methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = agent(ID)
            % This function creates an object representing one agent
            % Indexing from 1
            obj.ID = ID;
            obj.Target = -1;
            % North, East, Down convention
            obj.State = [ceil(rand()*49);ceil(rand()*49);-50;0;0;0];
            obj.Mass = 5; % 5 kg, placeholder until more detailed dynamics
            obj.Force = [0;0;0];% Current force exerted by actuators
            obj.Status = 1; % Target has not been destroyed
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
            % Eventually this will be replaced with a trajectory following
            % control system when we move from point-mass dynamics to
            % vehicle dynamics
            obj.State(4:6) = obj.Trajectory;
        end
        function dstate = stateDiff(obj, y)
            % State differential equations
            dstate(1:3) = y(4:6);
            dstate(4:6) = (1/obj.Mass)*obj.Force;
            dstate = dstate';
        end
        function [] = RK4(obj)
            % Runge-Kutta 4 Integrator at 10 Hz
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
            % Simulated Annealling approach
            % Randomly selects new target with probability based on angle
            % between heading and path to each target as well as number of
            % agents targeting each target
            robot.Model(robot.Target) = robot.Model(robot.Target)-1;
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
                    robot.Target = candidate_targets(i);
                    robot.Model(robot.Target) = robot.Model(robot.Target)+1;
                    return
                end
            end
            
        end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
        function retarget_bn(obj,target_loc,target_pk)
            % Best-next state, essentially a greedy local search
            % This approach is based on the observation that each agent
            % can only effect a very small percentage of the state space
            
            obj.Model(obj.Target) = obj.Model(obj.Target) - 1;
            for i = 1:length(target_loc)
                candidate_targets(i) = i;
            end
            
            min_cost =Inf;
            new_target = obj.Target;
            for i = 1:length(candidate_targets)
                
                result = obj.Model;
                result(obj.Target) = result(obj.Target)-1;
                result(i) = result(i)+1;
                cost = agent.get_cost(result,target_pk);

                if (cost < min_cost)
                    min_cost = cost;
                    new_target = i;
                end
            end
            obj.Target = new_target;
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Communication/Sensing Methods
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function get_model(obj,targeted,numTargets)
            % This function just takes the model from ground truth
            obj.Model = zeros(1,numTargets);
            for t = 1:numTargets
                for r = 1:length(targeted)
                    if targeted(r) == t
                        obj.Model(t) = obj.Model(t)+1;
                    end
                end
            end
        end
        function send_message(obj)
        end

        function [] = receive_message(obj,message,confidence)
            for i = 1:length(confidence)
                if (confidence(i) <= obj.message_confidence(i))
                    obj.message_confidence(i) = confidence(i)+1;
                    obj.message_content(i) = message(i);
                end
            end
        end
        
        function get_distances(obj, target_loc)
            % Get distance to each target
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
                %disp('Agent Destroyed');
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
        message_content
        message_confidence
        Status
    end
end
            