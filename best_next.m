clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Filming and plotting flags
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotting = false;
filming  = true;

if filming
    plotting = true;
end

% This flag determines if the planner stops when first target is destroyed
static_on_contact = false;


% Generate heatmap of attrition rate
% Eventually will want this to be learned
mu = [0 0];
Sigma = [.25 .3; .3 1];
x1 = 0:.5:50; x2 = 0:.5:150;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));

for i = 1:30
    mu =[floor(rand()*50) floor(rand()*150)];
    anti_diag = rand();
    Sigma = [anti_diag+rand() anti_diag; anti_diag anti_diag+rand()];
    tmp = mvnpdf([X1(:) X2(:)],mu,Sigma);
    tmp = reshape(tmp,length(x2),length(x1));
    F = F+tmp;
end

F = .001 + F./(10*max(max(F)));
F = F';

% Declare the number of agents and targets
robots = 60;
targets = 30;

% Initialize the agents
targeted = -1*ones(1,robots);

for i = 1:robots
    robot_array(i) = agent(i);
    robot_array(i).Target = ceil(rand()*targets);
    for j=1:robots
        robot_array(i).message_content(j) = 0;
        robot_array(i).message_confidence(j) = Inf;
    end
    %robot_array(i).message_content(robot_array(i).Target) = 1;
    robot_array(i).message_content(i) = robot_array(i).Target;
    %robot_array(i).message_confidence(robot_array(i).Target) = 0;
    robot_array(i).message_confidence(i) = 0;
    targeted(i) = robot_array(i).Target;
    robot_array(i).get_model(robot_array(i).message_content,targets);
end

%{
for i = 1:robots
    robot_array(i).get_model_cheat(targeted,targets);
end
%}

% Place the targets
% Consider destroying targets with a probability based on PK
% when the robots arrive
target_loc = zeros(2,targets);
target_status = ones(1,targets);
pk_at_destruction = zeros(1,targets);
for j = 1:targets
    target_loc(:,j) = [ceil(rand()*50);ceil(rand()*50)+100];
    target_color(j) = 'b';
end


% Randomly assign a desired pk of between 50 and 100%
for z=1:targets;
    target_pk(z) = 0.5 + rand()/2;
    value(z) = target_pk(z);
end

errorExists = true;
i =0;

% N is the max number of iterations
n=200;

% Score can be any of a numbber of metrics
% For using the "best-next" approach, lower cost is better
% If you want to maximize a score, change the retarget_bn function
cost = ones(1,n);

% Change this next line to change name of output movie
if filming
    aviobj = avifile('test.avi','compression','None');
end

tic
contact = false;


% Initialize inboxes
inbox_message = zeros(length(robot_array),length(robot_array));
inbox_confidence = zeros(length(robot_array),length(robot_array));
inbox_location = zeros(2,length(robot_array));
while(i<n)
    % Some plotting items
    if plotting
        fig = figure(1);
        clf
        axis([0 50 0 150 0 50])
        hold on
        axis equal
    end
    
    % Magic number is for keeping all agents from acting at the same time
    % This means that agents are more likely to act on up-to-date info
    magicNumber = floor(rand()*50);
    
    % The dynamics function at 10 Hz
    % Decision making is at 100 Hz
    % Combined with the "magic number" throttling, each agent effectively
    % makes decisions at 2 Hz
    if (~contact && errorExists)
        for j = 1:10
            %clear inbox_message inbox_confidence inbox_location
            %inbox_message = zeros(length(robot_array),length(robot_array));
            %inbox_confidence = zeros(length(robot_array),length(robot_array));
            %inbox_location = zeros(2,length(robot_array));
            for r = 1:length(robot_array)
                if (robot_array(r).Status~=0)
                    % One out of every fifty agents has the potential to switch
                    % on a given timestep
                    if (mod(r,50) == magicNumber)
                        robot_array(r).get_distances(target_loc);
                        if (rand()<1/(1+exp(-.4*(robot_array(r).Distance(robot_array(r).Target)-5))))
                            % Get updated information only when about to make a
                            % decision
                            for l = 1:size(inbox_location,2)
                                if (l~=r)
                                    % Checks if the squared distance to the sending
                                    % agent is within the communication radius squared
                                    if ((robot_array(r).State(1)-inbox_location(1,l))*(robot_array(r).State(1)-inbox_location(1,l))+ (robot_array(r).State(1)-inbox_location(1,l))*(robot_array(r).State(1)-inbox_location(1,l)) < 100)
                                        robot_array(r).receive_message(inbox_message(l,:),inbox_confidence(l,:));
                                    end
                                end                        
                            end
                            % Use updated information to update world model
                            robot_array(r).get_model(robot_array(r).message_content,targets);
                            % Select the target that will give the best next state
                            robot_array(r).retarget_bn(target_loc,target_pk);
                            % Update ground truth model
                            targeted(r) = robot_array(r).Target;
                            % Update robot's message
                            robot_array(r).message_content(r) = robot_array(r).Target;
                        end
                    end
                % One message per row
                inbox_message(r,:) = robot_array(r).message_content;
                % One confidence array per row;
                inbox_confidence(r,:) = robot_array(r).message_confidence;
                % One sender location per column
                inbox_location(:,r) = robot_array(r).State(1:2);

                % This is fine for now
                % TODO implement/integrate path planning
                robot_array(r).get_trajectory(target_loc);
                end % Paired with status check
            end
        end
    end 

    for r = length(robot_array):-1:1
        if (robot_array(r).Status~=0)
            %robot_array(r).get_model(robot_array(r).message_content,targets);
            % Check if each agent is destroyed or not on this timestep
            if (robot_array(r).check_attrition(x1,x2,F))
                for rc=1:length(robot_array)
                    robot_array(rc).message_content(r) = NaN;
                    robot_array(rc).message_confidence(r) =0;
                end
                robot_array(r).Status = 0;
                targeted(r) = NaN;

            else
                % velocity_hack sets the current velocity to match desired
                robot_array(r).velocity_hack();
                % RK4 integration of current state
                robot_array(r).RK4();
                if plotting
                    % Draw the agents
                    if (robot_array(r).Status ~= 0)
                        plot3(robot_array(r).State(1),robot_array(r).State(2),-robot_array(r).State(3),'rh');
                    end
                end
            end
        end
    end

    for tar = length(target_loc):-1:1
        contacting = [];
        for v = 1:length(robot_array)
            % Making a list of all agents within 1 meter of the target
            if (robot_array(v).Status ~= 0)
                if (norm(robot_array(v).State(1:3)-[target_loc(:,tar);0])<1)
                    contacting = [contacting;v];
                end
            end
        end
        % Assuming each weapon alone has an 85% chance of destroying the
        % target, calculate total Pk
        Pk = 1-.15^length(contacting);
        if (rand()<Pk)
            if (static_on_contact)
                contact = true;
            end
            target_pk(tar) = -0.000000000001;
            % Destoyed targets have a white fill color
            target_color(tar) = 'w';
%            disp('Target Destroyed');
            target_status(tar) = 0;
            pk_at_destruction(tar) = 1-0.3625^world_state(tar);
        else
            if plotting
                plot3(target_loc(1,tar),target_loc(2,tar),0,'h','MarkerFaceColor',target_color(tar));
            end
        end
        for c = length(contacting):-1:1
            % Clean up the messages sent as they pertain to targets about
            % to be destroyed
            for rc = 1:length(robot_array)
                robot_array(rc).message_content(contacting(c)) = NaN;
                robot_array(rc).message_confidence(contacting(c)) = 0;
            end
            % Destroy all robots that are within 1 meter o a target
            robot_array(contacting(c)).Status = 0;
            targeted(contacting(c)) = NaN;
%            disp('Agent Destroyed');
        end
    end
    if filming
        
        Frame = getframe(fig);
        aviobj = addframe(aviobj,Frame);
    end
    
    i = i +1;
   % Get the current world state from ground truth vector
    world_state = zeros(1,targets);
    for t = 1:targets
        for r = 1:length(targeted)
            if targeted(r) == t
                world_state(t) = world_state(t)+1;
            end
        end
    end
    
  %{
    disp('Agent');
    robot_array(1).message_content
    disp('World');
    targeted
%}
    % Score is based on placeholder effectiveness and attrition values
    % TODO make the cost more accurate
    cost(i) = agent.get_cost(world_state,target_pk);
    if cost(i) < 0.001
        %errorExists = false;
    end
    
    % If all agents are destroyed, display the iteration number
    if length(robot_array)<1
        i;
    end

end
toc

if (plotting)
    close(fig);
end
if filming
    aviobj = close(aviobj);
end
sum(target_status);


% Misc. ad hoc plots
%axis([0 30 -1 1])
%hold on
plot(cost)
% Plot the value of each target and whether or not it was destroyed
%{
figure(2)
plot(value)
hold on
plot(target_status,'r.');
%}
%{
% Plot the desired pk on each target & the pk at the time it was destroyed
figure(2)
plot(value)
hold on
plot(pk_at_destruction,'r.');
%}
