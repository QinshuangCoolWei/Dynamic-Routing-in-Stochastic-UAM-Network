


function queue_mdp_fcn(dist_data, queue, queue_prob, C)

% Case study for a delivery from Dallas to Austin
global queue;
global x_e;
global dt;
global dT_b;
global k_b_hat;
global C;
global Prob_preset;
global queue_prob;

max_speed = 150/60;
max_range = 60;

%[num3,txt3,raw3] = xlsread("Austin_Dallas.xlsx",4,'K2:O30');  % read the file for queue 
% [num2,txt2,raw2] = xlsread("Austin_Dallas.xlsx",4,'A2:D30');  % read the file for queue and capacity
% [num3,txt3,raw3] = xlsread("Austin_Dallas.xlsx",4,'R2:R30');  % read the file for queue 



%dist_data = num(:,3:end);
%q_data = num2(:,3:end);


N_node = max(num(:,1));
N_edge = sum(sum(dist_data>0 & dist_data<max_range,1),2);

% label the edges and Delta
mat2edge = zeros(N_node,N_node);
edge_arr = zeros(N_edge,3);
Delta = zeros(N_node,N_edge);

counter = 1;
for k1 = 1:N_node
    for k2 = 1:N_node
        if dist_data(k1,k2)>0 && dist_data(k1,k2)<max_range 
            mat2edge(k1,k2) = counter;
            edge_arr(counter,:) = [counter, k1,k2];
            Delta(k1,counter) = -1;
            Delta(k2,counter) = 1;
            counter = counter+1;
        end
    end
end

V = 1:N_node;
E = 1:N_edge;

% C = q_data(:,1);     % capacity (change later)
% queue = q_data(:,2:end);
% queue_prob = num3;


x_e = zeros(2,N_edge);
idx = sub2ind(size(dist_data),edge_arr(:,2),edge_arr(:,3));
dt = 3;
x_e(1,:) = floor(dist_data(idx)/max_speed/dt)*dt;   % minimum travel time floor to 2 * integer in minute
x_e(2,:) = round(x_e(1,:)*1.2/dt)*dt+dt;



b_cap = 30;         % capacity of battery
T_b = b_cap/5;      % total time to fully recharge battery
k_b_hat = 2;        % set there to be k_b_hat states of recharging (besides 0)
dT_b = T_b/k_b_hat; % time step of battery recharging
dt_b = b_cap/k_b_hat; % step size of battery recharging

[Prob_preset] = queue_preset(queue,k_b_hat);

A = zeros(1,length(E)+1);   % preset actions


% reward design
r_t = -dt;  % time-step penalty
r_b = dT_b/dt*r_t;   % recharging penalty
r_a = 1000;   % reward for arrival
r_s = -1000;      % sink penalty
r_d = -1000;     % battery dies

origin = 1; % the agent start from origin
dest = N_node;  % destination

source = [1]; % set of source nodes (not head node of any link)
sink = [N_node]; 

S_v = cell(length(V),3); 

for k_v = 1:length(source)
    v = source(k_v);
    S_v{v,1} = [v,0,0,b_cap];
    S_v{v,2} = 1;
    a = A;
    a(Delta(v,:)<0)=1;  % links that tau(e)=v (agent can fly on)
    S_v{v,3} = a;
end

for k_v = 1:length(V)
    v = V(k_v);
    if ismember(v,source)
        continue;
    end
    min_minus = min(x_e(1,Delta(v,:)>0)); % shortest time it may has experienced
    b_vhat = b_cap-min_minus;   % maximal possible battery when agent enter the queue
    N_state = b_vhat/dt+1;     % add a 0-battery state
    state_arr = zeros(N_state,4);   % initialize states for q_v
    A_arr = zeros(N_state,size(A,2)); 
    A_arr(2:end,end) = 1;       % next step is charging (unless battery =0)
    state_arr(:,1) = v;    
    for k_1 = 1:N_state
        state_arr(k_1,2) = 0;
        state_arr(k_1,3) = k_1-1;     %the first state is 0-battery state
        state_arr(k_1,4) = (k_1-1)*dt;
    end
    S_v{v,1} = state_arr;
    S_v{v,2} = N_state;
    S_v{v,3} = A_arr;
end



S_e = cell(length(E),3); 
for k_e = 1:length(E)
    e = E(k_e);
    tail_e = find(Delta(:,e)<0);
    head_e = find(Delta(:,e)>0);
    x_e_L = x_e(1,e);
    x_e_U = x_e(2,e);
    k_e_hat = (x_e_U-x_e_L)/dt+1;            
    b_ehat = b_cap-x_e_L;
    row_nozero = (b_cap-x_e_U)/dt;
    row_zero = k_e_hat;
    N_state = k_e_hat*row_nozero+ k_e_hat*(1+k_e_hat)/2; 
    state_arr = zeros(N_state,4);   % initialize states for e    
    A_arr = zeros(N_state,size(A,2));  % initialize actions
    counter = 0;
    for k_1 = 1:row_nozero % battery initial status different   
        b_start = b_ehat - (k_1-1)*dt;
        for kk_1 = 1:k_e_hat   % propagate through the states
            state_arr(kk_1+counter,1) = e;
            state_arr(kk_1+counter,2) = kk_1;
            state_arr(kk_1+counter,3) = 0;
            state_arr(kk_1+counter,4) = b_start-(kk_1-1)*dt;  
        end
        counter = counter+k_e_hat;
    end
    for k_2 = 1:row_zero
        b_start = b_ehat - row_nozero*dt - (k_2-1)*dt;
        for kk_2 = 1:k_e_hat- k_2 +1    % propagate through the queue
            state_arr(kk_2+counter,1) = e;
            state_arr(kk_2+counter,2) = kk_2;
            state_arr(kk_2+counter,3) = 0;
            if b_start-(kk_2-1)*dt >= 0
               state_arr(kk_2+counter,4) = b_start-(kk_2-1)*dt;     % if battery not 0         
            else
                break;
            end
        end
        counter = counter+k_e_hat - k_2+1;
    end
    S_e{e,1} = state_arr;
    S_e{e,2} = N_state;
    S_e{e,3} = A_arr;
end

% Decision states at each edge e
S_d = cell(length(E),3); 
for k_e = 1:length(E)
    e = E(k_e);
    tail_e = find(Delta(:,e)<0);
    head_e = find(Delta(:,e)>0);
    x_e_L = x_e(1,e);
    x_e_U = x_e(2,e);
    b_start = b_cap-x_e_L;      % maximal starting battery
    N_state = b_start/dt; 
    state_arr = zeros(N_state,4);   % initialize states  
    A_arr = zeros(N_state,size(A,2));  % initialize actions
    for k_d = 1:N_state
        state_arr(k_d,1) = e;
        state_arr(k_d,2) = 0;
        state_arr(k_d,3) = 0;
        state_arr(k_d,4) = b_start - (k_d-1)*dt;
        % action -> next edge of charge (b=1)
        edge_next = find(Delta(head_e,:)<0);
        for kk_d = 1:length(edge_next)
            A_arr(k_d,edge_next(kk_d))=1; 
        end
        A_arr(k_d,end)=1;

    end
    S_d{e,1} = state_arr;
    S_d{e,2} = N_state;
    S_d{e,3} = A_arr;
end


% Battery recharging states
S_b = cell(length(V),3); 
N_state = k_b_hat+1+b_cap/dt;
state_arr = zeros(N_state,4);   % initialize states for battery   

state_arr(1:b_cap/dt,end) = transpose(1:b_cap/dt)*dt;      % helper states


for kk_b = 1:k_b_hat
    state_arr(kk_b+1+b_cap/dt,2)=0;
    state_arr(kk_b+1+b_cap/dt,3)=0;
    state_arr(kk_b+1+b_cap/dt,4)=kk_b*dt_b;    % battery steps 
end

for k_v = 1:length(V)
    v = V(k_v);
    if ismember(v,source)
        continue;
    end

    edge_next = find(Delta(v,:)<0);
    action = A;
    action_helper = A;
    % action : choose to leave to next edge or to remain charging
    action(edge_next)=1;
    action(end) = 1;
    action_helper(end) = 1;
    % if v is a sink node (not a tail of any edge), no action?   
    if ismember(v,sink)
        A_arr = ones(N_state,1)*A;
        A_arr(b_cap/dt+1:end-1,end) =1;
    else
        A_arr = [ones(b_cap/dt,1)*action_helper;...
            zeros(1,size(A,2));...
            ones(k_b_hat,1)*action];
        A_arr(end,end) = 0;       % Full battery don't charge any more
        A_arr(b_cap/dt+1,end) = 1;     % 0 battery has to charge (this is the floored down battery remain, not strictly 0)
    end
    S_b{v,1} = state_arr+v*[1,0,0,0];
    S_b{v,2} = N_state;
    S_b{v,3} = A_arr;
end

% Begin formulation of S

% ind_arr -- index set
% ind_arr = S_v [v=1, b=0, e=0, d=0, N_start, N_state( of queue)]
%           S_b [v=0, b=1,e, d, N_start,  N_state]
%           S_e [...]   
%           S_d [...]
%           S_final[] 

N_temp = size(S_v,1) + size(S_b,1) + size(S_e,1) + size(S_d,1)+2;
ind_arr = zeros(N_temp,6); 
counter_S = 1;
counter_1 = 1;
counter_act = 0;

% set S_v
for k_1 = 1:size(S_v,1)    
    ind_arr(counter_1,1) =k_1;  % v
    ind_arr(counter_1,end-1) = counter_S;   % N_start
    ind_arr(counter_1,end) =S_v{k_1,2};  % N_state
    counter_S = counter_S + S_v{k_1,2};
    counter_1  =counter_1 + 1;
    counter_act = max(counter_act, max(sum(S_v{k_1,3},2)));
end

% set S_b
for k_2 = 1:size(S_b,1)
    if ismember(V(k_2),source)
        ind_arr(counter_1,2) = k_2;  % b
        counter_1  =counter_1 + 1;
        continue;       % the first node
    end
    ind_arr(counter_1,2) =k_2;  % v=1
    ind_arr(counter_1,5) = counter_S;   % N_start
    ind_arr(counter_1,6) =S_b{k_2,2};  % N_state
    counter_S = counter_S + S_b{k_2,2};
    counter_1  =counter_1 + 1;
    counter_act = max(counter_act, max(sum(S_b{k_2,3},2)));
end


% set S_e
for k_3 = 1:size(S_e,1)    
    ind_arr(counter_1,3) = k_3;  % e
    ind_arr(counter_1,5) = counter_S;   % N_start
    ind_arr(counter_1,6) =S_e{k_3,2};  % N_state
    counter_S = counter_S + S_e{k_3,2};
    counter_1  =counter_1 + 1;
    counter_act = max(counter_act, max(sum(S_e{k_3,3},2)));
end


% set S_d
for k_4 = 1:size(S_d,1)
    ind_arr(counter_1,4) = k_4;  % d
    ind_arr(counter_1,5) = counter_S;   % N_start
    ind_arr(counter_1,6) =S_d{k_4,2};  % N_state
    counter_S = counter_S + S_d{k_4,2};
    counter_1  =counter_1 + 1;
    counter_act = max(counter_act, max(sum(S_d{k_4,3},2)));
end

% set S_final & S_sink
ind_arr(counter_1,5) =counter_S;    % S_final
ind_arr(counter_1,6) =1;            

counter_S = counter_S+1;
counter_1 = counter_1+1;

ind_arr(counter_1,5) =counter_S;    % S_sink
ind_arr(counter_1,6) =1; 

state_total = counter_S;

% Setup P and R as SxSxA arrays
% action A: besides the actions, add a "do-nothing" action at the end
P = zeros(state_total,state_total,counter_act+1);
R = zeros(state_total,state_total,counter_act+1);


% set S_v -> S_e/S_v/ S_b
for k_v = 1:length(V)
    v = V(k_v);
    ind_S = (ind_arr(:,1)==v);
    N_start = ind_arr(ind_S,5);
    N_state = ind_arr(ind_S,6);
    state_arr = S_v{k_v,1};
    if ismember(v,source)
        counter_act = 0;
        for k_act = 1:size(A,2)-1     % edge with tail node v
            if S_v{v,3}(k_act)>0
                counter_act = counter_act+1;
                N_next = ind_arr(ind_arr(:,3)==k_act,5);    % start with full battery, next link
                P(N_start,N_next,counter_act) = 1;
                R(N_start,N_next,counter_act) = r_t*x_e(1,k_act)/dt;     % shortest travel time cost
            end
        end        
        continue;
    end

    for kk = 1:N_state
        ind = N_start+kk-1;
        if state_arr(kk,4) == 0 % battery dies
            P(ind,ind,:) = 1;     % loop
            R(ind,ind,:) = r_d;     % battery dying penalty
            continue;

        end
        % compute battery level
        battery = state_arr(kk,4);  % battery when enter the queue
        for k_b = 1:b_cap/dt     
            bat_d = battery-S_b{v,1}(k_b,end);  % battery difference with this state
            ind_b = k_b + ind_arr(ind_arr(:,2)==v,5)-1;

            P(ind,ind_b,end-1) = queue_next(bat_d,v);  % start charging
            R(ind,ind_b,end-1)= max(bat_d/dt,0)*r_t;   % penalty of the time difference
            
        end
        P(ind,N_start,end-1) = 1-sum(P(ind,:,end-1),2);    % battery dies
        R(ind,N_start,end-1)= r_d;



    end

end


% set S_b -> S_b/ S_e
for k_v = 1:length(V)
    v = V(k_v);
    if ismember(v,source)
        continue;
    end

    ind_S = (ind_arr(:,2)==v);
    N_start = ind_arr(ind_S,5);
    N_state = ind_arr(ind_S,6);
    state_arr = S_b{k_v,1};
    action_arr = S_b{k_v,3};



    % from battery helper states -> batter states
    for kk = 1:b_cap/dt
        ind =  N_start+kk-1;
        ind_temp = ceil(state_arr(kk,end)/dt_b);
        ind_next = ind_temp+1+b_cap/dt+N_start-1;
        P(ind,ind_next,end) = 1;
        R(ind,ind_next,end) = r_b*(state_arr(ind_temp+1+b_cap/dt,end)-state_arr(kk,end))/dt_b;

    end



    for kk = b_cap/dt+1:N_state
        ind = N_start+kk-1;
        counter_act = 0;
        for k_a = 1:size(A,2)-1     % edge_next = k_a (unless k_a indicates recharging)
            battery_next = max(0,state_arr(kk,end)-x_e(1,k_a));
            for k_e = 1:S_e{k_a,2}
                if k_e ==1 && S_e{k_a,1}(k_e,4)== battery_next
                    ind_temp = k_e;
                elseif (S_e{k_a,1}(k_e,4)==battery_next) && (S_e{k_a,1}(k_e,4)>=S_e{k_a,1}(k_e-1,4))     % the start of the link
                    ind_temp = k_e;
                end
            end
            ind_next = ind_arr((ind_arr(:,3)==k_a),5)+ind_temp-1;
            if action_arr(kk,k_a) >0  
                counter_act = counter_act+1;
                P(ind,ind_next,counter_act)=1;        % reach next state (go to next edge)
                R(ind,ind_next,counter_act)=r_t*x_e(1,k_a)/dt;     % shortest travel time cost

            end
        end
        if action_arr(kk,end) > 0   % can continue charging
            P(ind,ind+1,end-1)=1;        % reach next state (continue charging)
            R(ind,ind+1,end-1)=r_b;   % recharge time penalty
        end

    end

end





% set S_e -> S_e/S_d
for k_e = 1:length(E)
    e = E(k_e);
    ind_S = (ind_arr(:,3)==e);
    N_start = ind_arr(ind_S,5);
    N_state = ind_arr(ind_S,6);
    state_arr = S_e{k_e,1};
    
    for kk = 1:N_state-1 
        ind = N_start+kk-1;
        if (state_arr(kk,4) >0) && (state_arr(kk,4)> state_arr(kk+1,4))     % continue traveling or to decision state
            travel_start = find(S_e{e,1}(1:kk-1,4)<=S_e{e,1}(2:kk,4),1,'last');
            if isempty(travel_start)
                travel_start =0;                
            end
            k_pos = kk - travel_start;        % k_pos'th state in the traveling propagation
%             prob = 1 - dt/(x_e(2,e)-x_e(1,e)-(k_pos-1)*dt);
%             link_next(k_pos,e)
            P(ind,ind+1,end) = link_next(k_pos,e);  % continue traveling
            R(ind,ind+1,end) = r_t;     
    
            ind_d = find(state_arr(kk,4) == S_d{e,1}(:,end),1);
            ind_d = ind_d+ ind_arr(ind_arr(:,4)==e,5)-1;
            P(ind,ind_d,end) = 1-link_next(k_pos,e);  % end traveling (to decision state)
            R(ind,ind_d,end) = 0;     
        elseif (state_arr(kk,4) >0) && (state_arr(kk,4) <= state_arr(kk+1,4))    % reach maximal traveling time, to decision state
            ind_d = find(state_arr(kk,4) == S_d{e,1}(:,end));
            ind_d = ind_d+ ind_arr(ind_arr(:,4)==e,5)-1;
            P(ind,ind_d,end) = 1;  % end traveling (to decision state)
            R(ind,ind_d,end) = 0;                 
        elseif state_arr(kk,4) == 0
            ind_next = size(P,1);    
            P(ind,ind,:) = 1;
            R(ind,ind,:) = r_d;     % battery dying penalty
        end
        

    end
    ind = N_start+N_state-1;  % the last state must have battery = 0
    P(ind,ind,:) = 1;  % loop
    R(ind,ind,:) = r_d;                 


end




% set S_d -> S_e/S_v
for k_e = 1:length(E)
    e = E(k_e);
    e_head = find(Delta(:,e)>0);        % head node of link e
    ind_S = (ind_arr(:,4)==e);
    N_start = ind_arr(ind_S,5);
    N_state = ind_arr(ind_S,6);
    state_arr = S_d{k_e,1};
    action_arr = S_d{k_e,3};
    for kk = 1:N_state 
        ind = N_start+kk-1;    
        counter_act = 0;
        for k_a = 1:size(A,2)-1     % edge_next = k_a (unless k_a indicates recharging)
            battery_next = max(state_arr(kk,end)-x_e(1,k_a),0);
            for k_2 = 1:S_e{k_a,2}
                if k_2 ==1 && S_e{k_a,1}(k_2,4)== battery_next
                    ind_temp = k_2;
                elseif (S_e{k_a,1}(k_2,4)==battery_next) && (S_e{k_a,1}(k_2,4)>=S_e{k_a,1}(k_2-1,4))     % the start of the link
                    ind_temp = k_2;
                end
            end
            ind_next = ind_arr((ind_arr(:,3)==k_a),5)+ind_temp-1;       % compute index in S
             
            if action_arr(kk,k_a) > 0  
                counter_act = counter_act+1;
                P(ind,ind_next,counter_act)=1;        % reach next state (go to next edge)
                R(ind,ind_next,counter_act)=x_e(1,k_a)/dt*r_t;   %  lowest cost of traveling 
            end
        end
        % or enter queue of head node
        if action_arr(kk,end) > 0
            ind_start = ind_arr(ind_arr(:,1)==e_head,5);   % start index of S_v
            ind_temp = find(S_v{e_head,1}(:,4)==state_arr(kk,end));     % next state must have the same battery level
            ind_next = ind_start+ind_temp-1;
            P(ind,ind_next,end-1)=1;        % reach next state (enter the queue)
            R(ind,ind_next,end-1)=0;   
        end

    end

end

% set S_b{destination} -> S_final
for k_b = S_b{dest,2}-k_b_hat:S_b{dest,2}
    ind = ind_arr(ind_arr(:,2)==dest,5)+k_b-1;
    P(ind,end-1,end) = 1;
    R(ind,end-1,end) = r_a;
end
P(end-1,end-1,:) = 1;
R(end-1,end-1,:) = 0;

% make sure no actions -> sink
for i = 1:size(P,1)
    for act = 1:size(P,3)
        if sum(P(i,:,act))==0 && (i < size(P,1)-1)     % all other go to sink
            P(i,end,act) = 1;
            R(i,end,act) = r_s;
        elseif sum(P(i,:,act))==0 && (i == size(P,1))       % sink loop back
            P(i,i,act) = 1;
            R(i,i,act) = r_s;
        end

    end
end



mdp_verbose
% [policy, iter, cpu_time] = mdp_value_iteration(P, R, discount, epsilon, max_iter, V0)
[value,policy, iter, cpu_time] = mdp_value_iteration(P, R, 0.99,10,1000);

end


% save('AD_d6.mat')
% save('PR_new_d6.mat', 'P','R', '-v7.3')

function [prob] = queue_next(diff,v)
    global queue;
    global k_b_hat;     %    global dT_b;
    global C;    
    global dT_b;
    global Prob_preset;
    global queue_prob;
    k_q_list = queue(v);
    prob =0;
    for j = 1:length(k_q_list)
        k_q = k_q_list(j);

        if diff <0
            prob1 = 0;

        elseif k_q>0

            % create an array for product of probabilities
            Prob_table = Prob_preset{cell2mat(Prob_preset(:,1))==k_q,2};  %[T_q1, T_q2, ... , T_q(v), wait_time] (T_qk is the charging time for the k'th agent in the queue)
            Num = (k_b_hat)^k_q;
            Prob_table = [Prob_table,zeros(Num,1)]; % add a column for wait time
            % produce the permutation
            for l1 = 1:Num
                % find shortest wait time
                e_arr = zeros(C(v),1);
                for l3 = 1:k_q
                    [ee,ind] = min(e_arr);   % find the earliest time
                    e_arr(ind) = e_arr(ind)+Prob_table(l1,l3);  % pop the next in queue into the node
                end
                Prob_table(l1,end) = min(e_arr)*dT_b;       % waiting time
            end
            dP = 1/Num;
            num_diff = sum(Prob_table(:,end)==diff);  % number of terms with same difference
            prob1 = num_diff*dP;     %
        else
            if diff == 0
                prob1 = 1;
            else
                prob1 = 0;
            end
        end
        prob = prob + prob1*queue_prob(v,j);
    end
end

function [prob] = link_next(k,e)
    global x_e;
    global dt;
    if k==1
        prob = 1;   % for continuous distribution, the first prob must be 1
    elseif x_e(2,e)-x_e(1,e)-(k-2)*dt == 0
        prob = 0;
    else
        prob = 1 - dt/(x_e(2,e)-x_e(1,e)-(k-2)*dt);
    end
    
end

















