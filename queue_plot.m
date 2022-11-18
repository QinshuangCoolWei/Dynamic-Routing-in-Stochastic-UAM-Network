% compare difference of with/without queue

load('q_0.mat');


% find worst-case path for each origin 1-28)
path_cell = cell(length(V)-1,2);

for k_v = 1:length(V)-1
    path = zeros(round(size(P,1))/2,2);           % preset [state, action] pairs (adjust length if necessary)
    if k_v == 1
        state_0 = 1;
    else
        ind_temp = find(ind_arr(:,2)==k_v);
        state_0 = sum(ind_arr(ind_temp,end-1:end))-1;   % fully charge the battery one

    end
    
    path(1,1) = state_0;
    path(1,2) = policy(state_0);
    for i=2:size(path,1)
        state_current = path(i-1,1);
        action_current = path(i-1,2);
        state_next = find(P(state_current,:,action_current));
        if length(state_next)>1
            state_next = min(state_next);   % only possible in S_e or S_v, in both cases, the smallest one is worst        
        end
        path(i,1) = state_next;
        path(i,2) = policy(state_next);
        
        if (state_current == state_next) &&(state_next ~= size(P,1)-1)  
            % trapped in other states
            path_cell{k_v,2} = 0;     
            first_zero = find(path(:,2)==0,1);
            path_cell{k_v,1} = path(1:first_zero-1,:);
            break;
        elseif (state_current == state_next) &&(state_next == size(P,1)-1)
            % successfully arrive
            path_cell{k_v,2} = 1;
            first_zero = find(path(:,2)==0,1);
            path_cell{k_v,1} = path(1:first_zero-1,:);
            break;
        end
    end
end


% convert path to route
route_cell = cell(length(V)-1,1);   % []
for k_v = 1:length(V)-1
    if path_cell{k_v,2} ==0
        continue;
    end
    path = path_cell{k_v,1};
    route = zeros(29,3);    % preset route [state,next_edge,leaving battery, charge or not(at this state)]
    route(1,1) = k_v;
    next_acts = find(edge_arr(:,2)==k_v);   % available next edges
    route(1,2) = next_acts(path(1,2));  % next edge
    route(1,3) = b_cap; %leave with full battery
    route(1,4) = 0;     
    counter = 1;
    for i = 2:size(path,1)
        state_label = state_label_fcn(ind_arr,path(i,1)); %[node/edge num, indicator(0-6 for q,b,l,d,target,sink)]
        if (state_label(2) == 2) && (path(i,2)<13)
            % leaving from charging
            counter = counter+1;
            route(counter,1) = state_label(1);
            next_acts = find(edge_arr(:,2)==state_label(1));   % available next edges
            route(counter,2) = next_acts(path(i,2));  % next edge
            ind_0 = find(ind_arr(:,2)==state_label(1),1);
            ind = path(i,1)-ind_arr(ind_0,5)+1;       
            route(counter,3) = S_b{state_label(1),1}(ind,end);  % leaving battery
            route(counter,4) = 1;   
        elseif (state_label(2) == 4) && (path(i,2)~=13)
            % end traveling and not charging
            counter = counter+1;
            head = edge_arr(state_label(1),end);   % arrived at corresponding head node
            route(counter,1) = head;
            next_acts = find(edge_arr(:,2)==head);   % available next edges
            route(counter,2) = next_acts(path(i,2));  % next edge
            ind_0 = find(indArr(:,4)==state_label(1),1);
            ind = path(i,1)-ind_arr(state_label(1),5)+1;          
            route(counter,3) = S_d{state_label(1),1}(ind,end);
            route(counter,4) = 0;   % not landing
        elseif (state_label(2) == 5) 
            counter = counter+1;
            route(counter,1) = 29;
            route(counter,4) = 1;
            break;

        elseif (state_label(2) == 6)
            counter = counter+1;
            route(counter,1) = 30;
            break;
        end
        


    end
    first_zero = find(route(:,1)==0,1);
    route_cell{k_v} = route(1:first_zero-1,:);
    

end

q_max = 4;
% start comparison with queue
compare_cell = cell(length(V)-1,2);
for k_v = 1:length(V)-1
    route = route_cell{k_v};
    q_list = permn(0:q_max,size(route,1)-1);
    q_list_max = max(q_list,[],2);  % longest route along the route
    arrive_list = zeros(size(route,1),size(q_list,1));

    for i = 1:size(q_list,1)
        arrive_list(1,i) =1;        
        for k_stop = 2:size(route,1)
            max_wait = T_b*floor(q_list(i,k_stop-1)/C(k_v));
            if max_wait + x_e(2,route(k_stop-1,2)) <= b_cap
                 arrive_list(k_stop,i) =1;
             else
                 arrive_list(k_stop,i) =0;
             end

        end        
        
    end
    compare_cell{k_v,1} =arrive_list;
    comp = zeros(2,q_max+2);  %[all, 0,1,2,3,4]
    comp(1,1) = size(q_list,1);
    comp(2,1) = size(q_list,1);
    for k2 = 1:q_max+1
        arrive_max_ind = max(q_list,[],2)<=(k2-1);      % max queue along the route <= k2-1
        arrive_max = arrive_list(:,arrive_max_ind);
        arrive_judgement = prod(arrive_max,1);
        comp(1,k2+1) = size(arrive_max,2);  % number of cases
        comp(2,k2+1) = sum(arrive_judgement);   % successful ones
    end
   
%     arrive_judgement = prod(arrive_list,1);
%     comp = [size(q_list,1),sum(arrive_judgement)];
    compare_cell{k_v,2} = comp;
end


compare_total = zeros(2,q_max+2);
for k_v = 1:length(V)-1
    for k2 = 1:size(compare_total,2)
        compare_total(:,k2) = compare_total(:,k2)+compare_cell{k_v,2}(:,k2);
    end
    
end

% find # of links no still satisfies the assumption when max queue along the route = k2
link_compare_total = zeros(1,q_max+1);
link_compare_total(1) = length(E);
for k2 = 1:q_max
    for k_e = 1:length(E)
        head = edge_arr(k_e,3);        
        max_wait = T_b*floor(k2/C(head));
        if x_e(2,k_e) + max_wait <= b_cap
            link_compare_total(k2+1) = link_compare_total(k2+1)+1;
        end
    end
    
end






% battery c
counter = 0;
travel_time_arr = zeros(length(V)-1,2);
for k_v = 1:length(V)-1
    route = route_cell{k_v};
    if ismember(15,route(:,3))
        counter = counter+1;
        travel_time_arr(k_v,2)=1;
    end
    travel_time = 0;
    for kk = 1:size(route,1)-1
        link_t = x_e(2,route(kk,2));
        travel_time = travel_time+link_t;

        if kk == size(route,1)-1
            charge_t =0; % no charging after the last link
        elseif route(kk,3)-link_t>=15 
            %charge 1 lv
            charge_t = T_b/k_b_hat;
        else
            %charge 2 lv
            charge_t = T_b;
        end
        travel_time =travel_time+charge_t;
    end
    travel_time_arr(k_v,1) = travel_time;


end

save_battery = counter;
% choose 1st and 27st to compare
t_c1 =3;
t_c2 =0;


dT_charge = T_b/k_b_hat;
t_charge_list = 6:6:30;
t_comp1 = zeros(2,length(t_charge_list));
t_comp2 = zeros(2,length(t_charge_list));

for k = 1:length(t_charge_list) 
    t_charge = t_charge_list(k);
    t_comp1(1,k) = travel_time_arr(1,1)-t_c1*dT_charge+t_c1*t_charge/k_b_hat;
    t_comp1(2,k) = t_comp1(1,k)+t_charge/k_b_hat;
    t_comp2(1,k) = travel_time_arr(27,1)-t_c2*dT_charge+t_c2*t_charge/k_b_hat;
    t_comp2(2,k) = t_comp2(1,k)+t_charge/k_b_hat;

end



%%%%%%%%%%%%% make plots
%%%% q plots 
% figure
% plot(0:q_max,compare_total(2,2:end)./compare_total(1,2:end),'k-o')
% hold on
% plot(0:q_max,link_compare_total/length(E),'b:*')
% hold off
% legend('Route','Link')
% xlabel('Maximum queue length')
% ylabel('Arrive-without-exhaustion rate')




%%%% battery plots 
figure
plot(t_charge_list,t_comp1(1,:)./t_comp1(2,:),'k-o')
hold on
plot(t_charge_list,t_comp2(1,:)./t_comp2(2,:),'b:*')
hold off
legend('R1','R28')
xlabel('Maximum Charging Time')
ylabel('rate')













function state_label = state_label_fcn(ind_arr,state_num)
    state_label = zeros(1,2);
    if state_num == ind_arr(end-1,5)
        state_label(2) = 5;    % target state
        state_label(1) = max(ind_arr(:,1));    % target state
    elseif state_num == ind_arr(end,5)
        state_label = [0,6];    % sink state
    else
        ind_temp = find((ind_arr(1:end-1,5)<=state_num) & (state_num <(ind_arr(1:end-1,5)+ind_arr(1:end-1,6))));    % position in ind_arr
        state_label(2) = find(ind_arr(ind_temp,1:4));
        state_label(1) = ind_arr(ind_temp,state_label(2));
    end

end





% [num,txt,raw] = xlsread("../Austin_Dallas.xlsx",1,'A1:AE30');  % read the file for distance
% [num2,txt2,raw2] = xlsread("../Austin_Dallas.xlsx",4,'C2:I30');  % read the file for queue and capacity
% 
% 
% dist_data = num(:,3:end);
% C = num2(:,1);     % capacity (change later)


% makeup queue


% queue = q_data(:,2:end);
% queue_prob = num3;




% queue_mdp_fcn(dist_data, queue, queue_prob, C);