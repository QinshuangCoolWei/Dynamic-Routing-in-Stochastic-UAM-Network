function [Prob_preset] = queue_preset(queue,k_b_hat)
q_unique = unique(queue);   % only set for each appeared number
Prob_preset = cell(length(q_unique),2);

for k=1:length(q_unique)
    k_q = q_unique(k);  % num of agents in queue
    Prob_preset{k,1} = k_q;
    if k_q == 0
        Prob_preset{k,2} = [];
        continue;
    end
    % create an array for product of probabilities
    Prob_table = permn(1:k_b_hat,k_q);

%     Num = (k_b_hat)^k_q;
%     Prob_table = zeros(Num,k_q);  %[T_q1, T_q2, ... , T_q(v), wait_time] (T_qk is the charging time for the k'th agent in the queue)
%     % produce the permutation
%     for l1 = 1:Num
%         for l2 = 1:k_q
%             rep = k_b_hat^(k_q-1);
%             %times = Num/k_b_hat;
%             if l2 == k_q
%                 Prob_table(l1,l2) = mod(l1,k_b_hat);
%                 if mod(l1,k_b_hat) == 0
%                     Prob_table(l1,l2) = k_b_hat;
%                 end
%             else
%                 Prob_table(l1,l2) = mod(floor((l1-1)/rep)+1,k_b_hat);
%                 if Prob_table(l1,l2) == 0
%                     Prob_table(l1,l2) = k_b_hat;
%                 end
%             end
%         end
%     end
    Prob_preset{k,2} = Prob_table;

end
end