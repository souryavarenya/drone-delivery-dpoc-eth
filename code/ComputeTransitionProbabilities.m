function P = ComputeTransitionProbabilities( stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%  Predefine some values for convinience
M = size(map,1);
N = size(map,2);

%  Predefined Maps
DX = containers.Map({NORTH, SOUTH, EAST, WEST, HOVER}, {0, 0, 1, -1, 0});
DY = containers.Map({NORTH, SOUTH, EAST, WEST, HOVER}, {1, -1, 0, 0, 0});

%  Compute Base State Index
[~,baseIndex] = ismember([1+floor(find(map' == BASE)/size(map,2)), 1+floor(find(map == BASE)/size(map,1)), 0],stateSpace,'rows');

%  Locating and counting the shooters
[x_shooters, y_shooters] = ind2sub(size(map),find(map == SHOOTER));
num_shooters = length(x_shooters);

%  Initialize a zero K x K x L matrix
P_temp = zeros(K, K, 5);

for i = 1:K
    for u = 1:5
        % Current m,n
        m = stateSpace(i,1);
        n = stateSpace(i,2);
        
        % Immediate Next State on action u
        m_next = m + DX(u);
        n_next = n + DY(u);
        
        % Checks if action allowable - if not, skips this action
        if (m_next)*(n_next)*(m_next - M - 1)*(n_next - N - 1) == 0
            continue
        end
        
        if map(m_next, n_next) == TREE
            continue
        end
        
        % Shooters distance check and sets probabilities
        d_shooters = abs(m_next*ones(num_shooters,1) - x_shooters) + abs(n_next*ones(num_shooters,1) - y_shooters);
        d_shooters(d_shooters > R) = -1;
        d_shooters = d_shooters(d_shooters ~= -1);
        p_shooters = GAMMA*ones(length(d_shooters),1)./(ones(length(d_shooters),1) + d_shooters);
        p_not_shot = prod(ones(length(p_shooters),1) - p_shooters);
        
        % Finds stateSpace index of mn_next & Assign P value to the respective location in P_temp
        [~,j] = ismember([m_next, n_next, stateSpace(i,3)],stateSpace,'rows');
        P_temp(i,j,u) = (1 - P_WIND)*p_not_shot;
        
        % Initialize base state counter
        p_back2base = (1 - P_WIND)*(1 - p_not_shot);
        
        % Run similar checks for neighboring cells
        for wind = 1:4
            m_wind = m_next + DX(wind);
            n_wind = n_next + DY(wind);
            
            % Checks if hits tree/boundary
            if (m_wind)*(n_wind)*(m_wind - M - 1)*(n_wind - N - 1) == 0
                p_back2base = p_back2base + (P_WIND/4);
                continue
            end
            
            if map(m_wind, n_wind) == TREE
                p_back2base = p_back2base + (P_WIND/4);
                continue
            end
                
            % Checks for shooters and calcs probab
            d_shooters = abs(m_wind*ones(num_shooters,1) - x_shooters) + abs(n_wind*ones(num_shooters,1) - y_shooters);
            d_shooters(d_shooters > R) = -1;
            d_shooters = d_shooters(d_shooters ~= -1);
            p_shooters = GAMMA*ones(length(d_shooters),1)./(ones(length(d_shooters),1) + d_shooters);
            p_not_shot = prod(ones(length(p_shooters),1) - p_shooters);
            
            % Finds stateSpace index of mn_wind & Assign P value to the respective location in P_temp
            [~,j] = ismember([m_wind n_wind stateSpace(i,3)],stateSpace,'rows');
            P_temp(i,j,u) = (P_WIND/4)*p_not_shot;
            
            % Add remaining probability of getting shot to back2base
            p_back2base = p_back2base + (P_WIND/4)*(1 - p_not_shot);
            
        end
        
        P_temp(i,baseIndex,u) = p_back2base;

    end
end

%  Compute Pick-up State Indices
[~,bef_pickup_index] = ismember([1+floor(find(map' == PICK_UP)/size(map,2)), 1+floor(find(map == PICK_UP)/size(map,1)), 0],stateSpace,'rows');
[~,aft_pickup_index] = ismember([1+floor(find(map' == PICK_UP)/size(map,2)), 1+floor(find(map == PICK_UP)/size(map,1)), 1],stateSpace,'rows');

%  Connecting pickup state from phi 0 to phi 1
for u = 1:5
    P_temp(bef_pickup_index, aft_pickup_index, u) = 1;
end

%  Terminal State Condition
for j = 1:K
    for u = 1:5
        P_temp(TERMINAL_STATE_INDEX,j,u) = 0;
    end
end

%  Final Transfer!
P = P_temp;

end
