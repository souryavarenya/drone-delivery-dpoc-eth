function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    
    % IMPLEMENTATION OF G
    
    % Those are the 2 maps with the probs respectively for the shot and for
    % the wind
    map_of_shot = probShot(map);
    map_of_wind = probWind(map, map_of_shot);
    
   
    
    G = zeros(K,5); % matrix of stage costs
    
    [M,N] = size(map); % dimension of matrix of stage costs
    
    % calculation of the G for Hover input
    for j=1:K
        n = stateSpace(j,2);
        m = stateSpace(j,1);
        probCrash = map_of_shot(m,n) + map_of_wind(m,n) ;
        G(j,HOVER) =  (1-probCrash)*1 + probCrash*Nc;
    end
    
    %Once I have the calculations for HOVER, I need to consider the other
    %inputs: N,S,E,O.
    %What I do to get the G of the other inputs, given G of hover,
    %is to shift this matrix, that is equivalent of shifting the map
   
    
    % this matrix gives me the directions for the drone given the different
    % inputs
    translations = zeros(5,2);
    translations(HOVER, : ) = [0, 0];
    translations(NORTH, : ) = [0, +1];
    translations(SOUTH, : ) = [0, -1];
    translations(EAST, : ) = [+1, 0];
    translations(WEST, : ) = [-1, 0];
   
    
    % Firstly I send the information [m,n,phi] to the index which
    % corresponds in stateSpace
    % After a vecotr is created and it's used to map the information
    % [m,n,phi] in stateSpace to the index of the vector
    % The calculation done for the mapping is phi+[(m-1)*N+n]*2
    
    get_indexing = @(row, column, phi)((row-1)*N+column)*2+phi -1; % subtract 1 because of Matlab indexing
    
    from_state_to_idx = index_from_map(map);
    
    for direction = [NORTH,SOUTH,EAST,WEST]
       for j=1:K
               
            m_j = stateSpace(j, 1);
            n_j = stateSpace(j, 2);
            phi_j = stateSpace(j, 3);

            %I convert the possible input N,S,E,O in a vector
            dir = translations(direction,:);
            if m_j+dir(1)>=1 && m_j+dir(1)<=M && n_j+dir(2)>=1 && n_j+dir(2)<=N && map(m_j+dir(1),n_j+dir(2)) ~= TREE
                idx = get_indexing(m_j+dir(1), n_j+dir(2) ,phi_j);
                index_contained_in_StateSpace = from_state_to_idx( idx );
                G(j, direction) = G(index_contained_in_StateSpace, HOVER);
            else
                G(j, direction) = Inf; % infinite cost if I'm outside the range or in a tree
            end
       end
    end
    
    

end

% I compute the prob that the drone will drop due to the shooters
function map_of_shot = probShot(map)

global SHOOTER R GAMMA TREE

    [M,N] = size(map);
    map_of_shot = zeros(M,N);
        
    [row_s, col_s] = find(map==SHOOTER);
    
    for m=1:M
        for n=1:N
           if map(m,n) == TREE % not interested in the prob if I'm in a tree
               map_of_shot(m,n) = NaN; 
               continue
           end
           not_p = 1;
           for i = 1:size(row_s)
               d = norm( [m - row_s(i) , n - col_s(i)], 1);
               not_p = not_p * ( 1 - (d<=R)*GAMMA/( 1 + d ) );
               
           end
           map_of_shot(m,n) = 1 - not_p;
        end
    end
end

% I compute the prob that the drone will drop due to the wind
function map_of_wind = probWind(map, map_of_shot)

global TREE P_WIND
    [M,N] = size(map);
    map_of_wind = zeros(M,N);
    for m=1:M
        for n=1:N
            if map(m,n) == TREE
               map_of_wind(m,n) = NaN;
               continue
           end
            for i = [ [m;n-1], [m;n+1], [m-1;n], [m+1;n] ]
               if ~(i(1)>=1 && i(1)<=M && i(2)>=1 && i(2)<=N) || map(i(1),i(2)) == TREE
                   map_of_wind(m,n) = map_of_wind(m,n) + P_WIND/4;
               else
                   map_of_wind(m,n) = map_of_wind(m,n) + P_WIND/4*map_of_shot(i(1),i(2));
               end
            end
        end
    end
end

% This function maps the information [m, n, phi] tp the index in state space
% Practically a vector a 'from_state_to_idx' is created: its element at position [(m-1)*N+n]*2+phi
% corresponds to the information [m, n, phi] of the stateSpace
function from_state_to_idx = index_from_map(map)
global TREE
    count = 0;
    from_state_to_idx =[];
    % f([m, n, phi]) = ((m-1)*N+n)*2+phi, returns idx in stateSpace
    for m = 1 : size(map, 1)
        for n = 1 : size(map, 2)
            if map(m, n) ~= TREE
                from_state_to_idx =[ from_state_to_idx;
                        count+1;
                        count+2];
                count= count + 2;
            
            else
                from_state_to_idx =[ from_state_to_idx;
                        0;
                        0];
            end
        end
    end
end