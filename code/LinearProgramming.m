function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

P(TERMINAL_STATE_INDEX,:,:) = 0;
P(:,TERMINAL_STATE_INDEX,:) = 0;
G(TERMINAL_STATE_INDEX,:) = 0;

L = size(G,2); % number of control inputs
P_ = zeros(K*L,K);
G_ = zeros(K*L,1);
    
for i = 1:L
    P_((K*(i-1)+1):K*i,:) = P(:,:,i);
    P_((K*(i-1)+1):K*i,:) = eye(K,K) - P_((K*(i-1)+1):K*i,:);
    G_((K*(i-1)+1):K*i,1) = G(:,i);
end

G_(G_==Inf) = 1e10;

% solve
f = -ones(K,1);
J = linprog(f,P_,G_);

% find optimal inputs
u_opt = zeros(K,1);
P(TERMINAL_STATE_INDEX,:,:) = 1;
P(:,TERMINAL_STATE_INDEX,:) = 1;
G(TERMINAL_STATE_INDEX,:) = 0;
for i = 1:K
    argmin = 100;
    for u = 1:L
        if argmin > G(i,u) + P(i,:,u)*J 
            u_opt(i) = u;
            argmin = G(i,u) + P(i,:,u)*J;
        end
    end
end
u_opt(TERMINAL_STATE_INDEX) = 1;

J_opt = J;
u_opt_ind = u_opt;

end
