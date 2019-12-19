function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

L = size(G,2); % number of control inputs
converged = 0;

u_opt = ones(K-1,1);
u_opt_imp = ones(K-1,1);

P_ = zeros(K-1,K-1);
G_ = zeros(K-1,1);

G(G==Inf) = 1e10;
iter = 0;

P(TERMINAL_STATE_INDEX,:,:) = [];
P(:,TERMINAL_STATE_INDEX,:) = [];
G(TERMINAL_STATE_INDEX,:) = [];

while ~converged
    
    % Policy Evaluation
    for i = 1:K-1
        for j = 1:K-1
            G_(i,1) = G(i,u_opt(i));
            P_(i,j) = P(i,j,u_opt(i));
        end
    end
    
    J = (eye(K-1,K-1) - P_)\G_;
    
    % Policy Improvement
    for i = 1:K-1
        argmin = 1e10;
        for u = 1:L
            if argmin > G(i,u) + P(i,:,u)*J
                u_opt_imp(i) = u;    % find optimal inputs
                argmin = G(i,u) + P(i,:,u)*J;
                iter = iter + 1;
            end
        end
    end
    
    if all(u_opt_imp - u_opt < 1e-50)
        converged = 1;
        disp('Policy iteration converged.')
    end
    
    u_opt = u_opt_imp;
    J_opt = [J(1:TERMINAL_STATE_INDEX-1);0;J(TERMINAL_STATE_INDEX:end)];
    u_opt_ind = [u_opt(1:TERMINAL_STATE_INDEX-1);1;u_opt(TERMINAL_STATE_INDEX:end)];
    
end
end
