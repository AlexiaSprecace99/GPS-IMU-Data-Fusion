% get_mf_valid();
% hold on;
% get_mf_invalid();
function [mf_valid] = get_mf_valid()
% Definizione della soglia di confidenza e della pendenza della rampa
p = 7.8;
slope = 1/(3.9);

% Definizione del vettore di q da valutare
q_vec = 0:0.01:p*1.2;

% Definizione della membership function
mf_valid = zeros(size(q_vec));
mf_valid(q_vec <= 0.5*p) = 1;
mf_valid(q_vec > 0.5*p & q_vec <= p) = 1 - slope*(q_vec(q_vec > 0.5*p & q_vec <= p) - 0.5*p);
mf_valid(q_vec > p) = 0;
plot(q_vec,mf_valid);
end


% function [mf_invalid] = get_mf_invalid()
% % Definizione della soglia di confidenza e della pendenza della rampa
% s = 3.9;
% slope = 1/(3.9);
% 
% % Definizione del vettore di q da valutare
% q_vec = 0:0.01:7.8*1.2;
% 
% % Definizione della membership function
% mf_invalid = zeros(size(q_vec));
% mf_invalid(q_vec < s) = 0;
% mf_invalid(q_vec > s & q_vec <= 7.8) = 1+slope*(q_vec(q_vec > 3.9 & q_vec <= 7.8)-7.8);
% mf_invalid(q_vec >= 7.8) = 1;
% 
% plot(q_vec,mf_invalid);
% end
