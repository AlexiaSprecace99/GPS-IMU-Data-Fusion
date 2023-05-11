%In this matlab script there is the rappresentation of Chi-Square
%distribution for 2,3,6,9 deegree-of-freedom.In particular, we need the value 
% of the 3-degree-of-freedom distribution for which we have a 95 confidence interval.
%It will be used as measures limit in Kalman filter correction step. 

x = 0:0.1:20;
df = [2 3 6 9];
y = zeros(length(x), length(df));

for i=1:length(df)
    y(:,i) = chi2pdf(x,df(i));
end

figure;
plot(x,y,'LineWidth',2);
title('Chi-square distribution with different degree-of-freedom');
xlabel('x');
ylabel('f(x)');
legend(cellstr(num2str(df', 'Degree-of-freedom=%-d')));
grid on;

dq = 3;
alpha = 0.05;

q = chi2inv(1 - alpha,dq);

