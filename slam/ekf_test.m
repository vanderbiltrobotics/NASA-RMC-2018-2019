close all;
f = csvread('ekf_test_data');
std(f(:,1))
std(f(:,4))
x = 1:1:1000;

mean(f(:,4))
mean(f(:,5))
mean(f(:,6))
% 
% for i=1:1000
%     disp(num2str(f(i,1)) + "   " + num2str(f(i,4)))
% end
% 
% f(:,1)
% figure(1); hold on; 
% plot(x,f(:,1));
% plot(x,f(:,4)); title('x');legend({'1','2'});
% 
% figure(2); hold on;
% plot(x,f(:,2),x,f(:,5)); title('y');
% 
% figure(3); hold on;
% plot(x,f(:,3),x,f(:,6));title('theta');