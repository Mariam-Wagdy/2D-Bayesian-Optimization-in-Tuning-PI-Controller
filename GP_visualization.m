% Choose a kernel (covariance function) 
kernel = 5;

switch kernel
    case 1; k =@(x,y) 1*x'*y; % Linear 
    case 2; k =@(x,y) 1*min(x,y); % Brownian
    case 3; k =@(x,y) exp(-1013*(x-y)'*(x-y)); % Squared exponential
    case 4; k =@(x,y) exp(-1*sqrt((x-y)'*(x-y)));
    case 5; k =@(x,y) exp(-1*sin(50^2*pi^2*(x-y)'*(x-y))); % Periodic
end  
        

U=[100:100 :1000];
V=[100:100 :1000];
x=[U(:),V(:)]';

n=size(x,2);

% Construct the covariance matrix 
C = zeros(n,n);
for i = 1:n
    for j = 1:n
        C(i,j)= k(x(i),x(j));
    end
end

% Sample from the Gaussian process at these 
u = randsample(10,10); % (n,1) arrray of Normally distributed random numbers
[A,S, B] = svd(C); % factor C = ASB'
z = A*sqrt(S)*u; % z = A S^.5 u 
Z=z*z';
% Plot 
figure(1); hold on; clf
plot(x,z,'.-')
%axis([0, 1, -2, 2])

figure(2); clf
surf(U,V,Z);