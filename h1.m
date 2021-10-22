function h = h1(x,theta)
z = theta(1)+theta(2)*x(:,1)+theta(3)*x(:,2);
h = 1/(1+exp(-z));
end