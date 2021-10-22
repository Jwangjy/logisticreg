function h = h(x,theta) % Hypothesis Function
z = theta(1)+theta(2)*x;
h = 1/(1+exp(-z));
end