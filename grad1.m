function A = grad1(x,y,theta)
m = length(x);
A = zeros(2,1);
for i=1:m
    A(1) = A(1) + (h(x(i),theta)-y(i));
    A(2) = A(2) + (h(x(i),theta)-y(i))*x(i);
end
A = A/m;
end
