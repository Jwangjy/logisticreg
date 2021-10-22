function [A1, A2, A3] = grad2(x,y,theta)
m = length(x(:,1));
A1 = 0;
A2 = 0;
A3 = 0;
for i=1:m
    A1 = A1 + h1(x(i,:),theta)-y(i,:);
end
A1 = A1/m;
for j=1:m
    A2 = A2 + (h1(x(j,:),theta)-y(j,:))*x(j,1);
end
A2 = A2/m;

for j=1:m
    A3 = A3 + (h1(x(j,:),theta)-y(j,:))*x(j,2);
end
A3 = A3/m;
end
