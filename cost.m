function j = cost(x,y,theta,h) % Cost Function
j = 0;
m = length(x);
for i = 1:m
    j = j + y(i,:).*(-log(h(x(i,:),theta))) + (1-y(i,:)).*(-log(1-h(x(i,:),theta)));
end
end
