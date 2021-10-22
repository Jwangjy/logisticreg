function j = cost2(x,y,theta,h1) % Cost Function
j = 0;
m = length(x);
for i = 1:m
    j = j + y(i,:).*(-log(h1(x(i,:),theta))) + (1-y(i,:)).*(-log(1-h1(x(i,:),theta)));
end
end
