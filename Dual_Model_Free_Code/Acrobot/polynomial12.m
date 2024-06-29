function[ref1]=polynomial12(b,U1)

if b==1
p=(20*U1^6 -48*U1^5 + 30*U1^4 -1)/(24*U1^5 -60*U1^4 +40*U1^3 -2);
else 
    p=U1;
end
Ti=0;
Ts=0.001;
t = Ti:Ts:1-Ts;
f = @(t, a_6, a_5, a_4, a_3, a_2, a_1, a_0) a_6*t^6 + a_5*t^5 + a_4*t^4 + a_3*t^3 + a_2*t^2 + a_1*t + a_0;
a_6 = (2*p-1)*(6*p^4 -12*p^3 + 4*p^2 +2*p +1)/(p^3 *(p -1)^3);
a_5 = -3*(4*p^6 -18*p^4 +16*p^3 -1)/(p^3 *(p -1)^3);
a_4 = 3*(10*p^6 -18*p^5 +10*p^3 -1)/(p^3 *(p -1)^3);
a_3 = -(20*p^6 -48*p^5 +30*p^4 -1)/(p^3 *(p-1)^3);
a_2 = 0;
a_1 = 0;
a_0 = 1;
ref1 = zeros(size(t));

% Generate the reference trajectory
for n = 1:length(t)
ref1(n) = f(t(n), a_6, a_5, a_4, a_3, a_2, a_1, a_0);
end
end
