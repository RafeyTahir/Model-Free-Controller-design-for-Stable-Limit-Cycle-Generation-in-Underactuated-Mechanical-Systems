function[ref1]=polynomial1(p)
if p==0
    Ti=0;
    Ts=0.01;
    t = Ti:Ts:1-0.01;
    ref1 = zeros(size(t));
else
Ti=0;
Ts=0.01;
t = Ti:Ts:1-0.01;
f = @(t, a_6, a_5, a_4, a_3, a_2, a_1, a_0) a_6*t^6 + a_5*t^5 + a_4*t^4 + a_3*t^3 + a_2*t^2 + a_1*t + a_0;
a_6 = (2*p-1)*(6*p^4 -12*p^3 + 4*p^2 +2*p +1)/(p^3 *(p -1)^3);
a_5 = -3*(4*p^6 -18*p^4 +16*p^3 -1)/(p^3 *(p -1)^3);
a_4 = 3*(10*p^6 -18*p^5 +10*p^3 -1)/(p^3 *(p -1)^3);
a_3 = -(20*p^6 -48*p^5 +30*p^4 -1)/(p^3 *(p-1)^3);
a_2 = 0;
a_1 = 0;
a_0 = 1;
% 
% if p==p_min
%     p=(20*p^6 - 48*p^5 +30*p^4 -1)/(24*p^5 - 60*p^4 + 40*p^3 -2)
%     %ref1=0.5*ones(size(t));
%     f = @(t, a_6, a_5, a_4, a_3, a_2, a_1, a_0) a_6*t^6 + a_5*t^5 + a_4*t^4 + a_3*t^3 + a_2*t^2 + a_1*t + a_0;
% a_6 = (2*p-1)*(6*p^4 -12*p^3 + 4*p^2 +2*p +1)/(p^3 *(p -1)^3);
% a_5 = -3*(4*p^6 -18*p^4 +16*p^3 -1)/(p^3 *(p -1)^3);
% a_4 = 3*(10*p^6 -18*p^5 +10*p^3 -1)/(p^3 *(p -1)^3);
% a_3 = -(20*p^6 -48*p^5 +30*p^4 -1)/(p^3 *(p-1)^3);
% a_2 = 0;
% a_1 = 0;
% a_0 = 1;
% ref1 = zeros(size(t));
% 
% % Generate the reference trajectory
% for n = 1:length(t)
% ref1(n) = f(t(n), a_6, a_5, a_4, a_3, a_2, a_1, a_0);
% end
% else if p==p_max
%         p1=(20*p^6 -48*p^5 + 30*p^4 -1)/(24*p^5 -60*p^4 +40*p^3 -2)
%         %ref1=0.5*ones(size(t));
%         f = @(t, a_6, a_5, a_4, a_3, a_2, a_1, a_0) a_6*t^6 + a_5*t^5 + a_4*t^4 + a_3*t^3 + a_2*t^2 + a_1*t + a_0;
% a_6 = (2*p1-1)*(6*p1^4 -12*p1^3 + 4*p1^2 +2*p1 +1)/(p1^3 *(p1 -1)^3);
% a_5 = -3*(4*p1^6 -18*p1^4 +16*p1^3 -1)/(p1^3 *(p1 -1)^3);
% a_4 = 3*(10*p1^6 -18*p1^5 +10*p1^3 -1)/(p1^3 *(p1 -1)^3);
% a_3 = -(20*p1^6 -48*p1^5 +30*p1^4 -1)/(p1^3 *(p1-1)^3);
% a_2 = 0;
% a_1 = 0;
% a_0 = 1;
% ref1 = zeros(size(t));
% 
% % Generate the reference trajectory
% for n = 1:length(t)
% ref1(n) = f(t(n), a_6, a_5, a_4, a_3, a_2, a_1, a_0);
% end
% else

ref1 = zeros(size(t));

% Generate the reference trajectory
for n = 1:length(t)
ref1(n) = f(t(n), a_6, a_5, a_4, a_3, a_2, a_1, a_0);
end
end
end
%end