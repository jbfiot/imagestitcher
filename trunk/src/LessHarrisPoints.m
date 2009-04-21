function [x,y,v] = LessHarrisPoints(x,y,v,threshold)

%
% LessHarrisPoints
% 
% function [x,y,v] = LessHarrisPoints(x,y,v,threshold)
%
% Input
%   - x, y, v: output of the harris.m function (points coords and values)
%   - threshold: threshold to keep or not harris points
% 
% Output
%   - x, y, v: harris points kept with their values
% 
% An harris point is kept if his bias/gain-normalized value is above the
% threshold.

% @author: JB Fiot
% @Version: Nov08

    m = mean(v);
    s = std(v);
    for i=size(x):-1:1
        if  (v(i)-m)/s<threshold
            y(i)=[];
            x(i)=[];
        end
    end