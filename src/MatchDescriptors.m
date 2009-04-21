function [im1_matches, im2_matches] = MatchDescriptors(im1_points, im1_descriptors, im2_points, im2_descriptors, epsilon)
%
% MatchDescriptors
%
% function [im1_matches, im2_matches] = MatchDescriptors(im1_points, im1_descriptors, im2_points, im2_descriptors, epsilon) 
% 
% Input
%   - im1_points, im2_points: points from 2 images
%   - im1_descriptors, im2_descriptors: corresponding descriptors
%   - epsilon: threshold for descriptors to match
% 
% Output
%   - im1_matches, im2_points : points matching in the two images

% @author: JB Fiot
% @Version: Nov08


    im1_matches = [];
    im2_matches = [];
    
    DescDist = zeros(size(im1_points,1), size(im2_points,1));
    
    for i=1:size(im1_points,1)
        for j=1:size(im2_points,1)
            DescDist(i,j) = trace(dist2(im1_descriptors(:,:,i),im2_descriptors(:,:,j)));
        end
        
        col_i = DescDist(i,:);
        [first_min,ind] = min(col_i);
        col_i(ind) = [];
        sec_min = min(col_i);
        
        if sec_min ~= 0        
            if first_min/sec_min < epsilon
                im1_matches = [im1_matches; [im1_points(i,1), im1_points(i,2)]];
                im2_matches = [im2_matches; [im2_points(ind,1), im2_points(ind,2)]];
            end
        end
    end