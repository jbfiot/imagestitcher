function descriptors = GetDescriptors(pic, points)

%
% GetDescriptors
% 
% function descriptors = GetDescriptors(pic, points)
%
% Input
%   - pic : picture to get descriptors from
%   - points : points to compute descriptors
% 
% Output
%   - descriptors, each 2D slice / 3rd coord is a 9*9 descriptors, centered
%   on a (x,y) point. Each coefficient of this descriptor is the mean of a
%   5*5 submatrix. (Therefore a descriptor is based on a 45*45 window centered
%   on the (x,y) point.)

% @author: JB Fiot
% @Version: Nov08

    descriptors = zeros(9,9,size(points,1));
    descriptor = zeros(9,9);
    pic_padded = padarray(pic,[22,22]);
    for i=1:size(points,1)
        window = pic_padded(points(i,1):points(i,1)+44,points(i,2):points(i,2)+44);
        for j=1:9
            for k=1:9
                descriptor(j,k) = mean(mean(window(5*j-4:5*j,5*k-4:5*k)));
            end
        end
       % DescMean = repmat(mean(descriptor), [size(descriptor,2) 1]);
        %DesStd = repmat(std(descriptor), [size(descriptor,2) 1]);
        %descriptor = (descriptor - DescMean)/DesStd;
        descriptor = (descriptor - mean(mean(descriptor)))/std(std(descriptor));
        descriptors(:,:,i) = descriptor;
        
    end
