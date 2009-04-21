function [H,im1_inliers,im2_inliers] = ComputeH(im1_matches, im2_matches, nb_iter, epsilon)
%
% ComputeH
%   -> robust estimation of the homography using RANSAC
%
% function [H,im1_inliers,im2_inliers] = ComputeH(im1_matches, im2_matches, nb_iter, epsilon)
% 
% Input
%   - im1_matches, im2_points : points matching in the two images
%       im2 = Him1
%   - nb_iter : number of iterations for the RANSAC algorithm
% 
% Output
%   - H : computed homography

% @author: JB Fiot
% @Version: Nov08


    nb_points = size(im1_matches,1);
    Best_nb_inliers = 0;

    for iter=1:nb_iter
        selected_points = [];
        im1_inliers=[]; im2_inliers=[];
        while size(unique(selected_points),1)~=4
            selected_points = random('discrete uniform', nb_points, [4 1]);
        end
        
        A=[];
        
		for i=1:4
			x1 = im1_matches(selected_points(i),1);
			y1 = im1_matches(selected_points(i),2);
			x2prime = im2_matches(selected_points(i),1);
			y2prime = im2_matches(selected_points(i),2);

			ax = [-x1,-y1,-1, 0,0,0, x2prime*x1,x2prime*y1,x2prime];
			ay = [0,0,0 ,-x1,-y1,-1, y2prime*x1,y2prime*y1,y2prime];

            A = [A;ax;ay];
        end
            
 		[U,S,V] = svd(A);
 		h = V(:,end);
        
        H = reshape(h, [3,3])';
        H = H/H(3,3);
        
        nb_inliers = 0;
        for i=1:nb_points
            Hi = H*[im1_matches(i,1);im1_matches(i,2);1];
         %   invHi=inv(H)*[im1_matches(i,1);im1_matches(i,2);1];
          %  if (Hi(3) ~= 0 && invHi(3)~=0)
           %     if ((Hi(1)/Hi(3)-im2_matches(i,1))^2+(Hi(2)/Hi(3)-im2_matches(i,2))^2+(invHi(1)/invHi(3)-im1_matches(i,1))^2+(invHi(2)/invHi(3)-im1_matches(i,2))^2)<epsilon
            if Hi(3) ~= 0
                if ((Hi(1)/Hi(3)-im2_matches(i,1))^2+(Hi(2)/Hi(3)-im2_matches(i,2))^2)<epsilon
                    nb_inliers = nb_inliers + 1;
                    im1_inliers = [im1_inliers;im1_matches(i,:)];
                    im2_inliers = [im2_inliers;im2_matches(i,:)];
                end
            end
		
        end
        
        if nb_inliers>Best_nb_inliers
            Best_nb_inliers = nb_inliers;
            Best_im1_inliers=im1_inliers;
            Best_im2_inliers=im2_inliers;
        end
              
    end
    im1_inliers=Best_im1_inliers;
    im2_inliers=Best_im2_inliers;
    
    %Re-estimation of the homographies with all inliers.
    A=[];
    for i=1:size(im1_inliers,1)
		x1 = im1_inliers(i,1);
        y1 = im1_inliers(i,2);
        x2prime = im2_inliers(i,1);
        y2prime = im2_inliers(i,2);

		ax = [-x1,-y1,-1, 0,0,0, x2prime*x1,x2prime*y1,x2prime];
		ay = [0,0,0 ,-x1,-y1,-1, y2prime*x1,y2prime*y1,y2prime];
            
        A = [A;ax;ay];
    end


    [U,S,V] = svd(A);
    h = V(:,end);
    H = reshape(h, [3,3])';
    t=H(1,:);
    H(1,:)=H(2,:);
    H(2,:)=t;
    t=H(:,1);
    H(:,1)=H(:,2);
    H(:,2)=t;
    
    
