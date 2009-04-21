% Master MVA
% Object Recognition and Vision
% Assignment 2 - Stitching photo mosaics
% Jean-Baptiste FIOT


clear all; close all; clc;

display('========================================================================');
display('                           -= Image Stitcher =-');
display('                            By JB Fiot - Nov08');
display('========================================================================');



%%%%%%%%%%%%%%
% Parameters %
%%%%%%%%%%%%%%

% left_pic = imread('../Pics/keble_a.jpg');
% center_pic = imread('../Pics/keble_b.jpg');
% right_pic = imread('../Pics/keble_c.jpg');

% harris_threshold = 1.25;
% epsilon_descriptors = 0.7;
% epsilon_RANSAC = 10;
% nb_iter = 3000; 


left_pic = imread('../Pics/Zandvoort_4a_small.jpg');
center_pic = imread('../Pics/Zandvoort_4b_small.jpg');
right_pic = imread('../Pics/Zandvoort_4c_small.jpg');
harris_threshold = 2; %2->0.5 pour 5 et 6
epsilon_RANSAC = 10;
nb_iter = 5000; 
epsilon_descriptors = 1;

% nb_iter
% 1 à 4 : 3000
% 5 : 30000
% 6 : 15000

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Detection of local features in each image %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display('Detection of local features in each image...');
tic;

figure();
[left_y, left_x,left_v] = harris(left_pic);
title('Harris points from the left picture');
[left_x, left_y,left_v] = LessHarrisPoints(left_x, left_y,left_v, harris_threshold);
left_points = [left_x, left_y];    

figure();
[center_y, center_x, center_v] = harris(center_pic);
title('Harris points from the center picture');
[center_x,center_y,center_v] = LessHarrisPoints(center_x,center_y,center_v,harris_threshold);
center_points = [center_x, center_y];

figure();
[right_y, right_x, right_v] = harris(right_pic);
title('Harris points from the right picture');
[right_x,right_y,right_v] = LessHarrisPoints(right_x,right_y,right_v,harris_threshold);
right_points = [right_x, right_y];


display(sprintf('-> Left : %d harris points.',size(left_points,1)));
display(sprintf('-> Center : %d harris points.',size(center_points,1)));
display(sprintf('-> Right : %d harris points.',size(right_points,1)));

display(sprintf('-> Done in %.1f seconds.',toc));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extraction of feature descriptor for each feature point %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display('Extraction of feature descriptor for each feature point...');
tic;

left_descriptors = GetDescriptors(left_pic,left_points);
center_descriptors = GetDescriptors(center_pic,center_points);
right_descriptors = GetDescriptors(right_pic,right_points);

display(sprintf('-> Done in %.1f seconds.',toc));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matching of feature descriptors %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display('Matching of feature descriptors...');
tic;

[left_matches, L_center_matches] = MatchDescriptors(left_points,left_descriptors,center_points,center_descriptors,epsilon_descriptors);
[right_matches, R_center_matches] = MatchDescriptors(right_points,right_descriptors,center_points,center_descriptors,epsilon_descriptors);

display(sprintf('-> Center and Left : %d matches.',size(left_matches,1)));
display(sprintf('-> Center and Right : %d matches.',size(right_matches,1)));

display(sprintf('-> Done in %.1f seconds.',toc));


figure;
clf;
imagesc(left_pic);
hold on;

line([left_matches(:,2)'; L_center_matches(:,2)'],[left_matches(:,1)'; L_center_matches(:,1)'],'color','y');
plot(L_center_matches(:,2),L_center_matches(:,1),'<r');
hold off;
title('Matches between left and center pics');

figure;
clf;
imagesc(right_pic);
hold on;

line([right_matches(:,2)'; R_center_matches(:,2)'],[right_matches(:,1)'; R_center_matches(:,1)'],'color','y');
plot(R_center_matches(:,2),R_center_matches(:,1),'>r');
hold off;
title('Matches between right and center pics');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robust estimation of homographies using RANSAC %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

display('Robust estimation of homographies using RANSAC...');
tic;
[H_C2L, L_center_inliers, left_inliers] = computeH(L_center_matches,left_matches,nb_iter,epsilon_RANSAC);
display(H_C2L);
display(sprintf('--> with %d inliers.',size(L_center_inliers,1)));
[H_C2R, R_center_inliers, right_inliers] = computeH(R_center_matches,right_matches,nb_iter,epsilon_RANSAC);
display(H_C2R);
display(sprintf('--> with %d inliers.',size(R_center_inliers,1)));
display(sprintf('-> Done in %.1f seconds.',toc));


figure;
clf;
imagesc(left_pic);
hold on;
line([left_inliers(:,2)'; L_center_inliers(:,2)'],[left_inliers(:,1)'; L_center_inliers(:,1)'],'color','y');
plot(L_center_inliers(:,2),L_center_inliers(:,1),'<r');
hold off;
title('Inliers between left and center pics');

figure;
clf;
imagesc(right_pic);
hold on;
line([right_inliers(:,2)'; R_center_inliers(:,2)'],[right_inliers(:,1)'; R_center_inliers(:,1)'],'color','y');
plot(R_center_inliers(:,2),R_center_inliers(:,1),'>r');
hold off;
title('Inliers between right and center pics');


%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Warping and compositing %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
display('Warping and compositing...');
tic;

bbox = [-400 1200 -200 700];

center_w = vgg_warp_H(double(center_pic), eye(3), 'linear', bbox);
left_w = vgg_warp_H(double(left_pic), inv(H_C2L/H_C2L(3,3)), 'linear',bbox);
right_w = vgg_warp_H(double(right_pic), inv(H_C2R/H_C2R(3,3)), 'linear',bbox);

 figure;
imagesc(double(max(max(left_w,center_w),right_w))/255);
title('Such an awesome panorama!!');

display(sprintf('-> Done in %.1f seconds.',toc));




