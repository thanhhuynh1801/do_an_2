% path='image1.png';
% Point=img_processing(path);
function [point] = img_processing(path)
img = imread(path);

imageSize = size(img);
disp(['imgsize: ', num2str(imageSize(1)), 'x', num2str(imageSize(2)), ' pixels']);

grayImg = rgb2gray(img);

binaryImg = imbinarize(grayImg);
binaryImg = ~binaryImg;

boundary = bwboundaries(binaryImg);

boundaryPoints = boundary{1};

k = 30;
boundaryPoints = boundaryPoints(1:k:end, :);
boundaryPoints = sortrows(boundaryPoints, 2);
point =boundaryPoints;

disp(boundaryPoints);
end