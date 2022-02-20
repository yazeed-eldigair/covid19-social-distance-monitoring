close all
%Defining camera intrinsics
focalLength = cameraParams.FocalLength;
principalPoint = cameraParams.PrincipalPoint;
imageSize =  [2150, 1210];
camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

%Measuring camera extrinsics
height = 1.7;
pitch = 45;
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

%Define the area of the view that needs to be transformed
distAhead = 7;
spaceToOneSide = 3.5;
bottomOffset = 1.2;
outView = [bottomOffset, distAhead, -spaceToOneSide, spaceToOneSide];

%Define the birdsEye object and use it to apply the transformation on 
% an orginial image
outImageSize = [NaN, 1210];
birdsEye = birdsEyeView(sensor, outView, outImageSize);

img = imread('test2.jpg');
figure 
imshow(img)

bird = transformImage(birdsEye, img);
figure
imshow(bird)



