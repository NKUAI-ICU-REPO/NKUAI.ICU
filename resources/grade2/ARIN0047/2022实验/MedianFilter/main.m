clc;clear all;close all;

img=imread('testimg.jpg');

% 彩色图转为灰度图
img=rgb2gray(img);

%中值滤波，需编程实现
Medfilt2_result = MyMedfilt2(img);

figure;
imshow(Medfilt2_result);
title('Medfilt2_result');