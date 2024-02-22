clc;clear all;close all;

img=imread('testimg.jpg');

% 彩色图转为灰度图
img=rgb2gray(img);

%中值滤波，需编程实现
OTSU_result = MyOTSU(img);

figure;
imshow(OTSU_result);
title('OTSU_result');