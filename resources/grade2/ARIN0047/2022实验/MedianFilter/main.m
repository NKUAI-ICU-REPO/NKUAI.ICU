clc;clear all;close all;

img=imread('testimg.jpg');

% ��ɫͼתΪ�Ҷ�ͼ
img=rgb2gray(img);

%��ֵ�˲�������ʵ��
Medfilt2_result = MyMedfilt2(img);

figure;
imshow(Medfilt2_result);
title('Medfilt2_result');