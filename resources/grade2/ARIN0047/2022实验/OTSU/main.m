clc;clear all;close all;

img=imread('testimg.jpg');

% ��ɫͼתΪ�Ҷ�ͼ
img=rgb2gray(img);

%��ֵ�˲�������ʵ��
OTSU_result = MyOTSU(img);

figure;
imshow(OTSU_result);
title('OTSU_result');