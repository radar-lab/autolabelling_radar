close all
clear all
clc
%%
c_idx = 1;
p_idx = 1;
b_idx = 1;
m_idx = 1;

%%
X_rad=[];
Y_rad=[];
Z_rad=[];
Dop_rad =[];
SNR_rad =[];
Az_rad = [];
El_rad = [];
TS_rad = [];
Img_Cropped = [];
Label = [];
Idx_d = 1;
%% 
format long
bagfile = '../../sample.bag';

[c_idx,p_idx,b_idx,m_idx,X_rad,Y_rad,Z_rad,Dop_rad,SNR_rad,Az_rad,El_rad,TS_rad,Img_Cropped,Label,Idx_d] = gen_rad_cam_data(bagfile,c_idx,p_idx,m_idx,b_idx,X_rad,Y_rad,Z_rad,Dop_rad,SNR_rad,Az_rad,El_rad,TS_rad,Img_Cropped,Label,Idx_d);


%%
writeNPY(X_rad,'../../X_rad.npy');
writeNPY(Y_rad,'../../Y_rad.npy');
writeNPY(Z_rad,'../../Z_rad.npy');
writeNPY(Dop_rad,'../../Dop_rad.npy');
writeNPY(SNR_rad,'../../SNR_rad.npy');
writeNPY(Az_rad,'../../Az_rad.npy');
writeNPY(El_rad,'../../El_rad.npy');
writeNPY(TS_rad,'../../TS_rad.npy');
writeNPY(Img_Cropped,'../../Img_Cropped.npy');
writeNPY(Label,'../../Class_rad_cam.npy');