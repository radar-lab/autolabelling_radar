close all
clear all
clc
%%
c_idx = 1;
p_idx = 1;
b_idx = 1;
m_idx = 1;
%%
%%
X_rad=[];
Y_rad=[];
Z_rad=[];
Dop_rad =[];
SNR_rad =[];
Az_rad = [];
El_rad = [];
TS_rad = [];
I_yolo_d = [];
Label = [];
Idx_d = 1;
%% 
format long
bagfile = '../../sample.bag';

[c_idx,p_idx,b_idx,m_idx,X_rad,Y_rad,Z_rad,Dop_rad,SNR_rad,Az_rad,El_rad,TS_rad,Label,Idx_d] = gen_rad_only_data(bagfile,c_idx,p_idx,m_idx,b_idx,X_rad,Y_rad,Z_rad,Dop_rad,SNR_rad,Az_rad,El_rad,TS_rad,Label,Idx_d);


%%
writeNPY(X_rad,'../../X_rad_only.npy');
writeNPY(Y_rad,'../../Y_rad_only.npy');
writeNPY(Z_rad,'../../Z_rad_only.npy');
writeNPY(Dop_rad,'../../Dop_rad_only.npy');
writeNPY(SNR_rad,'../../SNR_rad_only.npy');
writeNPY(Az_rad,'../../Az_rad_only.npy');
writeNPY(El_rad,'../../El_rad_only.npy');
writeNPY(TS_rad,'../../TS_rad_only.npy');
writeNPY(Label,'../../Label_rad_only.npy');