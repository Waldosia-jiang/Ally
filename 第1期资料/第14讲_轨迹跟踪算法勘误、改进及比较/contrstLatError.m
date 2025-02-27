% 比较四种轨迹跟踪算法的横向误差
% 作者:Ally
% 日期:2021/04/29
clc
clear
close all
load latError_PP.mat
load latError_Stanley.mat
load latError_LQR.mat
load latError_MPC.mat

%% 画图
figure
plot(latError_PP(:,1),latError_PP(:,2),'b')
hold on
plot(latError_Stanley(:,1),latError_Stanley(:,2),'k')
plot(latError_LQR(:,1),latError_LQR(:,2),'g')
plot(latError_MPC(:,1),latError_MPC(:,2),'r')
legend('PP','Stanley','LQR','MPC')
xlabel('参考轨迹索引号')
ylabel('横向误差')

