function flag = judgeObs(path,X,Y,Z)
% 判断生成的曲线是否与与障碍物相交
flag = 0;
for i = 2:size(path,1)
    x = path(i,1);
    y = path(i,2);
    z_interp = interp2(X,Y,Z,x,y);
    if path(i,3) < z_interp
        flag = 1;
        break
    end
end