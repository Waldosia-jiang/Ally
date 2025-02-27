function Bik_u = BaseFunction(i, k , u, NodeVector)

if k == 0       % 0次B样条
    if u >= NodeVector(i+1) && u < NodeVector(i+2)
        Bik_u = 1;
    else
        Bik_u = 0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % 支撑区间的长度
    if Length1 == 0       % 规定0/0 = 0
        Length1 = 1;
    end
    if Length2 == 0
        Length2 = 1;
    end
    Bik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end
