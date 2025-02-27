function k=getCur(x,y)
ta=sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
tb=sqrt((x(3)-x(2))^2+(y(3)-y(2))^2);
M=[1,-ta,ta^2;
    1,0,0;
    1,tb,tb^2];
A=M\x;
B=M\y;
k=(2*(A(2)*B(3)-A(3)*B(2)))/((A(2)^2+B(2)^2)^1.5+1e-10);
end