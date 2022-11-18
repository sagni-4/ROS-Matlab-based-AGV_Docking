robotposes=[1 1;
           -1 1;
           1 -1;
          -1 -1];
endpose=[2 3;
        -2 3;
        2 -3;
       -2 -3];
angles=zeros(16,1);
p=zeros(2,1);q=zeros(2,1); 
c=1;
for i=1:4
  
  for  j=1:4
      p=endpose(j,:);
      q=robotposes(i,:);
      a= atand((p(2)-q(2))/(p(1)-q(1)));
      angles(c)=a;
      c=c+1;
  end
end