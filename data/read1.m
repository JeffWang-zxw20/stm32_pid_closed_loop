fileID = fopen('data2.txt');
C = textscan(fileID,'%f');
fclose(fileID);

A = C{1,1};
Odd =[];
Eve=[];
for i = 1:length(A)
    if mod(i,2) ==1
        Odd =[Odd,A(i)];
    else
        Eve = [Eve,A(i)];
    end
end
Eve = [Eve,0];
t = linspace(0,length(Eve),length(Eve));
plot(t,Odd,t,Eve);
