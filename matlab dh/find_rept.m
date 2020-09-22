a=[];
for i=2:length(rawdata100segundos)
if rawdata100segundos(i-1,7)==rawdata100segundos(i,7)
    a = [a; i];
    rawdata100segundos(i,7)=0;
end
end

[ind,~,~] = find(rawdata100segundos==0);
rawdata100segundos(ind,:)=[];