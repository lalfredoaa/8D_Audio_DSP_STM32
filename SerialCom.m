clear;

[y,fs]=audioread("C:\Users\Elliot\Desktop\Adele2.wav");

y=randn(1,100000)*0.3;

buffersize=128;
s=serialport("COM8",1600000,"ByteOrder","big-endian");

q15_y= nearest(y*2^15);

readtemp=[];
readtemp2=[];

for i=0:(floor(numel(y)/buffersize)-1)
	write(s,q15_y((buffersize*i)+1:(i+1)*buffersize), "int16");
	
    data=read(s,256,"int16");
    readtemp=[readtemp data(1:128)];
    readtemp2=[readtemp2 data(129:256)];
    %data2=read(s,256,"int16");
    %readtemp2=[readtemp2 data2];
	i
end