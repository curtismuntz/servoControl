%Im_raw=(imread(listLeft(i).name));
%Im=rgb2ycbcr(imread(listLeft(i).name));
Im_raw=(imread("/home/me/Pictures/Webcam/2014-07-30-211338.jpg"));
Im=rgb2ycbcr(Im_raw);
Im_cr=Im(:,:,3);
Im_y=Im(:,:,1);
Im_cb=Im(:,:,2);


subplot(231);
imshow(Im_raw);
title('raw image');

subplot(232);
imshow(Im);
title('YCbCr');


subplot(233)
imshow(Im_cr);
title('Chroma');



S=strel('disk',9); %bounce
subplot(234)
Bw=im2bw(Im_cr,0.57);
imshow(Bw);
title('im2bw');



subplot(235) 
Bw=imerode(Bw,S);
Bw=imdilate(Bw,S);

imshow(Bw);
title('Isolated object');

stats=regionprops(Bw);
index=[0 0];
totArea=0;
for j=1:length(stats)
totArea=totArea+stats(j).Area;
end

for j=1:length(stats)
index=index+stats(j).Centroid*(stats(j).Area/totArea);
end

subplot(236)
imshow(Im_raw), title('Tracked Object'); hold on;
plot(index(1), index(2), 's', 'color', 'cyan'); hold off;

pause(0.01);

figure
imshow(Im_y);
figure
imshow(Im_cb);