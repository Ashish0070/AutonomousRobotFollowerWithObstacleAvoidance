recMsg1 = t1.LatestMessage;
i1 = readImage(recMsg1);
recMsg2 = t2.LatestMessage;
i2 = readImage(recMsg2);
j1 = rgb2gray(i1);
j2 = rgb2gray(i2);
disparityRange = [0 48];
dM = disparitySGM(j1,j2,'DisparityRange',disparityRange,'UniquenessThreshold',18);

dM(isnan(dM)) = 0;

B = dM(:, 221:450);
kount1 = sum(B(:) >= 9 & B(:) <= 20);
kount2 = sum(B(:) >= 20.1 & B(:) <= 33);
kount3 = sum(B(:) > 33);
figure
imshow(B,disparityRange)
title('Disparity Map')
colormap jet
colorbar
A = stereoAnaglyph(i1,i2);
C = A(:, 221:450);
figure
imshow(C)
p = kount1 /(kount1 + kount2 + kount3);
q = kount2 /(kount1 + kount2 + kount3) ;
r = kount3 /(kount1 + kount2 + kount3);