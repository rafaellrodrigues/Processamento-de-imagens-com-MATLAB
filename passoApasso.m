% x = imread('haduk.jpg');
% imshow(x)

%%
% whos x


%% Negativo da imagem
% x = imread('haduk.jpg');
% y = uint8(-1*(double(x)-255));
% imshow(y)


%% Aumentar e reduzir brilho
% x = imread('haduk.jpg');
% z = x - 60;
% z = x + 60;
% imshow(z)


%% Aumentar e reduzir o contraste
% x = imread('lula.jpg');
% limiar = 180;
% a = double(x)/limiar;
% imshow(a)
% 
% b = a.^2;
% c = uint8(b*limiar);
% subplot(1,2,1); imshow(x);  title('original')
% subplot(1,2,2); imshow(c);  title('contraste aum')

%% Suavização ------------------com erro-------------
% x = imread('lula.jpg');
% suav1 = suav1(2:end-1, 2:end-1);
% size(suav1)
% 
% subplot(1,2,1); imshow(x)
% subplot(1,2,2); imshow(uint8(suav1))

%%
x = imread('haduk.jpg');
borda_v = convn(x, sobel_v);
borda_h = convn(x, sobel_h);
borda_v = borda_v(2:end-1, 2:end-1);
borda_h = borda_h(2:end-1, 2:end-1);

% figure
% subplot(1,2,1); 
% imshow(uint8(borda_v));
% title('Bordas verticais da imagem')
% 
% subplot(1,2,2);
% imshow(uint8(borda_h));
% title('Bordas horizontais da imagem')

borda = borda_v + borda_h;
subplot(1,2,1);
imshow(x);
title('original')

subplot(1,2,2);
imshow(uint8(borda));
title('bordas das imagens')



