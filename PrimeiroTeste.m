a = imgetfile();
a = imread(a);

a = rgb2gray(a);
imshow(a);

%% errado?
pq = paddedsize(size(a));
% imshow(pq);
%% 








%% 
% a = imgetfile();
% a = imread('lula.jpg');
% dst = rgb2gray(a);
% 
% BW = edge(dst,'canny');
%% hough

%% suavização
% f1 = (1/9)*[1 1 1,1 1 1,1 1 1];
% b = imfilter(a, f1);
% imshow(b);

%%
% %criando as mascara de convolução
% mx = [-1, 0, 1; -2, 0, 2; -1, 0, 1];
% my = [1, 2, 1; 0, 0, 0; -1, -2, -1];
% 
% %cal o gradiente (em x e y)
% gx = conv2(dst,mx);
% gy = conv2(dst,my);
% 
% %magnitude do gradiente
% gxy = sqrt(gx.^2 + gy.^2);
% 
% figure, imshow(gxy);

%%
% paal1 = edge(dst, 'Sobel'); %Filtro de Passa alta aplicando o método de SOBEL
% 
% paal2 = edge(dst, 'Roberts');    %Filtro de Passa alta aplicando o método de Roberts
% 
% figure;
% montage({dst, paal1, paal2 },'Size',[1 3]);                        %   Montagem Lado à lado
% title("Original Dessaturada / Passa Alta (Sobel) / Passa Alta (Roberts)");
%  
 
% g = rgb2gray(a);
% 
% i1 = edge(g,'prewitt');
% 
% imshow(i1);

% g = imnoise(a, 'gaussian');
% imshow(g);

% ps = imnoise(a, 'salt & pepper');
% imshow(ps);

% sp = imnoise(a, 'speckle');
% imshow(sp);

% Mc = imadjust(a,[.7 .7 .7; 1 1 1],[0 0 0; 1 1 1]);   % mais contraste
% 
% mc = imadjust(a,[.7 .7 .7; 1 1 1],[.3 .3 .3; .7 .7 .7]);  % menos constraste
% 
% imshow(Mc);
% 
% imshow(mc);

%% 
% a = imread('lula.jpg', 'tif');
% image(a)


%%
% imwrite(a, 'lula.jpg', 'tif')

%%
% imread('lula.jpg')
% [X, MAPA] = imread('lula.jpg');
% imshow(X, MAPA);