function varargout = ProjetoSexta(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ProjetoSexta_OpeningFcn, ...
                   'gui_OutputFcn',  @ProjetoSexta_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end


function ProjetoSexta_OpeningFcn(hObject, eventdata, handles, varargin)


handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


function varargout = ProjetoSexta_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;


%%%% Acima é padrão da interface gráfica
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  Onde começa o cod 

function pushbutton1_Callback(hObject, eventdata, handles)
% Carrega a Imagem
global r
global save

[r, user_cance] = imgetfile();
    if user_cance
        msgbox(sprintf('Selecione uma Imagem!'),'Error','Error');
        return
    end   
r = imread(r);
axes(handles.axes1);
imshow(r);
save = r;

axes(handles.axes2);
imhist(r);
axis(handles.axes2, 'off');

% a = figure; imhist(r); 
% 
% saveas(a,"histograma.png");


function pushbutton2_Callback(hObject, eventdata, handles)
% Reseta imagem
global r

axes(handles.axes1);
imshow(r);

global save
save = r;


axes(handles.axes2);
imhist(r);
axis(handles.axes2, 'off');

function pushbutton3_Callback(hObject, eventdata, handles)
% Tons de cinza    LEMBRANDO QUE R É A IMAGEM
global r 
dst = rgb2gray(r);

%imwrite(dst,"Dessaturada.jpg");   %Salva a imagem na pasta

global save
save = dst;

axes(handles.axes1);
imshow(dst);

axes(handles.axes2);
imhist(dst);
axis(handles.axes2, 'off');

% a = figure; imhist(dst); 
% 
% saveas(a,"histogramadessat.png");



function pushbutton4_Callback(hObject, eventdata, handles)
% Imagem negativo
global r 
% Opção1
% neg = imcomplement(r);
%Opção2
neg = uint8(-1*(double(r)-255));
axes(handles.axes1);
imshow(neg);

global save

save = neg;

axes(handles.axes2);
imhist(neg);
axis(handles.axes2, 'off');

% a = figure; imhist(neg); 
% 
% saveas(a,"histogramanegat.png");

function pushbutton5_Callback(hObject, eventdata, handles)
% Aumentar Contraste
global r
Mc = imadjust(r,[.4 .4 .4; .7 .7 .7],[0 0 0; 1 1 1]);
% limiar = 180;
% a = double(r)/limiar;
% b = a.^2;
% Mc = uint8(b*limiar);
axes(handles.axes1);
imshow(Mc);

global save
save = Mc;

axes(handles.axes2);
imhist(Mc);
axis(handles.axes2, 'off');

% a = figure; imhist(Mc); 
% 
% saveas(a,"histogramamaiscontrast.png");

function pushbutton6_Callback(hObject, eventdata, handles)
% Diminuir Contraste
global r
mc = imadjust(r,[0 0 0; 1 1 1],[.1 .1 .1; .6 .6 .6]);
%ta errado
% limiar = 180;
% a = double(r)*limiar;
% b = a.^2;
% mc = uint8(b/limiar);
axes(handles.axes1);
imshow(mc);

global save
save = mc;

axes(handles.axes2);
imhist(mc);
axis(handles.axes2, 'off');

% a = figure; imhist(mc); 
% 
% saveas(a,"histogramamenoscontr.png");

function pushbutton7_Callback(hObject, eventdata, handles)
% gaussian
global r
gau = imnoise(r, 'gaussian');
axes(handles.axes1);
imshow(gau);

global save
save = gau;

axes(handles.axes2);
imhist(gau);
axis(handles.axes2, 'off');

% a = figure; imhist(gau); 
% 
% saveas(a,"histogramagauss.png");


function pushbutton8_Callback(hObject, eventdata, handles)
% pepper
global r
sep = imnoise(r, 'salt & pepper');
axes(handles.axes1);
imshow(sep);

global save
save = sep;

axes(handles.axes2);
imhist(sep);
axis(handles.axes2, 'off');

% a = figure; imhist(sep); 
% 
% saveas(a,"histogramapepper.png");


function pushbutton9_Callback(hObject, eventdata, handles)
% speckle
global r
k = imnoise(r, 'speckle');
axes(handles.axes1);
imshow(k);

global save
save = k;

axes(handles.axes2);
imhist(k);
axis(handles.axes2, 'off');

% a = figure; imhist(k); 
% 
% saveas(a,"histogramaspecle.png");



function pushbutton18_Callback(hObject, eventdata, handles)
% Detecção de borda canny
global r 
gray = rgb2gray(r);
i1 = edge(gray, 'canny');
axes(handles.axes1);
imshow(i1);



axes(handles.axes2);
imhist(i1);
axis(handles.axes2, 'off');

 global save
 save = i1;

%  a = figure; imhist(i1); 
%  
%  saveas(a,"histogramacanny.png");


function pushbutton19_Callback(hObject, eventdata, handles)
% Detecção de Borda sobel
global r
gray = rgb2gray(r);
% i2 = edge(gray, 'sobel');
    sobel = conv2(gray,[1/9,2/9,1/9;0/9,0/9,0/9;-1/9,-2/9,-1/9], 'same'); %sobel vertical
    sobel2 = conv2(sobel,[-1/9,0/9,1/9;-2/9,0/9,2/9;-1/9,0/9,1/9], 'same'); %sobel horizontal

axes(handles.axes1);
imshow(sobel2);

global save
save = sobel2;

axes(handles.axes2);
imhist(sobel2);
axis(handles.axes2, 'off');

% a = figure; imhist(sobel2); 
% 
% saveas(a,"histogramasobel.png");



function pushbutton20_Callback(hObject, eventdata, handles)
% Detecção de borda prewitt
global r
gray = rgb2gray(r);
i3 = edge(gray, 'prewitt');
axes(handles.axes1);
imshow(i3);

global save
save = i3;

axes(handles.axes2);
imhist(i3);
axis(handles.axes2, 'off');

% a = figure; imhist(i3); 
% 
% saveas(a,"histogramaprewitt.png");


function pushbutton23_Callback(hObject, eventdata, handles)
% terceiro
global r
f5 = (1/25)*[2 1 1 1 2, 1 1 2 1 1, 1 2 1 2 1, 1 1 1 1 1, 1 1 1 1 1];
b = imfilter(r, f5);
axes(handles.axes1);
imshow(b);

global save
save = b;

axes(handles.axes2);
imhist(b);
axis(handles.axes2, 'off');

% a = figure; imhist(b); 
% 
% saveas(a,"histogramaterceiro.png");



function pushbutton22_Callback(hObject, eventdata, handles)
% segundo
global r
f4 = (1/16)*[1 1 1 1, 1 1 1 1, 1 1 1 1, 1 1 1 1];
b = imfilter(r, f4);
axes(handles.axes1);
imshow(b);

global save
save = b;

axes(handles.axes2);
imhist(b);
axis(handles.axes2, 'off');

% a = figure; imhist(b); 
% 
% saveas(a,"histogramasegundo.png");



function pushbutton21_Callback(hObject, eventdata, handles)
% primeiro
global r
f3 = (1/9)*[1 1 1, 1 1 1, 1 1 1];
b = imfilter(r, f3);
axes(handles.axes1);
imshow(b);

global save
save = b;

axes(handles.axes2);
imhist(b);
axis(handles.axes2, 'off');


% a = figure; imhist(b); 
% 
% saveas(a,"histogramapimeiro.png");




function slider4_Callback(hObject, eventdata, handles)
global r

%%%SLIDER de brilho

valor = get(hObject,'Value')
Mb = r + valor;
axes(handles.axes1);
imshow(Mb);

global save
save = Mb;

axes(handles.axes2);
imhist(Mb);
axis(handles.axes2, 'off');

% a = figure; imhist(r + 100); 
% 
% saveas(a,"histogramamais100brilho.png");
% 
% b = figure; imhist(r - 100); 
% 
% saveas(b,"histogramamenosbrilho-100.png");


function slider4_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function pushbutton26_Callback(hObject, eventdata, handles)

global save;

imwrite(save,imputfile());



function pushbutton30_Callback(hObject, eventdata, handles)

global r                    %PASSA ALTA

gray = rgb2gray(r);
a = fft2(double(gray));
a1 = fftshift(a);

%escolha do filtro de Gaussian
[m n] = size(a);
p = 10;
x = 0 : n-1;
y = 0 : m-1;
[x y] = meshgrid(x,y);
cx = 0.5*n;
cy = 0.5*m;

passaB = exp(-((x-cx).^2+(y-cy).^2)./(2*p).^2);
passaA = 1-passaB; 

j = a1.*passaB;
j1 = ifftshift(j);
b1 = ifft2(j1);
k = a1.*passaA;
k1 = ifftshift(k);
b2 = ifft2(k1);

axes(handles.axes1);
imshow(b2);
axes(handles.axes2);
imhist(b2);

global save
save = b2;

%  a = figure; imhist(b2); 
%  
%  saveas(a,"histogramafourierPASSAALTA.png");




function pushbutton31_Callback(hObject, eventdata, handles)

global r                    %PASSA baixa

gray = rgb2gray(r);
a = fft2(double(gray));
a1 = fftshift(a);

%escolha do filtro de Gaussian
[m n] = size(a);
p = 10;
x = 0 : n-1;
y = 0 : m-1;
[x y] = meshgrid(x,y);
cx = 0.5*n;
cy = 0.5*m;

passaB = exp(-((x-cx).^2+(y-cy).^2)./(2*p).^2);
passaA = 1-passaB; 

j = a1.*passaB;
j1 = ifftshift(j);
b1 = uint8(ifft2(j1));
k = a1.*passaA;
k1 = ifftshift(k);
b2 = ifft2(k1);

axes(handles.axes1);
imshow(b1);
axes(handles.axes2);
imhist(b1);

global save
save = b1;



function pushbutton32_Callback(hObject, eventdata, handles)

global r;    % houghlines

dst = rgb2gray(r);
rot = imrotate(dst,33,'crop');
BW = edge(rot, 'canny');
[H,T,R] = hough(BW);

P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
x = T(P(:,2)); y = R(P(:,1));

lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
axes(handles.axes1);
imshow(rot), hold on
max_len = 0;

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end

plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');

axes(handles.axes2);
imhist(rot);

global save;

F = getframe(handles.axes1);
save = frame2im(F);

 a = figure; imhist(rot); 
 
 saveas(a,"histogramaHOUGH.png");
