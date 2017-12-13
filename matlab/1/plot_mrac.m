%-------- Print eps plots -----

close all;

%Set matlab interpreter to latex
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

%Set figures positions and size
fig_xpos = 500;
fig_ypos = 250;
fig_width = 600;
fig_height = 250;
fig_pos = [fig_xpos fig_ypos fig_width fig_height];
pos_pct = .075;

switch changed
    case 1
        str1 = strcat('$\gamma=',num2str(gamma_1),'$');
        str2 = strcat('$\gamma=',num2str(gamma_2),'$');
        file_name = strcat('gamma',num2str(gamma_1),'gamma',num2str(gamma_2));
        
    case 2 
        str1 = '$P_1$';
        str2 = '$P_2$';
        file_name = 'P1P2_2';
        
    case 3
        str1 = '$P_{m_1}$';
        str2 = '$P_{m_2}$';
        file_name = 'Pm1Pm2';
        
    case 4
        y0_str_1 = '[\,';
        for i=1:length(y0)
            if i ~= length(y0)
                y0_str_1 = strcat(y0_str_1,num2str(y0_1),'\,\,');
            else
                y0_str_1 = strcat(y0_str_1,num2str(y0_1),'\,]');
            end
        end
        y0_str_2 = '[\,';
        for i=1:length(y0)
            if i ~= length(y0)
                y0_str_2 = strcat(y0_str_2,num2str(y0(i)),'\,\,');
            else
                y0_str_2 = strcat(y0_str_2,num2str(y0(i)),'\,]');
            end
        end
        str1 = strcat('$y(0)=',y0_str_1,'^T$');
        str2 = strcat('$y(0)=',y0_str_2,'^T$');
        file_name = 'y01y02';
end

path_modtheta = strcat('../../relatorio/figs/1/modtheta/',sim_str,file_name,'.eps');
path_e0 = strcat('../../relatorio/figs/1/e0/',sim_str,file_name,'.eps');

%--------------- Fig1: mod theta -------------
figure(1);clf;
set(gcf,'position',fig_pos);

plot(T_1,modtt_1);grid on;hold on;
plot(T_2,modtt_2);

title('$||\theta||$');
legend(str1,str2,'Location','SouthEast');

if PRINT
    print(path_modtheta,'-depsc2','-painters')
end

%--------------- Fig3: e -------------
figure(2);clf;
set(gcf,'position',[fig_pos(1:2) fig_pos(3) 2*fig_pos(4)]);

h1 = subplot(211);
plot(T_1,e0_1);
title(strcat('$e0_1$ com~ ', str1));

h2 = subplot(212);
plot(T_2,e0_2);
title(strcat('$e0_2$ com~ ', str2));
% h2.YLim = h1.YLim;

set(gcf,'position',fig_pos);

if PRINT
    print(path_e0,'-depsc2','-painters')
end
