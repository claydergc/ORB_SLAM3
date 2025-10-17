A3mp = load('map_points_ratio.txt');
A3mp_filtered = A3mp(A3mp(:,3) <= 8, :);

A3mp_filtered(:,1) = A3mp_filtered(:,1) - A3mp_filtered(1,1);
plot(A3mp_filtered(:,1), A3mp_filtered(:,2), 'b-')
hold on
%figure
plot(A3mp_filtered(:,1), A3mp_filtered(:,3)*100, 'r-')

plot(A3mp_filtered(:,1), (A3mp_filtered(:,4)-2)*100, 'g-')

legend('New map points', 'Delta pitch x 100', 'OS State');

xlabel('Time [s]')
%ylabel('Map points/Delta Pitch')
title('MADMAX - A3')
hold off

ax = gca;
ax.FontSize = 25;  % Font Size of 15

saveas(gcf,'A3_LOST_MADMAX.png')


%%

A4mp = load('A4_map_points_ratio.txt');
A4mp = load('map_points_ratio.txt');
A4mp_filtered = A4mp(A4mp(:,3) <= 11, :);

A4mp_filtered(:,1) = A4mp_filtered(:,1) -1543763302;
plot(A4mp_filtered(:,1), A4mp_filtered(:,2), 'b-')
hold on
%figure
plot(A4mp_filtered(:,1), A4mp_filtered(:,3)*100, 'r-')

plot(A4mp_filtered(:,1), (A4mp_filtered(:,4)-2)*100, 'g-')

legend('New map points', 'Delta pitch x 100', 'OS State');

xlabel('Time [s]')
%ylabel('Map points/Delta Pitch')
title('MADMAX - A4')
hold off

ax = gca;
ax.FontSize = 25;  % Font Size of 15

saveas(gcf,'A4_LOST_MADMAX.png')

%%

A5mp = load('map_points_ratio.txt');
A5mp_filtered = A5mp(A5mp(:,3) <= 8, :);

A5mp_filtered(:,1) = A5mp_filtered(:,1) - A5mp_filtered(1,1);
plot(A5mp_filtered(:,1), A5mp_filtered(:,2), 'b-')
hold on
%figure
plot(A5mp_filtered(:,1), A5mp_filtered(:,3)*100, 'r-')

plot(A5mp_filtered(:,1), A5mp_filtered(:,4)*10, 'm-')

plot(A5mp_filtered(:,1), (A5mp_filtered(:,5)-2)*100, 'g-')

legend('New map points', 'Delta pitch x 100', 'Search radius x 10' ,'OS State');

xlabel('Time [s]')
%ylabel('Map points/Delta Pitch')
title('MADMAX - A5')
hold off

ax = gca;
ax.FontSize = 25;  % Font Size of 15

saveas(gcf,'A5_LOST_MADMAX.png')


%%

%F3mp = load('F3_map_points_ratio.txt');
F3mp = load('map_points_ratio.txt');
F3mp_filtered = F3mp(F3mp(:,3) <= 20, :);
F3mp_filtered(:,1) = F3mp_filtered(:,1) - 1543578974;

plot(F3mp_filtered(200:end,1), F3mp_filtered(200:end,2), 'b-')
hold on
plot(F3mp_filtered(200:end,1), F3mp_filtered(200:end,3)*100, 'r-')
plot(F3mp_filtered(200:end,1), (F3mp_filtered(200:end,4)-2)*100, 'g-')

legend('New map points', 'Delta pitch x 100', 'OS State');

xlabel('Time [s]')
%ylabel('Map points/Delta Pitch')
title('MADMAX - F3')
hold off

ax = gca;
ax.FontSize = 25;  % Font Size of 15

saveas(gcf,'F3_LOST_MADMAX.png')

%%

%F3mp = load('F3_map_points_ratio.txt');
A6mp = load('map_points_ratio.txt');
A6mp_filtered = A6mp(A6mp(:,3) <= 20, :);

A6mp_filtered(:,1) = A6mp_filtered(:,1) - 1543764592;
plot(A6mp_filtered(3800:end,1), A6mp_filtered(3800:end,2), 'b-')
hold on
plot(A6mp_filtered(3800:end,1), A6mp_filtered(3800:end,3)*100, 'r-')
plot(A6mp_filtered(3800:end,1), (A6mp_filtered(3800:end,4)-2)*100, 'g-')

legend('New map points', 'Delta pitch x 100', 'OS State');

xlabel('Time [s]')
%ylabel('Map points/Delta Pitch')
title('MADMAX - A6')
hold off

ax = gca;
ax.FontSize = 25;  % Font Size of 15

saveas(gcf,'A6_LOST_MADMAX.png')

%%

%F3mp = load('F3_map_points_ratio.txt');
Aptmp = load('map_points_ratio_Apt.txt');
Aptmp_filtered = Aptmp(Aptmp(:,3) <= 20, :);
Aptmp_filtered(:,1) = Aptmp_filtered(:,1) - 1758089196.327782154;
plot(Aptmp_filtered(:,1), Aptmp_filtered(:,2), 'b-')
hold on
plot(Aptmp_filtered(:,1), Aptmp_filtered(:,3)*100, 'r-')
plot(Aptmp_filtered(:,1), (Aptmp_filtered(:,4)-2)*100, 'g-')

legend('New map points', 'Delta pitch x 100', 'OS State');

xlabel('Time [s]')
%ylabel('Map points/Delta Pitch')
title('Apt - 16:03')
hold off

ax = gca;
ax.FontSize = 25;  % Font Size of 15

saveas(gcf,'Apt_1603_LOST.png')