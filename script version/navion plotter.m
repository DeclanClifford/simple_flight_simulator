%{
xwing1(:, 1) = xa(:, 1);
xwing1(:, 2) = xb(:, 1);
xwing2(:, 1) = xa(:, 2);
xwing2(:, 2) = xb(:, 2);
xwing3(:, 1) = xa(:, 3);
xwing3(:, 2) = xb(:, 3);
xwing4(:, 1) = xa(:, 4);
xwing4(:, 2) = xb(:, 4);
xtail1(:, 1) = xa(:, 5);
xtail1(:, 2) = xb(:, 5);
xtail2(:, 1) = xa(:, 6);
xtail2(:, 2) = xb(:, 6);
xfus(:, 1) = xa(:, 7);
xfus(:, 2) = xb(:, 7);
xref(:, 1) = [0; 0; 6];
xref(:, 2) = [0; 0; -6];
xref2(:, 1) = [0; 6; 0];
xref2(:, 2) = [0; -6; 0];

plot3(xwing1(1, :), xwing1(2, :), xwing1(3, :), 'r',...
      xwing2(1, :), xwing2(2, :), xwing2(3, :), 'g',...
      xwing3(1, :), xwing3(2, :), xwing3(3, :), 'b',...
      xwing4(1, :), xwing4(2, :), xwing4(3, :), 'c',...
      xtail1(1, :), xtail1(2, :), xtail1(3, :), 'k',...
      xtail2(1, :), xtail2(2, :), xtail2(3, :), 'g',...
      xfus(1, :), xfus(2, :), xfus(3, :), 'k',...
      xc(1, :), xc(2, :), xc(3, :), '.k',...
      xref(1, :), xref(2, :), xref(3, :), '--k',...
      xref2(1, :), xref2(2, :), xref2(3, :), '--k')

xlim([-6, 6])
ylim([-6, 6])
zlim([-6, 6])

%}
  