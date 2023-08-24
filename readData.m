RadarData_file = ".\data\RadarData.csv";
Lane1_file = ".\data\Lane1_rtk.xlsx";
Lane2_file = ".\data\Lane2_rtk.xlsx";
Lane3_file = ".\data\Lane3_rtk.xlsx";
RadarData_table = readtable(RadarData_file);
Lane1_table = readtable(Lane1_file);
Lane2_table = readtable(Lane2_file);
Lane3_table = readtable(Lane3_file);

RadarData = RadarData_table.Variables;
Lane1 = Lane1_table{:, 2:12};
Lane2 = Lane2_table{:, 2:12};
Lane3 = Lane3_table{:, 2:12};

Lane1_sp = sqrt(Lane1(:, 6).^2 + Lane1(:, 7).^2);