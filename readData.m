RadarData_file = ".\data\RadarData.csv";
Lane1_file = ".\data\Lane1_rtk.dat";
Lane2_file = ".\data\Lane2_rtk.dat";
Lane3_file = ".\data\Lane3_rtk.dat";
Lane1_file_A = ".\A_data\Lane1_rtk.xlsx";
Lane2_file_A = ".\A_data\Lane2_rtk.xlsx";
Lane3_file_A = ".\A_data\Lane3_rtk.xlsx";
RadarData_table = readtable(RadarData_file);
Lane1_table = readRtk(Lane1_file, 11, 10357);
Lane2_table = readRtk(Lane2_file, 11, 9505);
Lane3_table = readRtk(Lane3_file, 11, 9944);

Lane1_table_A = readtable(Lane1_file_A);
Lane2_table_A = readtable(Lane2_file_A);
Lane3_table_A = readtable(Lane3_file_A);
Lane1_A = Lane1_table_A{:, 2:12};
Lane2_A = Lane2_table_A{:, 2:12};
Lane3_A = Lane3_table_A{:, 2:12};

Lane1 = Lane1_table(Lane1_table(:, 1) ~= 0, :);
Lane2 = Lane2_table(Lane2_table(:, 1) ~= 0, :);
Lane3 = Lane3_table(Lane3_table(:, 1) ~= 0, :);
RadarData = RadarData_table.Variables;

function rtkline = readRtk(rtkPath, n_col, n_row)
    % Initialize an empty array to store rtkline
    rtkline = zeros(n_row, n_col);
    % Open the file
    fid = fopen(rtkPath, 'r');
    tline = fgetl(fid);
    cnt = 0;
    while ischar(tline)
        cnt = cnt + 1;
        if tline(2) == 'B'
            tline = fgetl(fid);
            continue;
        end
        
        index = 1;
        while (tline(index) ~= ';')
            index = index + 1;
        end
        index = index + 1;
        % Split the line by ';' to get the values
        values = str2double(strsplit(tline(index:end), ','));

        % Assign the values to the rtk structure
        rtkline(cnt, 1) = values(1);
        rtkline(cnt, 2) = values(2);
        rtkline(cnt, 3) = values(3);
        rtkline(cnt, 4) = values(4);
        rtkline(cnt, 5) = values(5);
        rtkline(cnt, 6) = values(6);
        rtkline(cnt, 7) = values(7);
        rtkline(cnt, 8) = values(8);
        rtkline(cnt, 9) = values(9);
        rtkline(cnt, 10) = values(10);
        rtkline(cnt, 11) = values(11);


        % Read the next line
        tline = fgetl(fid);
    end

    % Close the file
    fclose(fid);

end
