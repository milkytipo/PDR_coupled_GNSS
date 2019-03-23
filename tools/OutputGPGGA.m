function OutputGPGGA(latitude, longitude, height, recv_time, satnum, path, name,posiCheck)
    
    BJhour = recv_time.hour;
    BJmin = recv_time.min;
    BJsec = recv_time.sec;
    logName = strcat(path, name, '_GPGGA.txt');
    if isnan(latitude)
        useful = 0;  % the positioning result is null
        latitude = 0;
        longitude = 0;
        height = 0;
    else
        useful = 1;
        latitude = (latitude-fix(latitude))*60 + fix(latitude)*100;
        longitude = (longitude-fix(longitude))*60 + fix(longitude)*100;    
    end
    head = '$GPGGA';
    fid = fopen(logName,'at');
    fprintf(fid,'%s,%2.2d%2.2d%06.3f,%09.4f,N,%010.4f,E,%1.1d,%2.2d,%09.3f,M \n',...
        head, BJhour, BJmin, BJsec, latitude, longitude, posiCheck, satnum, height); 
    fclose(fid);
end