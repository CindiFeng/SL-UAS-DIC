function res = JsonLoad(file)
    if ~exist(file, 'file')
        error('%s does not exist', file);
    end
    fp = fopen(file, 'r');
    c = onCleanup(@()fclose(fp));
    buf = fread(fp, inf, 'char');
    buf = char(reshape(buf, 1, []));
    res = jsondecode(buf);
end