function JsonDump(file, pld)
    fp = fopen(file, 'w');
    c = onCleanup(@()fclose(fp));
    fwrite(fp, jsonencode(pld));
end

