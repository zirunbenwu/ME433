function live_view(port)
% LIVE_VIEW  Live force + angle viewer (left = force, right = angle).
%
%   live_view('COM6')
%
% Pico must run the combined firmware that streams
%   raw,filt,time_ms,angle
% after receiving "0" (stream mode). Close the figure to stop.

    if nargin < 1
        port = 'COM6';
    end

    WINDOW_SEC   = 30;
    MAX_POINTS   = 600;
    AVG_PERIOD_S = 0.2;       % 5 averaged points per second
    BAUD         = 115200;

    s = serialport(port, BAUD, 'Timeout', 1);
    configureTerminator(s, "LF");
    cleanupSer = onCleanup(@() delete(s));
    pause(2.0);
    flush(s);

    % ---- Handshake: send "0", wait for STREAM or data ----
    gotHandshake = false;
    primedLine   = "";
    for attempt = 1:3
        flush(s);
        writeline(s, "0");
        fprintf('Attempt %d: sent ''0''...\n', attempt);
        tStart = tic;
        while toc(tStart) < 2.0
            if s.NumBytesAvailable == 0
                pause(0.05); continue;
            end
            ln = strtrim(readline(s));
            if ismissing(ln) || ln == "", continue; end
            if ln == "STREAM"
                fprintf('Pico acknowledged stream mode\n');
                gotHandshake = true; break;
            elseif count(ln, ",") == 3        % 4 fields = 3 commas
                fprintf('Data flowing: %s\n', ln);
                primedLine = ln;
                gotHandshake = true; break;
            else
                fprintf('  (ignored: %s)\n', ln);
            end
        end
        if gotHandshake, break; end
    end
    if ~gotHandshake
        error('No response from Pico - unplug/replug and retry');
    end

    fprintf('Setting up figure...\n');

    tBuf     = nan(1, MAX_POINTS);
    rawBuf   = nan(1, MAX_POINTS);     % raw force
    forceBuf = nan(1, MAX_POINTS);     % filtered force
    angBuf   = nan(1, MAX_POINTS);     % angle (deg)
    nPoints = 0;
    bucketStart = NaN;
    bucketRaw   = 0;
    bucketFilt  = 0;
    bucketAng   = 0;
    bucketN     = 0;
    partial     = "";

    fig = figure('Name', 'Live force + angle', 'NumberTitle', 'off', ...
                 'Position', [100 100 1100 500]);

    % ---- LEFT: force ----
    axF = subplot(1, 2, 1, 'Parent', fig);
    hRaw  = plot(axF, NaN, NaN, 'o-', 'LineWidth', 1, 'MarkerSize', 4, ...
                 'Color', [0.6 0.6 0.9]);
    hold(axF, 'on');
    hFilt = plot(axF, NaN, NaN, 'o-', 'LineWidth', 2, 'MarkerSize', 4, ...
                 'Color', [0.1 0.1 0.8]);
    hold(axF, 'off');
    grid(axF, 'on');
    xlabel(axF, 'Time (s)');
    ylabel(axF, 'Force (HX711 counts)');
    title(axF, sprintf('Load cell force - %g s avg', AVG_PERIOD_S));
    legend(axF, {'Raw', 'Filtered'}, 'Location', 'northwest');

    % ---- RIGHT: angle ----
    axA = subplot(1, 2, 2, 'Parent', fig);
    hAng = plot(axA, NaN, NaN, 'o-', 'LineWidth', 2, 'MarkerSize', 4, ...
                'Color', [0.8 0.2 0.2]);
    grid(axA, 'on');
    xlabel(axA, 'Time (s)');
    ylabel(axA, 'Angle (deg)');
    title(axA, 'Encoder angle');
    ylim(axA, [-190 190]);
    drawnow;

    if primedLine ~= ""
        ingestLine(primedLine);
    end

    fprintf('Entering main loop. Close the figure to stop.\n');

    lastPrint = tic;
    samplesRx = 0;
    diagShown = false;

    while isvalid(fig)
        % --- Read available serial bytes ---
        try
            nAvail = s.NumBytesAvailable;
        catch ME
            fprintf('Serial disconnected: %s\n', ME.message);
            break;
        end

        if nAvail > 0
            try
                bytes = read(s, nAvail, 'uint8');
            catch ME
                fprintf('Read error: %s\n', ME.message);
                break;
            end
            chunkStr = string(char(reshape(uint8(bytes), 1, [])));
            combined = partial + chunkStr;
            lines    = splitlines(combined);

            if ~diagShown && numel(lines) >= 2
                fprintf('First line received: "%s"\n', lines(1));
                diagShown = true;
            end

            if numel(lines) >= 2
                partial = lines(end);                  % maybe-incomplete tail
                completeLines = lines(1:end-1);
                for k = 1:numel(completeLines)
                    ln = strtrim(completeLines(k));
                    if strlength(ln) == 0, continue; end
                    if ingestLine(ln)
                        samplesRx = samplesRx + 1;
                    end
                end
            else
                partial = lines(1);
            end
        end

        % --- Update plots ---
        if nPoints > 0
            xv = tBuf(1:nPoints);
            set(hRaw,  'XData', xv, 'YData', rawBuf(1:nPoints));
            set(hFilt, 'XData', xv, 'YData', forceBuf(1:nPoints));
            set(hAng,  'XData', xv, 'YData', angBuf(1:nPoints));

            tNow = tBuf(nPoints);
            xlo  = max(0, tNow - WINDOW_SEC);
            xhi  = tNow + AVG_PERIOD_S;
            if xhi > xlo
                xlim(axF, [xlo xhi]);
                xlim(axA, [xlo xhi]);
            end

            % autoscale force y to recent data
            visIdx = xv >= xlo;
            yvals  = [rawBuf(visIdx), forceBuf(visIdx)];
            yvals  = yvals(~isnan(yvals));
            if ~isempty(yvals)
                lo = min(yvals); hi = max(yvals);
                pad = max(50, (hi - lo) * 0.1);
                ylim(axF, [lo - pad, hi + pad]);
            end
            % angle axis stays fixed at +/-190
        end

        if toc(lastPrint) >= 1.0
            fprintf('  rx %d/s   plotted: %d   bucket n=%d\n', ...
                    samplesRx, nPoints, bucketN);
            samplesRx = 0;
            lastPrint = tic;
        end

        drawnow limitrate;
        pause(0.05);
    end

    % stop the Pico's stream on exit
    try, write(s, uint8('x'), 'uint8'); pause(0.1); catch, end
    fprintf('Stopped.\n');

    % ----- nested helpers -----
    function ok = ingestLine(lnStr)
        ok = false;
        lnChar = char(lnStr);
        if startsWith(lnChar, 'STREAM') || startsWith(lnChar, 'STOP')
            return;
        end
        commas = strsplit(lnChar, ',');
        if numel(commas) ~= 4, return; end       % raw,filt,time,angle
        r = str2double(commas{1});
        f = str2double(commas{2});
        t = str2double(commas{3});
        a = str2double(commas{4});
        if any(isnan([r f t a])), return; end
        ingest(r, f, t, a);
        ok = true;
    end

    function commitBucket(endMs)
        if bucketN == 0, return; end
        centerS = (bucketStart + endMs) / 2 / 1000;
        if nPoints < MAX_POINTS
            nPoints = nPoints + 1;
        else
            tBuf(1:end-1)     = tBuf(2:end);
            rawBuf(1:end-1)   = rawBuf(2:end);
            forceBuf(1:end-1) = forceBuf(2:end);
            angBuf(1:end-1)   = angBuf(2:end);
        end
        tBuf(nPoints)     = centerS;
        rawBuf(nPoints)   = bucketRaw  / bucketN;
        forceBuf(nPoints) = bucketFilt / bucketN;
        angBuf(nPoints)   = bucketAng  / bucketN;
        bucketStart = NaN; bucketRaw = 0; bucketFilt = 0;
        bucketAng = 0; bucketN = 0;
    end

    function ingest(rawVal, filtVal, tMs, angVal)
        if isnan(bucketStart), bucketStart = tMs; end
        if tMs - bucketStart >= AVG_PERIOD_S * 1000
            commitBucket(tMs);
            bucketStart = tMs;
        end
        bucketRaw  = bucketRaw  + rawVal;
        bucketFilt = bucketFilt + filtVal;
        bucketAng  = bucketAng  + angVal;
        bucketN    = bucketN    + 1;
    end
end