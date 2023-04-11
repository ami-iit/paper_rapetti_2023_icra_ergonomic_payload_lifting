TOLL = 1e-12;

% Analyze solution for Robot 1 single-robot optimization
analyzeRankLinearSystem(out.Aeq1.Data, out.beq1.Data, out.Aeq1.Time, 'A Robot1', TOLL)

% Analyze solution for Robot 2 single-robot optimization
analyzeRankLinearSystem(out.Aeq2.Data, out.beq2.Data, out.Aeq1.Time, 'A Robot2', TOLL)

% Analyze solution for total system optimization
analyzeRankLinearSystem(out.Aeq.Data, out.beq.Data, out.Aeq1.Time, 'A Total System', TOLL)


% Analyze inverse dynamics lambda
analyzeRankLinearSystem(out.Lambda.Data, [], out.Aeq1.Time, 'Lambda Total System', TOLL)

% Analyze inverse dynamics lambda for robot 1
analyzeRankLinearSystem(out.Lambda1.Data, [], out.Aeq1.Time, 'Lambda Robot 1', TOLL)

% Analyze inverse dynamics lambda for robot 2
analyzeRankLinearSystem(out.Lambda2.Data, [], out.Aeq1.Time, 'Lambda Robot 2', TOLL)




function analyzeRankLinearSystem(Aeq, beq, Time, Title, tollerance)

[m, n, k] = size(Aeq);
AeqCell = mat2cell(Aeq, m, n, ones(k,1));

ranksAeq = cellfun(@(A) rank(A, tollerance), AeqCell);
yLimMin = min(ranksAeq)-1;
yLimMax = max(ranksAeq)+1;

figure()
plot(Time, squeeze(ranksAeq));
hold on

if(not(isempty(beq)))
    [m, n, k] = size(beq);
    beqCell  = mat2cell(beq, m, n, ones(k,1));
    
    ranksAeqbeq = cellfun(@(A, b) rank([A, b], tollerance), AeqCell, beqCell);
    yLimMax = max(ranksAeqbeq)+1;
    
    plot(Time, squeeze(ranksAeqbeq));
    
    legend('rank(A)', 'rank(A|b)')
end


ylabel('Rank')
xlabel('Time [s]')


title(Title)

xlim([-inf, inf])
ylim([yLimMin yLimMax])

end
