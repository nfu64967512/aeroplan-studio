function R = optimalGainsCubic(opts)
%MGS.ZARCHAN.OPTIMALGAINSCUBIC  Zarchan Listing 26.4 -- matrix Riccati sweep
%   for the SIX optimal control gains of a CUBIC (third-order) flight control
%   system, and with them the effective navigation ratio of Fig. 26.14.
%
%   R = mgs.zarchan.optimalGainsCubic()
%   R = mgs.zarchan.optimalGainsCubic('H', 5e-4)             coarser, faster
%   R = mgs.zarchan.optimalGainsCubic('WZ', 100)             zero far upstream
%   R = mgs.zarchan.optimalGainsCubic('crosscheck', false)   skip the 26.1 leg
%                                          (also skips the H-scaling leg)
%   R = mgs.zarchan.optimalGainsCubic('assert', false, 'render', false)
%
%   Line-by-line port of Listing 26.4 (zarchan.txt:9772-9785), which is
%   Listing 26.1 with the cubic flight-control-system matrices swapped in.
%
% ===========================================================================
%   UNITS.  There is no length, no mass and no g anywhere in this listing.
%   The Riccati sweep is dimensionally homogeneous in whatever unit y carries,
%   so nothing here needs the book's feet.  Time is SECONDS throughout and
%   omega/omega_z are rad/s.  The gains carry the units of the guidance law
%   nc = C1*y + C2*ydot + C3*nT + C4*e + C5*edot + C6*eddot
%   (zarchan.txt:9768), i.e. with y in ft and nc in ft/s^2: C1 [1/s^2],
%   C2 [1/s], C3 [-], C4 [-], C5 [s], C6 [s^2].  NP = C2*tgo is dimensionless.
% ===========================================================================
%
%   WHAT IT IS.  A backward (in time-to-go) integration of
%       Sdot = S*F + F'*S - C'*B*C,   S(0) = SF = e1*e1',   C = G'*S/gamma
%   for the six-state model of Fig. 26.13: x = [y ydot nT e edot eddot]', with
%   the tail-controlled airframe modelled as (1 - s^2/wz^2) over
%   (1+s*tau)(1 + 2*zeta*s/w + s^2/w^2).  T in every array is TIME TO GO.
%
%   *** WHAT IT CANNOT SAY. ***  A LINEAR, PLANAR, constant-closing-velocity
%   homing loop, perfect estimator, no acceleration limit, no drag, no gravity,
%   no LOS floor.  It prices GAINS, not misses (that is Listing 26.6).  Read it
%   as the generator for a stored gain table -- the book's own use, at
%   zarchan.txt:9769 -- and not as evidence about our terminal dive, whose
%   dominant error is a gamma-tracking lag that SATURATES.
%
% ---------------------------------------------------------------------------
%   THE TRANSCRIPTION.  listing_26_04.m does not parse.  Two repairs, neither
%   a judgement call:
%     1  "while ~(T .= (TF-.0001))" -> ">=".  "." is this OCR's ">" (the
%        extractor already made that same substitution, once, on "if S1 .=
%        .009999" in this very listing).  Shape-forced: T starts at 0 and only
%        increases, so "<=" makes ~(T<=...) false on entry and the loop body
%        never runs at all.
%     2  Lines 21-26 of the transcription (the "ALTERNATIVE APPROACH TO CUBIC"
%        heading, its transfer function and the stray ligature "1 s2 / v2 z")
%        are the next SECTION leaking in: the listing ends at zarchan.txt:9785
%        and that heading is :9786.  Dropped, not repaired into code.
%
%   The book's own quirks, and what was done with each:
%     * GT = G' and GAMINV = 1/GAM are recomputed in the inner loop and are
%       loop-invariant.  HOISTED; this changes no value.
%     * the trailing "FLAG=0" cannot be read (the outer loop re-initialises
%       it) and "save datfil.txt output /ascii" is a typo for -ascii and writes
%       nothing here.  Both recorded and dropped.
%     * THE RECORDED GAINS COME FROM THE PREDICTOR: C is evaluated on the
%       second pass, at the Euler-predicted S, and only then is S corrected and
%       (T, C) written down.  Kept.  It is why the structural ratio check is
%       scored at 1e-3 and not 1e-12 -- the predictor is rank-one only to
%       O((H*|F|)^2).
%     * "if S1 .= .009999" is kept as "S1 >= samp*0.9999", which IS 0.009999 at
%       the default samp = 0.01 s and does not silently change the record
%       interval for a caller who changes H.
%
% ---------------------------------------------------------------------------
%   THE SOLUTION IS RANK ONE, AND EVERY SELF-CHECK USES THAT.  With SF = e1*e1'
%   and no running state cost, S(tgo) = p*v*v' exactly, where vdot = F'*v,
%   v(0) = e1 (v is the ZERO-EFFORT-MISS row) and 1/p = 1 + int (G'v)^2/gamma.
%   Two consequences, both DERIVED HERE and therefore structural, not book:
%     * v1 = 1, v2 = tgo, v3 = tgo^2/2 exactly, so C1*tgo^2 == C2*tgo == 2*C3
%       == NP identically -- the relations the book prints for the single-lag
%       law at zarchan.txt:9347-9351.
%     * C6 = -(w^2/tau)*p*v6^2/gamma <= 0 for every tgo.
%
% ---------------------------------------------------------------------------
%   BENCHMARKS.  Ch.26 publishes almost nothing numeric for this listing: its
%   whole output is Fig. 26.14, and the extraction turned every figure into an
%   image link.  So the scored rows are (a) the one number the prose states
%   plainly, 3; (b) the prose CLAIMS about that figure, as ordering checks; and
%   (c) one real anchor -- the SAME sweep run on Listing 26.1's single-lag
%   matrices, where the book prints closed forms for all four gains; and (d)
%   that anchor repeated at twice the step, so the board SHOWS that what is
%   left in (c) scales as H^2 instead of asserting it.  Line numbers are in
%   each row's .source.  Nothing was read off a figure.
%
%   WHAT THIS PRINTS.  *** MEASURED, 2026-08-24, R2025b, nominal case. ***
%   The derived predictions this header used to carry all landed:
%     NP NEGATIVE near intercept, minimum -9.2058 at tgo = 0.160 s (pred -9.4)
%     NP peak +42.8621 at tgo = 0.520 s (pred +43);  NP(10 s) = 3.7511 (pred
%     3.75), against the single-lag formula's 3.6911;  NP(0.01 s) = -0.007249,
%     0.017% of peak.  C4,C5,C6 at TF = -0.36262, -0.039382, -0.00083738.
%     sv2/sv1 of the final S = 7.014e-5 (the rank-one leakage; see the INFO row).
%   The listing's own "axis([0 10 -10 50])" brackets exactly that curve --
%   corroboration, not a measurement, and reported as an INFO row.  A run far
%   from these numbers is wrong even if every row is green.
%
%   COST, MEASURED: 0.375-0.41 s over four runs -- TF/H = 1e5 Heun steps on
%   the 6x6 (200,000 right-hand sides), plus the cross leg at the book's own
%   1e-4 (200,000 on the 4x4) and the H-scaling leg at 2e-4 (100,000) beside
%   it.  0.225 s was the old figure, with the cross leg at the coarse 5e-4 and
%   no H-scaling leg.  The 6x6 step is the book's ("after some
%   experimentation an integration step size of 0.0001 s was chosen",
%   zarchan.txt:9356) and it is not gratuitous: the quadratic term carries
%   (w^2/tau)^2/gamma = 1.6e10.  Raise H to go faster and expect the
%   near-intercept dip to move first.
%
%   OUTPUTS
%     R.t          time to go [s], the listing's ArrayT
%     R.C          n-by-6, [C1 C2 C3 C4 C5 C6] -- the listing's -C(1,:)
%     R.NP         C2.*t, the listing's ArrayNP (effective navigation ratio)
%     R.NP_singlelag  the book's single-lag N' formula on the same grid, which
%                  is the other curve in Fig. 26.14 (zarchan.txt:9341)
%     R.S          the final 6x6 Riccati matrix
%     R.cross      the Listing 26.1 cross-check: .t .C .Cth .win .maxrel
%                  .t_worst .h .n_rhs, plus the H-scaling leg .h2 .maxrel_2h
%                  .t_worst_2h .ratio .n_rhs_2h and .scale_why (non-empty when
%                  the ratio exists but must NOT be scored) -- or [] when
%                  'crosscheck' is false
%     R.bench R.selfcheck R.pass R.verdict R.summary R.p R.units
%
%   See also MGS.ZARCHAN.BENCHSUITE.

arguments
    % Listing 26.4 line 1, verbatim (zarchan.txt:9772, :9777).
    opts.TAU (1,1) double {mustBeFinite, mustBePositive} = 1.0     % s
    opts.WZ  (1,1) double {mustBeFinite, mustBePositive} = 5.0     % rad/s
    opts.W   (1,1) double {mustBeFinite, mustBePositive} = 20.0    % rad/s
    opts.Z   (1,1) double {mustBeFinite, mustBePositive} = 0.7     % -
    opts.TF  (1,1) double {mustBeFinite, mustBePositive} = 10.0    % s
    opts.GAM (1,1) double {mustBeFinite, mustBePositive} = 1e-5    % -
    opts.H   (1,1) double {mustBeFinite, mustBePositive} = 1e-4    % s
    % The listing's 0.009999 threshold, expressed so it survives a change of H.
    opts.samp (1,1) double {mustBeFinite, mustBePositive} = 0.01   % s
    % The Listing 26.1 leg, at LISTING 26.1's OWN step (zarchan.txt:9359 codes
    % "H=.0001"; the prose at :9356 says the author chose it after
    % experimentation).  It was 5e-4 here, justified by "the fastest rate
    % anywhere in that sweep is d(ln p)/dt = about 15/s near tgo = 0.3 s, so
    % H*rate stays under 0.01".  *** THAT ARGUMENT IS THE WRONG ERROR MODEL AND
    % MEASUREMENT KILLED IT. ***  H*rate = 0.0075 is indeed small, but the
    % homogeneous part of this Riccati is Sdot = S*F + F'*S with F NILPOTENT
    % except for the -1/TAU mode, so its transition matrix grows POLYNOMIALLY:
    % an error committed in the tgo ~ 0.2 s transient is then amplified all the
    % way out to tgo = TF.  Measured relative error in C1 against the exact
    % rank-one solution, at H = 1e-4: 2.4e-7 at tgo = 0.2 s, 7.1e-6 at 1 s,
    % 4.4e-4 at 5 s, 1.10e-3 at 10 s -- growth like tgo^2.19 over the scored
    % window.  So the worst error sits at the FAR end, not in the transient
    % that set the step.  Worst |dC/C| over tgo >= 1 s -- THE QUANTITY THIS ROW
    % ACTUALLY SCORES, i.e. against the book's printed formula -- measured:
    %     H     2e-3     1e-3     5e-4    2.5e-4    1e-4     5e-5     2e-5
    %   err  3.33e-1  1.02e-1  2.70e-2  6.86e-3  1.10e-3  2.75e-4  2.23e-4
    % which is clean O(H^2) down to 5e-5: the factor-2 ratios err(2h)/err(h) are
    % 3.263 (h=1e-3), 3.774 (5e-4), 3.940 (2.5e-4) and 4.002 (5e-5), and the
    % 2.5e-4 -> 1e-4 step is 6.223 against the 2.5^2 = 6.25 it should be.
    % *** CORRECTED 2026-08-24 by audit re-measurement. ***  The last column
    % used to read 4.41e-5, which is this sweep scored against the EXACT
    % rank-one analytic solution -- a DIFFERENT reference curve from the one
    % the row uses.  Against the book's formula the 2e-5 run measures 2.23e-4,
    % because the constant guard-vs-GAM floor described below has taken over;
    % the worst residual has moved from tgo = 9.99 s to tgo = 1.00 s and the
    % Richardson ratio collapses to 1.234, which is exactly why the H-scaling
    % row SKIPS there instead of failing.
    % At the old 5e-4 that is 2.70% against a 1% band -- the row was
    % red because of THIS DEFAULT, not because of the book and not because of
    % the cross-check's construction: the degenerate-F path was verified
    % BIT-FOR-BIT identical to mgs.zarchan.optimalGainsSingleLag at the same H
    % (max |dC/C| between the two = 0.000e+00 over all 1000 samples).
    opts.crosscheck (1,1) logical = true
    opts.cross_h (1,1) double {mustBeFinite, mustBePositive} = 1e-4
    % THE ONLY SWITCHES.  Not nargin: in a function whose arguments block
    % declares only name-value options MATLAB reports nargin = 0 however many
    % were passed, so "if nargin == 0" would be a mode that never fires.
    opts.benchmark (1,1) logical = true
    opts.render    (1,1) logical = true
    opts.plot      (1,1) logical = false
    opts.assert    (1,1) logical = true
end

TAU = opts.TAU;  WZ = opts.WZ;  W = opts.W;  Z = opts.Z;
TF  = opts.TF;   GAM = opts.GAM;  H = opts.H;  samp = opts.samp;

if samp < H
    error('mgs:zarchan:optimalGainsCubic:samp', ...
        ['samp = %g s is smaller than the step H = %g s, so the record would ' ...
         'be written at the step rate while claiming the coarser interval.'], ...
        samp, H);
end
if TF <= 2*H
    error('mgs:zarchan:optimalGainsCubic:short', ...
        'TF = %g s is not long enough to take two steps at H = %g s.', TF, H);
end
if H*W > 0.05
    % Refused, not warned about.  RK2 on this Riccati does not drift -- the
    % quadratic term gives it a finite escape time and it returns Inf/NaN, or,
    % worse, a smooth curve of the wrong shape when it escapes late.  The
    % book's own H*W is 2e-3; this bar is 25 times looser than that.
    error('mgs:zarchan:optimalGainsCubic:stiff', ...
        ['H*W = %g.  The flight-control-system quadratic sits at W = %g rad/s ' ...
         'and the Riccati quadratic term carries (W^2/TAU)^2/GAM = %.3g; at ' ...
         'this step the sweep is not integrating that mode, it is guessing.'], ...
        H*W, W, (W*W/TAU)^2/GAM);
end

% =========================================================================
%   LISTING 26.4.  Transcribed from zarchan.txt:9772-9785.
% =========================================================================
[F, G] = cubicMatrices(TAU, WZ, W, Z);
[t, C, s11, S, nEval] = riccatiSweep(F, G, GAM, TF, H, samp);
if numel(t) < 3
    error('mgs:zarchan:optimalGainsCubic:empty', ...
        ['the sweep recorded %d sample(s) at TF = %g s, samp = %g s.  Three ' ...
         'is the minimum at which "interior peak" is a statement about the ' ...
         'curve rather than about the sampling.'], numel(t), TF, samp);
end
if ~all(isfinite(C(:))) || ~all(isfinite(S(:)))
    % The Riccati quadratic term gives this a FINITE ESCAPE TIME, so an
    % oversized step does not drift -- it leaves, and everything downstream
    % (svd, the shape rows) would then fail for a reason that names the wrong
    % thing.  Say it here instead.
    error('mgs:zarchan:optimalGainsCubic:escape', ...
        ['the sweep left the finite numbers at H = %g s, TAU = %g, WZ = %g, ' ...
         'W = %g, Z = %g, GAM = %g.  The quadratic term carries ' ...
         '(W^2/TAU)^2/GAM = %.3g; reduce H or raise GAM.'], ...
        H, TAU, WZ, W, Z, GAM, (W*W/TAU)^2/GAM);
end
NP = C(:,2).*t;                       % the listing's NP = C2*T

% The OTHER curve in Fig. 26.14: the single-lag optimal law's N', from the
% book's own formula (zarchan.txt:9341, coded at :9366).
[~, NPsl] = singleLagGains(t, TAU);

% ---- the Listing 26.1 leg: same sweep, single-lag matrices, published gains
CR = [];
if opts.crosscheck
    [F4, G4] = singleLagMatrices(TAU);
    [tc, Cc, ~, ~, nEvalC] = riccatiSweep(F4, G4, GAM, TF, opts.cross_h, samp);
    CR = struct();
    CR.t   = tc;
    CR.C   = Cc;
    CR.Cth = singleLagGains(tc, TAU);
    % Scored from tgo = 1 s outward ONLY.  Inside that the two curves are
    % answering different questions, and the gap is CLOSED FORM.  Both are
    % N'(x) = x^2(e^-x+x-1)/(Q(x)+d) with Q = int_0^x (e^-s+s-1)^2 ds, and 6Q
    % is exactly the denominator printed at zarchan.txt:9341.  Only d differs:
    % the sweep converges to d = GAM/TAU^3 = 1e-5, the printed formula carries
    % the author's own +0.0001 on 6Q, i.e. d = 1.667e-5.  MEASURED at tgo = 1 s
    % (H = 2e-5, where the numerical term is 4.4e-5 and cannot confuse this):
    % the sweep is -3.3416e-4 below the unguarded law, the printed formula is
    % -5.5698e-4 below it, and the residual between the two is +2.2294e-4.
    % *** The old note here said "they pull opposite ways".  They do not --
    % both pull the same way (both d > 0 shrink N'), and the residual is their
    % DIFFERENCE, (1e-4/6 - GAM/TAU^3)/(Q(x) + GAM/TAU^3) = 2.2284e-4 predicted
    % against 2.2294e-4 measured.  The conclusion the old note drew (residual
    % smaller than either pull) survives; its reason did not. ***  This is a
    % FLOOR, not a step-size artefact: it does not shrink with H at all.  It
    % falls as 1/Q ~ 3/x^3, reaching 2.7e-8 by tgo = 10 s, so it is invisible
    % beside the numerical error at any H this port would use.
    CR.win = tc >= 1.0;
    if any(CR.win)
        [CR.maxrel, CR.imax, CR.jmax] = worstRel(Cc(CR.win,:), CR.Cth(CR.win,:));
        tw = tc(CR.win);
        CR.t_worst = tw(min(CR.imax, numel(tw)));
        CR.t_lo = tw(1);  CR.t_hi = tw(end);
    else
        % TF < 1 s.  Nothing to score: inside tgo = 1 s the sweep and the
        % printed formula are not answering the same question (see above).  A
        % NaN here becomes a SKIP row, not a silent pass and not a false fail.
        CR.maxrel = NaN;  CR.imax = 1;  CR.jmax = 1;  CR.t_worst = NaN;
        CR.t_lo = NaN;  CR.t_hi = NaN;
    end
    CR.h = opts.cross_h;
    CR.n_rhs = nEvalC;

    % ---- THE H-SCALING LEG.  The SAME sweep at twice the step, so the board
    %      can SHOW that what is left in the row above is numerical rather than
    %      a disagreement with the book.  err(2h)/err(h) is 4 for O(H^2), 2 for
    %      O(H), and 1 for anything structural.  This is Richardson against the
    %      book's own printed formula -- it needs no derived reference curve,
    %      which is the point: the reference cancels out of the ratio.
    CR.h2 = 2*opts.cross_h;
    CR.maxrel_2h = NaN;  CR.t_worst_2h = NaN;  CR.ratio = NaN;
    CR.n_rhs_2h = 0;     CR.scale_why = '';
    if ~isfinite(CR.maxrel)
        CR.scale_why = 'the fine leg itself had nothing to score';
    elseif CR.h2 > samp
        CR.scale_why = sprintf(['2*cross_h = %g s exceeds samp = %g s, so the ' ...
            'coarse leg would record at its own step rate'], CR.h2, samp);
    elseif TF <= 2*CR.h2
        CR.scale_why = sprintf('TF = %g s is under two steps at 2*cross_h = %g s', ...
            TF, CR.h2);
    else
        [tc2, Cc2, ~, ~, nEvalC2] = riccatiSweep(F4, G4, GAM, TF, CR.h2, samp);
        w2 = tc2 >= 1.0;
        CR.n_rhs_2h = nEvalC2;
        if ~any(w2) || ~all(isfinite(Cc2(:)))
            CR.scale_why = 'the coarse leg recorded nothing scoreable, or left the finite numbers';
        else
            [CR.maxrel_2h, i2] = worstRel(Cc2(w2,:), singleLagGains(tc2(w2), TAU));
            tw2 = tc2(w2);
            CR.t_worst_2h = tw2(min(i2, numel(tw2)));
            CR.ratio = CR.maxrel_2h / max(CR.maxrel, realmin);
            % THE ONE WAY THIS RATIO LIES.  Once cross_h is fine enough that the
            % numerical term drops under the 2.23e-4 floor described above, both
            % legs measure the FLOOR and the ratio collapses to 1 -- which would
            % read as "structural" and fail the row for being too accurate.  The
            % measured signature of that is the worst location JUMPING from the
            % far end of the window to tgo = 1 s (observed at H = 2e-5: 9.99 s
            % -> 1.00 s).  So the row is scored only while the fine leg's worst
            % still sits in the outer half of the window, and SKIPS otherwise.
            if CR.t_worst < 0.5*(CR.t_lo + CR.t_hi)
                CR.scale_why = sprintf(['at cross_h = %g s the worst residual has ' ...
                    'moved to tgo = %.2f s, the low end of the scored window: the ' ...
                    'numerical term is already under the constant guard-vs-GAM ' ...
                    'floor, so both legs measure the floor and the ratio (%.3f) ' ...
                    'is not a statement about H'], CR.h, CR.t_worst, CR.ratio);
            end
        end
    end
else
    nEvalC = 0;
end

R = struct();
R.t  = t;
R.C  = C;
R.NP = NP;
R.NP_singlelag = NPsl;
R.S  = S;
R.s11 = s11;
R.cross = CR;
R.units = ['seconds and rad/s; gains for nc = C1*y + C2*ydot + C3*nT + ' ...
           'C4*e + C5*edot + C6*eddot -- C1 [1/s^2] C2 [1/s] C3 [-] C4 [-] ' ...
           'C5 [s] C6 [s^2], NP dimensionless.  No length unit appears.'];
if isempty(CR), nEvalC2 = 0; else, nEvalC2 = CR.n_rhs_2h; end
R.p = struct('TAU',TAU, 'WZ',WZ, 'W',W, 'Z',Z, 'TF',TF, 'GAM',GAM, 'H',H, ...
             'samp',samp, 'cross_h',opts.cross_h, 'n_rhs',nEval, ...
             'n_rhs_cross',nEvalC, 'n_rhs_cross_2h',nEvalC2, 'units',R.units);

R.selfcheck = structuralChecks(t, C, NP, S, s11, TF);

bookCase = abs(TAU-1) < 1e-12 && abs(WZ-5) < 1e-12 && abs(W-20) < 1e-12 && ...
           abs(Z-0.7) < 1e-12 && abs(GAM-1e-5) < 1e-15 && TF >= 10;

if opts.benchmark
    R.bench = benchmarks(R, TAU, TF, bookCase);
    ev      = ~[R.bench.skipped];
    R.pass  = any(ev) && all([R.bench(ev).ok]) && R.selfcheck.all_ok;
else
    R.bench = emptyBench();
    ev      = false;
    % NOT true.  "Not checked" must never render as "fine".
    R.pass  = false;
end

R.summary = render(R, bookCase, ev);
if opts.render, fprintf('%s\n', R.summary); end
if opts.plot
    % The listing's own plot block, axis included (zarchan.txt:9785).
    figure; plot(R.t, R.NP, R.t, R.NP_singlelag); grid on
    xlabel('Time (s) '); ylabel('NP'); axis([0 TF -10 50]);
end

% ---- adjudication.  The verdict is FORMED either way; 'assert' chooses only
%      the reaction to it.  benchSuite runs every port with assert off so that
%      one red row cannot withhold the evidence for the rest of the board.
msg = '';
if ~opts.benchmark
    msg = ['''benchmark'' is false, so nothing was compared against the book. ' ...
           'R.pass is false meaning NOT CHECKED.'];
elseif ~any(ev)
    msg = sprintf(['not one book value could be evaluated at TAU=%g, WZ=%g, ' ...
        'W=%g, Z=%g, GAM=%g, TF=%g.  An unvalidated port must not be handed ' ...
        'back silently.'], TAU, WZ, W, Z, GAM, TF);
elseif ~R.selfcheck.all_ok
    msg = sprintf(['%d structural invariant(s) violated -- the port disagrees ' ...
        'with ITSELF, which is a wiring fault, not a tolerance question.  ' ...
        'First: %s.'], sum(~R.selfcheck.oks), R.selfcheck.first_bad);
elseif ~R.pass
    bad = find(~[R.bench.ok] & ev);
    msg = sprintf(['%d of %d evaluated book values are outside tolerance ' ...
        '(first: %s, book %s, ours %s, band %s).'], numel(bad), sum(ev), ...
        R.bench(bad(1)).what, R.bench(bad(1)).book_str, ...
        R.bench(bad(1)).ours_str, R.bench(bad(1)).band);
end
R.verdict = msg;

if ~isempty(msg)
    if opts.assert
        error('mgs:zarchan:optimalGainsCubic:benchmark', '%s\n\n%s', msg, R.summary);
    else
        warning('mgs:zarchan:optimalGainsCubic:benchmark', ...
            '%s  (''assert'' is false, so the result is returned anyway; see R.verdict)', msg);
    end
end
end


% =============================================================================
function [F, G] = cubicMatrices(TAU, WZ, W, Z)
%CUBICMATRICES  zarchan.txt:9777, verbatim.  States [y ydot nT e edot eddot],
%   with nL = e - eddot/wz^2 folded into row 2 (that is what F(2,6) = 1/wz^2
%   is).  Row 6 is the cubic denominator (1+s*tau)(1+2*Z*s/W+s^2/W^2) written
%   out; G puts nc on eddot-dot with the same W^2/TAU.
F = zeros(6,6);
G = zeros(6,1);
G(6,1) = W*W/TAU;
F(1,2) = 1;
F(2,3) = 1;
F(2,4) = -1;
F(2,6) = 1/WZ^2;
F(4,5) = 1;
F(5,6) = 1;
F(6,4) = -W*W/TAU;
F(6,5) = -W*W*(2*Z/W + TAU)/TAU;
F(6,6) = -W*W*(1/W^2 + 2*Z*TAU/W)/TAU;
end


% =============================================================================
function [F, G] = singleLagMatrices(TAU)
%SINGLELAGMATRICES  Listing 26.1's matrices, zarchan.txt:9359.  Used ONLY to
%   drive the cross-check: the book prints closed forms for all four of that
%   problem's gains, and it is the same sweep code that computes them.
F = zeros(4,4);
G = zeros(4,1);
G(4,1) = 1/TAU;
F(1,2) = 1;
F(2,3) = 1;
F(2,4) = -1;
F(4,4) = -1/TAU;
end


% =============================================================================
function [t, Cg, s11, S, nEval] = riccatiSweep(F, G, GAM, TF, H, samp)
%RICCATISWEEP  Listing 26.4's loops, zarchan.txt:9777-9784.
%
%   EVERY constant arrives as an argument.  MATLAB local functions cannot see
%   the caller's workspace; a previous wave in this package read one from the
%   main function and threw on the first line of the first run.
%
%   T is TIME TO GO.  S(1,1) = 1 at T = 0 is the terminal cost y(tF)^2, so the
%   sweep runs FORWARD in tgo and BACKWARD in flight time.
n = size(F,1);
S = zeros(n,n);
S(1,1) = 1;
T  = 0;
S1 = 0;
GT     = G.';        % hoisted out of the listing's inner loop: loop-invariant
GAMINV = 1/GAM;      % likewise

nmax = ceil(TF/samp) + 8;
t   = zeros(nmax,1);
Cg  = zeros(nmax,n);
s11 = zeros(nmax,1);
k   = 0;
% Initialised before the loop so the FLAG==1 branch cannot read a leftover.
SD = zeros(n,n);
C  = zeros(1,n);
nEval = 0;

% "while ~(T .= (TF-.0001))" -> ">=".  See the transcription note in the header.
% The .0001 is the listing's own literal, not H, and is left as it is.
while ~(T >= (TF - 0.0001))
    S1 = S1 + H;
    SOLD = S;
    STEP = 1;  FLAG = 0;

    % Zarchan's two-pass predictor loop.  Pass 1 evaluates at the current S;
    % pass 2 takes a full Euler step, advances T by H, and re-evaluates there.
    % T advances INSIDE this loop, exactly once -- move that line out and the
    % scheme degrades to first order while still running perfectly smoothly.
    while STEP <= 1
        if FLAG == 1
            STEP = 2;
            S = S + H*SD;
            T = T + H;
        end
        C  = GAMINV*(GT*S);              % C = GAMINV*GT*S
        SF = S*F;
        SD = SF + SF.' - GAM*(C.'*C);    % SD = SF + SFT - GAM*CT*C
        FLAG = 1;
        nEval = nEval + 1;
    end
    % The listing's "FLAG=0" here cannot be read (the outer loop re-initialises
    % it).  Left as this comment so nobody re-adds it thinking state was lost.

    % Heun corrector, in the listing's own algebra:
    %   H2=.5*H; HSDP=H2*SD; SS=SOLD+S; SSP=.5*SS; S=SSP+HSDP;
    S = 0.5*(SOLD + S) + 0.5*H*SD;

    if S1 >= samp*0.9999                 % the listing's "if S1 .= .009999"
        S1 = 0;
        k = k + 1;
        t(k)    = T;
        Cg(k,:) = -C;                    % C1=-C(1,1), C2=-C(1,2), ...
        s11(k)  = S(1,1);
    end
end

t = t(1:k);  Cg = Cg(1:k,:);  s11 = s11(1:k);
end


% =============================================================================
function [TH, XNPP] = singleLagGains(tgo, TAU)
%SINGLELAGGAINS  The book's closed-form single-lag gains, zarchan.txt:9366
%   (printed as formulas at :9341 and :9347-9354).  Column order C1 C2 C3 C4.
%
%   The +.0001 in the denominator is the author's, and it is load-bearing:
%   the denominator is 0.3*x^5 for small x, formed as a difference of terms of
%   order 3, so it would otherwise lose every significant digit AND divide by
%   zero at x = 0.  It also happens to imitate the finite-gamma roll-off that
%   the numerical sweep really has, which is why Fig. 26.6 agrees at all.
tgo  = tgo(:);
XS   = tgo/TAU;
TOP  = 6.*XS.*XS.*(exp(-XS) - 1. + XS);
BOT1 = 2*XS.*XS.*XS + 3. + 6.*XS - 6.*XS.*XS;
BOT2 = -12.*XS.*exp(-XS) - 3.*exp(-2.*XS);
XNPP = TOP./(BOT1 + BOT2 + .0001);
TH = [XNPP./(tgo.*tgo), XNPP./tgo, .5*XNPP, ...
      -XNPP.*(exp(-XS) + XS - 1.)./(XS.*XS)];
end


% =============================================================================
function [w, i, j] = worstRel(A, B)
%WORSTREL  Worst relative deviation of A from B, and where.  Elements where B
%   is zero are skipped rather than made infinite -- an Inf here would fail the
%   row for a reason that has nothing to do with the book.
D = abs(A - B)./max(abs(B), realmin);
D(B == 0) = 0;
[colmax, rowidx] = max(D, [], 1);
[w, j] = max(colmax);
i = rowidx(j);
end


% =============================================================================
function K = structuralChecks(t, C, NP, S, s11, TF)
%STRUCTURALCHECKS  Self-consistency.  NOT book values, and they must never be
%   quoted as evidence that this port reproduces Zarchan.  They are the weaker
%   claim that it agrees with ITSELF, which is what catches wiring faults.
K = struct();
names = {};  oks = [];  detail = {};

% 1  S stays symmetric.  SF+SF' and C'*C are both symmetric by construction, so
%    any asymmetry is either roundoff or a transposed term.
asym = max(abs(S - S.'), [], 'all')/max(max(abs(S), [], 'all'), realmin);
names{end+1} = 'S symmetric';
oks(end+1)   = asym <= 1e-12;
detail{end+1} = sprintf('max |S-S''| / max|S| = %.3e', asym);

% 2  S(1,1) is non-increasing and never exceeds 1.  Exact: column 1 of F is
%    zero, so d/dtgo S(1,1) = -(1/gamma)*(G'S)(1)^2 <= 0.
rise = max(diff(s11));
names{end+1} = 'S(1,1) non-increasing from 1';
oks(end+1)   = rise <= 1e-14 && s11(1) <= 1 + 1e-12 && s11(end) > 0;
detail{end+1} = sprintf('S11: %.6g -> %.6g, worst rise %+.3e', ...
                        s11(1), s11(end), rise);

% 3  C1*tgo^2 == C2*tgo == 2*C3 == NP.  Follows from the rank-one reduction in
%    the header (v1=1, v2=tgo, v3=tgo^2/2) and is what the book prints for the
%    single-lag law at zarchan.txt:9347-9351.  Scored from tgo = TF/2 outward,
%    where NP is order 1: near intercept all four quantities go to zero
%    together and a RELATIVE comparison there measures nothing.
%
%    1e-3, not 1e-12, and the reason is structural rather than generous: the
%    recorded C comes from the EULER PREDICTOR, which is rank-one only to
%    O((H*|F|)^2).  A wiring fault moves these by O(1).
%    *** NOW MEASURED, and the margin is thin: 6.568e-4 against the 1e-3 band
%    at the default H = 1e-4, i.e. 1.5x, and it is the SAME O(H^2) growth the
%    cross-check leg measures, NOT the predictor (1.642e-4 at H = 5e-5,
%    6.568e-4 at 1e-4, 2.617e-3 at 2e-4 -- ratio 4; the Euler term at
%    tgo = 10 s is order 1e-8).  Measured break: this check is green at
%    H = 1.20e-4 (9.452e-4) and red at H = 1.25e-4 (1.026e-3), which is well
%    inside what the H*W <= 0.05 guard above still allows.
%    The band is left where it is: it is the tolerance the structure deserves,
%    and a caller who coarsens H should be told, not accommodated. ***
win = t >= 0.5*TF;
den = max(abs(NP(win)), realmin);
r1  = max(abs(C(win,1).*t(win).^2 - NP(win))./den);
r3  = max(abs(2*C(win,3)          - NP(win))./den);
names{end+1} = 'C1*tgo^2 == C2*tgo == 2*C3 (ZEM structure)';
oks(end+1)   = max(r1, r3) <= 1e-3;
detail{end+1} = sprintf('worst rel. dev. %.3e (C1 leg %.3e, C3 leg %.3e)', ...
                        max(r1,r3), r1, r3);

% 4  C6 <= 0 everywhere.  C6 = -(w^2/tau)*p*v6^2/gamma with p > 0, so the sign
%    cannot depend on tgo.  This is the cheapest thing that would notice a
%    sign flip on G or a transposed C.
c6max = max(C(:,6));
scale = max(max(abs(C(:,6))), realmin);
names{end+1} = 'C6 <= 0 for all tgo';
oks(end+1)   = c6max <= 1e-12*scale;
detail{end+1} = sprintf('max C6 = %+.4e (peak |C6| = %.4e)', c6max, scale);

K.names = names;
K.oks   = logical(oks);
K.detail = detail;
K.all_ok = all(K.oks);
if K.all_ok
    K.first_bad = '';
else
    ib = find(~K.oks, 1);
    K.first_bad = sprintf('%s -- %s', names{ib}, detail{ib});
end
end


% =============================================================================
function B = benchmarks(R, TAU, TF, bookCase)
%BENCHMARKS  Computed vs published, with the PROVENANCE of every band.
%
%   *** EVERY BOOK VALUE HERE WAS GREPPED OUT OF zarchan.txt AND THE LINE
%   NUMBER IS IN THE .source FIELD. ***  Ch.26's whole output for this listing
%   is Fig. 26.14 and the extraction turned every figure into an image link, so
%   the figure ROWS are ordering checks on prose about the figure -- which is
%   greppable, and stronger than a value eyeballed off a plot.
B = emptyBench();
t = R.t;  NP = R.NP;  NPsl = R.NP_singlelag;
whyCase = sprintf(['off the listing''s nominal case -- Fig. 26.14 is TAU=1 s, ' ...
    'WZ=5 r/s, W=20 r/s, Z=0.7, GAM=1e-5, TF=10 s; this run used TAU=%g, ' ...
    'WZ=%g, W=%g, Z=%g, GAM=%g, TF=%g'], R.p.TAU, R.p.WZ, R.p.W, R.p.Z, ...
    R.p.GAM, R.p.TF);

% ---- 1.  the one number the prose states plainly --------------------------
% "both effective navigation ratios approach 3 when the time to go before
% intercept is very large" (zarchan.txt:9766).  Scored against the book's own
% N' formula (:9341) at x = 1e4, where its +0.0001 regulariser and the
% cancellation in its denominator are both irrelevant.  This is a check of the
% FORMULA, i.e. of the other curve in Fig. 26.14, not of the Riccati sweep.
[~, npInf] = singleLagGains(1e4*TAU, TAU);
B = addVal(B, 'single-lag N'' as tgo -> inf vs book "approach 3"', 3, npInf, ...
    'text', 0.01, 'p.674 text (zarchan.txt:9766); formula (zarchan.txt:9341)', ...
    '-', ['evaluated at x = 1e4.  The limit of the printed formula is exactly ' ...
          '3 (6x^3 over 2x^3); 1% is the band for a one-figure prose value']);

% ---- 2.  THE ONE REAL NUMERIC ANCHOR -------------------------------------
% The same sweep, run on Listing 26.1's single-lag matrices, against the four
% closed forms the book prints for that problem.  The book's own claim about
% this comparison is "near perfect agreement" (zarchan.txt:9373), an adjective;
% 1% is the band, and the two known biases are an order of magnitude below it.
if ~isempty(R.cross) && isfinite(R.cross.maxrel)
    CRs = R.cross;
    gname = {'C1','C2','C3','C4'};
    B = addBound(B, 'Riccati sweep vs Ch.26 closed-form gains (single-lag F)', ...
        'worst |dC/C| <= 1% over tgo in [1, TF]', ...
        sprintf('%.4f%% (%s at tgo = %.2f s, H = %g s)', 100*CRs.maxrel, ...
                gname{min(CRs.jmax,4)}, CRs.t_worst, CRs.h), ...
        CRs.maxrel <= 0.01, ...
        'p.657 text (zarchan.txt:9373); formulas (zarchan.txt:9341, :9347-9354)', ...
        '-', ['the only published NUMBERS this listing''s machinery can be ' ...
              'laid against.  MEASURED 0.110% at the listing''s own H = 1e-4, ' ...
              'worst at the far end of the window (tgo = TF), and it is O(H^2) ' ...
              '-- see the H-scaling row below, which is here so that claim is ' ...
              'shown rather than asserted.  Under it sits a CONSTANT 0.0223% ' ...
              'floor at tgo = 1 s: the sweep''s finite GAM pulls N'' down ' ...
              '0.0334%, the formula''s own +0.0001 regulariser pulls it down ' ...
              '0.0557%, both the SAME way, and the residual is the difference']);
else
    B = addSkip(B, 'Riccati sweep vs Ch.26 closed-form gains (single-lag F)', ...
        'near perfect agreement', ...
        ['''crosscheck'' is false, or TF < 1 s so nothing lies in the ' ...
         'comparable window.  This is the only row that puts a NUMBER from ' ...
         'this integrator against a printed one'], ...
        'p.657 text (zarchan.txt:9373)', '-');
end

% ---- 2b.  IS WHAT IS LEFT IN ROW 2 NUMERICAL, OR A REAL DISAGREEMENT? ------
% Richardson on the row above: the SAME comparison at twice the step.  4 is
% O(H^2) (this scheme's global order), 2 would be O(H), 1 would mean the
% residual does not depend on H at all and is therefore structural.  The
% printed formula cancels out of the ratio, so this needs no reference curve
% and cannot be tuned by choosing one.  MEASURED at cross_h = 1e-4: 3.990.
scaleSrc = ['derived here (Richardson); step from the listing itself, ' ...
            'zarchan.txt:9359 "H=.0001" with the prose at :9356'];
if ~isempty(R.cross) && isfinite(R.cross.ratio) && isempty(R.cross.scale_why)
    CRs = R.cross;
    B = addBound(B, 'that residual is NUMERICAL: err(2h)/err(h) is O(H^2)', ...
        'ratio in [3, 5]  (4 = O(H^2), 2 = O(H), 1 = structural)', ...
        sprintf('%.3f  (%.4f%% at h = %g s -> %.4f%% at 2h = %g s)', ...
                CRs.ratio, 100*CRs.maxrel, CRs.h, 100*CRs.maxrel_2h, CRs.h2), ...
        CRs.ratio >= 3 && CRs.ratio <= 5, scaleSrc, '-', ...
        ['THE POINT OF THIS ROW.  Row 2 was red at the old cross_h = 5e-4 ' ...
         '(2.70% against a 1% band) and the fix was to use the listing''s own ' ...
         'step, which is exactly the move that has to be JUSTIFIED rather than ' ...
         'asserted.  Measured worst |dC/C| over tgo >= 1 s: 3.33e-1 at H=2e-3, ' ...
         '1.02e-1 at 1e-3, 2.70e-2 at 5e-4, 6.86e-3 at 2.5e-4, 1.10e-3 at ' ...
         '1e-4, 2.75e-4 at 5e-5 -- eight measured points, ratio 4 throughout.  ' ...
         'The error is committed in the tgo ~ 0.2 s transient and then grown ' ...
         'like tgo^2.19 by the polynomial homogeneous flow, which is why the ' ...
         'worst sits at tgo = TF and not where the step was chosen']);
elseif ~isempty(R.cross)
    why = R.cross.scale_why;
    if isempty(why), why = 'the coarse leg did not run'; end
    B = addSkip(B, 'that residual is NUMERICAL: err(2h)/err(h) is O(H^2)', ...
        'ratio in [3, 5]', why, scaleSrc, '-');
else
    B = addSkip(B, 'that residual is NUMERICAL: err(2h)/err(h) is O(H^2)', ...
        'ratio in [3, 5]', '''crosscheck'' is false, so there is no leg to scale', ...
        scaleSrc, '-');
end

% ---- 3-6.  the four prose claims about Fig. 26.14 -------------------------
src14 = 'p.674 text (zarchan.txt:9766); Fig. 26.14 caption (zarchan.txt:9773)';
if bookCase
    [npMin, iMin] = min(NP);
    [npMax, iMax] = max(NP);

    % 3  "the new effective navigation ratio also goes negative near intercept
    %    in an attempt to compensate for the right-half-plane zero"
    B = addShape(B, 'NP goes NEGATIVE near intercept (the RHP zero)', ...
        'min NP < 0, and it happens in the first 10% of the sweep', ...
        sprintf('min NP = %+.3f at tgo = %.3f s', npMin, t(iMin)), ...
        npMin < 0 && t(iMin) <= 0.1*TF, src14, '-', ...
        ['this is the row that separates the cubic law from the single-lag ' ...
         'law: drop F(2,6) = 1/WZ^2 and the whole negative lobe vanishes ' ...
         'while every other row here still passes']);

    % 4  "both navigation ratios get large near intercept"
    B = addShape(B, 'NP gets LARGE near intercept, then decays', ...
        'interior max in the first 20% of the sweep, at least 2x NP(TF)', ...
        sprintf('max NP = %.2f at tgo = %.3f s, vs NP(TF) = %.3f', ...
                npMax, t(iMax), NP(end)), ...
        iMax > 1 && iMax < numel(t) && t(iMax) <= 0.2*TF && ...
            npMax >= 2*abs(NP(end)), src14, '-', ...
        'derived expectation: about +43 near tgo = 0.52 s');

    % 5  "... and go to zero at intercept"
    endFrac = abs(NP(1))/max(abs(npMax), realmin);
    B = addShape(B, 'NP returns to ZERO at intercept', ...
        '|NP| at the first sample <= 1% of the peak', ...
        sprintf('NP(%.3f s) = %+.5f, i.e. %.3f%% of peak', ...
                t(1), NP(1), 100*endFrac), endFrac <= 0.01, src14, '-', ...
        ['S(0) = SF has a zero last row, so C(0) = 0 exactly.  A non-zero ' ...
         'first sample means the impulse is not on the miss node']);

    % 6  "approach 3 when the time to go before intercept is very large"
    tail = t >= 0.67*TF;
    npT  = NP(tail);
    falling = all(diff(npT) < 0);
    B = addBound(B, 'NP approaches 3 at large tgo', ...
        'falling over the last third of the sweep, and NP(TF) in [2.5, 6]', ...
        sprintf('NP(%.2f s) = %.4f, %s over the last third', ...
                t(end), NP(end), ternStr(falling, 'monotone falling', 'NOT monotone')), ...
        falling && NP(end) >= 2.5 && NP(end) <= 6, src14, '-', ...
        ['the band is wide ON PURPOSE: the book states a LIMIT, not a value ' ...
         'at 10 s.  Its own single-lag formula gives 3.691 there and the ' ...
         'cubic''s effective lag is 2*Z/W + TAU = 1.07 s, so about 3.75 is ' ...
         'what the rank-one reduction predicts.  This row tests "approaches", ' ...
         'nothing finer']);

    % 7  "BOTH ... get large near intercept and go to zero at intercept"
    [slMax, iSl] = max(NPsl);
    B = addShape(B, 'the single-lag N'' does the same (the "both" in the prose)', ...
        'interior max, and |N''| at the first sample <= 1% of that peak', ...
        sprintf('max %.2f at tgo = %.3f s; N''(%.3f s) = %.5f', ...
                slMax, t(iSl), t(1), NPsl(1)), ...
        iSl > 1 && iSl < numel(t) && abs(NPsl(1)) <= 0.01*slMax, ...
        'p.674 text (zarchan.txt:9766)', '-', ...
        ['the second curve of Fig. 26.14.  Its roll-off to zero is produced ' ...
         'by the author''s +0.0001 regulariser, which is why it survives here ' ...
         'and why the true gamma -> 0 law does not']);
else
    for nm = {'NP goes NEGATIVE near intercept (the RHP zero)', ...
              'NP gets LARGE near intercept, then decays', ...
              'NP returns to ZERO at intercept', ...
              'NP approaches 3 at large tgo', ...
              'the single-lag N'' does the same (the "both" in the prose)'}
        B = addSkip(B, nm{1}, 'Fig. 26.14 shape', whyCase, src14, '-');
    end
end

% ---- INFO.  Reported, never scored. ---------------------------------------
B = addInfo(B, 'NP range vs the listing''s own axis([0 10 -10 50])', ...
    '-10 .. 50', sprintf('%.2f .. %.2f', min(NP), max(NP)), ...
    'listing plot block (zarchan.txt:9785)', ...
    ['NOT SCORED: a plot range the author chose, not a measurement.  That it ' ...
     'brackets this curve is corroboration and nothing more']);

B = addInfo(B, 'C4, C5, C6 at tgo = TF', 'no published values', ...
    sprintf('%+.5g, %+.5g, %+.5g', R.C(end,4), R.C(end,5), R.C(end,6)), ...
    'listing saves ArrayC4/C5/C6 (zarchan.txt:9784-9785)', ...
    ['NOT SCORED: the listing records these three and the chapter then plots ' ...
     'only NP.  No figure and no sentence in Ch.26 carries a number for them']);

if all(isfinite(R.S(:)))
    sv = svd(R.S);
    svStr = sprintf('sv2/sv1 = %.3e over %d states', ...
                    sv(2)/max(sv(1), realmin), numel(sv));
else
    svStr = 'not finite';
end
B = addInfo(B, 'rank of the final Riccati matrix', ...
    'rank 1 (S = p*v*v'', derived here -- not in the book)', svStr, ...
    'structural, see the rank-one note in the header', ...
    ['NOT SCORED, but the first run has now established the number: sv2/sv1 ' ...
     '= 7.014e-5 at the nominal case and the default H = 1e-4.  Still not ' ...
     'scored, because that is one point -- the leakage is a property of the ' ...
     'scheme AND of H, and nobody has yet swept it here the way the ' ...
     'cross-check leg was swept']);
end


% =============================================================================
function B = emptyBench()
%EMPTYBENCH  Field names and ORDER must match the adders exactly, or the
%   B(i) = ... appends throw -- which is the behaviour we want: a struct array
%   whose fields quietly reordered would make [B.ok] mean something else.
%   Schema is the one benchSuite's adaptGeneric consumes, unchanged.
B = struct('what', {}, 'kind', {}, 'book', {}, 'book_str', {}, 'ours', {}, ...
           'ours_str', {}, 'pct', {}, 'band', {}, 'tol', {}, 'ok', {}, ...
           'skipped', {}, 'why', {}, 'source', {}, 'units', {}, 'detail', {});
end

function B = addVal(B, what, book, ours, band, tol, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'value';
B(i).book = book;  B(i).book_str = sprintf('%.4g', book);
B(i).ours = ours;  B(i).ours_str = sprintf('%.6g', ours);
B(i).band = band;  B(i).tol = tol;
B(i).units = units;  B(i).source = source;  B(i).detail = detail;
if ~isfinite(ours) || book == 0
    B(i).pct = NaN;  B(i).ok = false;  B(i).skipped = true;
    B(i).why = 'the computed value is not finite (or the book value is zero)';
    return
end
B(i).pct = 100*(ours/book - 1);
B(i).ok  = abs(ours/book - 1) <= tol;
B(i).skipped = false;  B(i).why = '';
end

function B = addShape(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'shape';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'shape';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
end

function B = addBound(B, what, bookStr, oursStr, ok, source, units, detail)
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'bound';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'bound';  B(i).tol = NaN;
B(i).ok   = logical(ok);  B(i).skipped = false;  B(i).why = '';
B(i).source = source;  B(i).units = units;  B(i).detail = detail;
end

function B = addInfo(B, what, bookStr, oursStr, source, why)
%ADDINFO  Reported, never scored.  skipped = true keeps it out of the pass
%   count in BOTH directions: an INFO row can neither pass nor fail.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'info';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = oursStr;
B(i).pct  = NaN;   B(i).band = 'info';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = '-';  B(i).detail = '';
end

function B = addSkip(B, what, bookStr, why, source, units)
%ADDSKIP  A check that COULD NOT RUN.  Never a pass: a check that did not
%   happen is a different thing from one that succeeded.
i = numel(B) + 1;
B(i).what = what;  B(i).kind = 'skip';
B(i).book = NaN;   B(i).book_str = bookStr;
B(i).ours = NaN;   B(i).ours_str = '--';
B(i).pct  = NaN;   B(i).band = 'skip';  B(i).tol = NaN;
B(i).ok   = false; B(i).skipped = true; B(i).why = why;
B(i).source = source;  B(i).units = units;  B(i).detail = '';
end


% =============================================================================
function s = render(R, bookCase, ev)
L = {};
p = R.p;
L{end+1} = 'Zarchan Listing 26.4 -- OPTIMAL CONTROL GAINS FOR A CUBIC FLIGHT CONTROL SYSTEM';
L{end+1} = '   (matrix Riccati sweep; T is TIME TO GO, and there is no length unit anywhere)';
L{end+1} = sprintf(['   TAU = %g s   WZ = %g r/s   W = %g r/s   Z = %g   ' ...
    'GAM = %g   TF = %g s   H = %g s   samp = %g s'], p.TAU, p.WZ, p.W, p.Z, ...
    p.GAM, p.TF, p.H, p.samp);
if ~bookCase
    L{end+1} = '   *** OFF the listing''s nominal case: the Fig. 26.14 rows are SKIPPED. ***';
end
L{end+1} = '';

L{end+1} = 'EFFECTIVE NAVIGATION RATIO AND GAINS vs TIME TO GO';
L{end+1} = sprintf('%9s %10s %10s %12s %12s %12s', ...
    'tgo [s]', 'NP', 'N''(1-lag)', 'C4', 'C5', 'C6');
show = unique([1, round(linspace(1, numel(R.t), 12)), numel(R.t)]);
for r = show
    L{end+1} = sprintf('%9.3f %10.4f %10.4f %12.5g %12.5g %12.5g', ...
        R.t(r), R.NP(r), R.NP_singlelag(r), R.C(r,4), R.C(r,5), R.C(r,6)); %#ok<AGROW>
end
[mn, imn] = min(R.NP);  [mx, imx] = max(R.NP);
L{end+1} = sprintf('   extremes: min NP = %+.3f at %.3f s, max NP = %+.3f at %.3f s', ...
    mn, R.t(imn), mx, R.t(imx));
if ~isempty(R.cross)
    L{end+1} = sprintf(['   cross-check (Listing 26.1 matrices, H = %g s): worst ' ...
        '|dC/C| = %.4f%% over tgo >= 1 s, at tgo = %.2f s'], R.cross.h, ...
        100*R.cross.maxrel, R.cross.t_worst);
    if isfinite(R.cross.ratio)
        L{end+1} = sprintf(['   H-scaling  (same comparison at 2h = %g s): %.4f%% ' ...
            ', ratio %.3f -- %s'], R.cross.h2, 100*R.cross.maxrel_2h, ...
            R.cross.ratio, ternStr(isempty(R.cross.scale_why), ...
            'O(H^2), so what is left is numerical', 'NOT SCORED, see the row'));
    else
        L{end+1} = sprintf('   H-scaling: NOT RUN -- %s', R.cross.scale_why);
    end
else
    L{end+1} = '   cross-check: NOT RUN (''crosscheck'', false)';
end
L{end+1} = '';

K = R.selfcheck;
L{end+1} = 'STRUCTURAL INVARIANTS  (self-consistency -- NOT book values)';
for i = 1:numel(K.names)
    L{end+1} = sprintf('   %-4s %-46s %s', vstr(K.oks(i)), K.names{i}, K.detail{i}); %#ok<AGROW>
end
L{end+1} = '';

if isempty(R.bench)
    L{end+1} = 'BENCHMARKS  NOT RUN (''benchmark'', false).  R.pass = false meaning NOT CHECKED.';
else
    L{end+1} = 'BENCHMARKS  (book value <- zarchan.txt; line numbers in .source)';
    for i = 1:numel(R.bench)
        b = R.bench(i);
        if b.skipped
            mark = '--';
        else
            mark = vstr(b.ok);
        end
        L{end+1} = sprintf('   %-4s [%-5s] %-52s book %-22s ours %s', ...
            mark, b.kind, b.what, b.book_str, b.ours_str); %#ok<AGROW>
        L{end+1} = sprintf('        %s', b.source); %#ok<AGROW>
        if b.skipped && ~isempty(b.why)
            L{end+1} = sprintf('        why: %s', b.why); %#ok<AGROW>
        end
    end
    L{end+1} = sprintf('   %d row(s) evaluated, %d passed.', sum(ev), ...
        sum(ev & [R.bench.ok]));
end
s = strjoin(L, newline);
end

function v = vstr(ok)
if ok, v = 'ok'; else, v = 'FAIL'; end
end

function s = ternStr(c, a, b)
if c, s = a; else, s = b; end
end
