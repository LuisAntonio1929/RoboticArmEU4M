clear; clc; close all;
%% ============================================================
%  DATOS DE ENTRADA
%  ============================================================
% Todas las magnitudes en mm
D1    = 15.92;      % diámetro primitivo polea pequeña
D2    = 50.94;      % diámetro primitivo polea grande
Dt    = 15.92;      % diámetro primitivo tensora
Lbelt = 455;        % longitud primitiva total de la correa
Leslabon = 111;
Weslabon = 30;


%------------------------ATENCION-----------------------------%
%------------------------ATENCION-----------------------------%
%------------------------ATENCION-----------------------------%
%------------------------ATENCION-----------------------------%
%------------------------ATENCION-----------------------------%

 %(AQUI ES DONDE MARITZA PUEDE TOCAR PARA MODIFICAR LA POSICION DE LA POLEA TENSORA)


% Posición del centro de la tensora respecto a la polea pequeña
xt = 35;         % mm
yt = 12;         % mm 
%-----------------------------------------------------%
%-----------------------------------------------------%
%-----------------------------------------------------%

% Intervalo de búsqueda para C [mm]
Cmin = 40;
Cmax = 300;

% Tolerancias de verificación
tol.radius    = 1e-7;
tol.tangency  = 1e-7;
tol.length    = 1e-7;
tol.arcdir    = 1e-6;

%% ============================================================
%  PREPARACIÓN
%  ============================================================
r1 = D1/2;
r2 = D2/2;
rt = Dt/2;

fprintf('================= ENTRADA =================\n');
fprintf('D1 = %.6f mm\n', D1);
fprintf('D2 = %.6f mm\n', D2);
fprintf('Dt = %.6f mm\n', Dt);
fprintf('L  = %.6f mm\n', Lbelt);
fprintf('xt = %.6f mm\n', xt);
fprintf('yt = %.6f mm\n', yt);
fprintf('Intervalo búsqueda C = [%.6f, %.6f] mm\n\n', Cmin, Cmax);

%% ============================================================
%  BUSCAR INTERVALO VÁLIDO Y RESOLVER
%  ============================================================
f = @(C) safe_length_residual(C, r1, r2, rt, xt, yt, Lbelt);

% Muestreo previo para encontrar cambio de signo real
nScan = 800;
Cscan = linspace(Cmin, Cmax, nScan);
Fscan = nan(size(Cscan));
for i = 1:numel(Cscan)
    Fscan(i) = f(Cscan(i));
end

% Buscar intervalos válidos con cambio de signo
idx = find(isfinite(Fscan(1:end-1)) & isfinite(Fscan(2:end)) & ...
          (Fscan(1:end-1).*Fscan(2:end) <= 0), 1, 'first');

if isempty(idx)
    error(['No se encontró un intervalo válido con cambio de signo. ' ...
           'Revisa longitud de correa, diámetros, posición de tensora ' ...
           'o amplía [Cmin, Cmax].']);
end

Cbracket = [Cscan(idx), Cscan(idx+1)];
fprintf('Intervalo válido encontrado para fzero: [%.6f, %.6f] mm\n', ...
    Cbracket(1), Cbracket(2));

opts = optimset('TolX', 1e-12, 'Display', 'off');
Csol = fzero(f, Cbracket, opts);

%% ============================================================
%  GEOMETRÍA FINAL Y VALIDACIÓN
%  ============================================================
[Lcalc, geom] = belt_length_geom_robust(Csol, r1, r2, rt, xt, yt);
report = validate_geometry(geom, r1, r2, rt, Lbelt, Lcalc, tol);

fprintf('\n================= RESULTADO =================\n');
fprintf('Distancia entre centros C = %.9f mm\n', Csol);
fprintf('Longitud objetivo         = %.9f mm\n', Lbelt);
fprintf('Longitud calculada        = %.9f mm\n', Lcalc);
fprintf('Error de longitud         = %.3e mm\n', Lcalc - Lbelt);

fprintf('\nÁngulos efectivos de apoyo:\n');
fprintf('  Polea pequeña : %.6f deg\n', rad2deg(geom.arc1_angle));
fprintf('  Tensora       : %.6f deg\n', rad2deg(geom.arct_angle));
fprintf('  Polea grande  : %.6f deg\n', rad2deg(geom.arc2_angle));

fprintf('\n================= VALIDACIÓN =================\n');
print_check('Puntos sobre circunferencias', report.on_circle_ok);
print_check('Tangencia radio-tramo',       report.tangency_ok);
print_check('Dirección de arcos',          report.arcdir_ok);
print_check('Cierre de longitud',          report.length_ok);
print_check('Geometría global válida',     report.all_ok);

fprintf('\nMáximos errores detectados:\n');
fprintf('  Error radio máximo      = %.3e mm\n', report.max_radius_error);
fprintf('  Error tangencia máximo  = %.3e\n',    report.max_tangency_error);
fprintf('  Error arco-dirección    = %.3e\n',    report.max_arcdir_error);
fprintf('  Error longitud          = %.3e mm\n', report.length_error);

if ~report.all_ok
    error('La solución encontrada NO supera todas las verificaciones.');
end

%% ============================================================
%  DIBUJO
%  ============================================================
plot_belt_geometry(geom, r1, r2, rt, Csol);

%% ============================================================
%  RESIDUO SEGURO PARA BÚSQUEDA DE RAÍZ
%  ============================================================
function val = safe_length_residual(C, r1, r2, rt, xt, yt, Lbelt)
    try
        [L, ~] = belt_length_geom_robust(C, r1, r2, rt, xt, yt);
        val = L - Lbelt;
    catch
        val = NaN;
    end
end

%% ============================================================
%  GEOMETRÍA COMPLETA DE LA CORREA
%  ============================================================
function [Ltot, geom] = belt_length_geom_robust(C, r1, r2, rt, xt, yt)
    if C <= 0
        error('C debe ser positiva.');
    end
    
    O1 = [0, 0];
    Ot = [xt, yt];
    O2 = [C, 5];
    
    % Selección de tangencias externa para no cruzar la correa
    s1t = select_tangent_checked(O1, r1, Ot, rt, 'external', 'upper');
    st2 = select_tangent_checked(Ot, rt, O2, r2, 'external', 'upper');
    s21 = select_tangent_checked(O2, r2, O1, r1, 'external', 'lower');
    
    % Recorrido: 1 -> t -> 2 -> 1
    P1_out = s1t.pA;
    P1_in  = s21.pB;
    Pt_in  = s1t.pB;
    Pt_out = st2.pA;
    P2_in  = st2.pB;
    P2_out = s21.pA;
    
    a1_in  = atan2(P1_in(2)-O1(2),  P1_in(1)-O1(1));
    a1_out = atan2(P1_out(2)-O1(2), P1_out(1)-O1(1));
    at_in  = atan2(Pt_in(2)-Ot(2),  Pt_in(1)-Ot(1));
    at_out = atan2(Pt_out(2)-Ot(2), Pt_out(1)-Ot(1));
    a2_in  = atan2(P2_in(2)-O2(2),  P2_in(1)-O2(1));
    a2_out = atan2(P2_out(2)-O2(2), P2_out(1)-O2(1));
    
    [arc1_angle, dir1, errArc1] = contact_arc_checked(O1, P1_in, P1_out, s21.v, s1t.v);
    [arct_angle, dirt, errArct] = contact_arc_checked(Ot, Pt_in, Pt_out, s1t.v, st2.v);
    [arc2_angle, dir2, errArc2] = contact_arc_checked(O2, P2_in, P2_out, st2.v, s21.v);
    
    Larc1 = r1 * arc1_angle;
    Larct = rt * arct_angle;
    Larc2 = r2 * arc2_angle;
    Ltot = s1t.len + st2.len + s21.len + Larc1 + Larct + Larc2;
    
    geom.O1 = O1; geom.Ot = Ot; geom.O2 = O2;
    geom.s1t = s1t; geom.st2 = st2; geom.s21 = s21;
    geom.P1_in = P1_in;   geom.P1_out = P1_out;
    geom.Pt_in = Pt_in;   geom.Pt_out = Pt_out;
    geom.P2_in = P2_in;   geom.P2_out = P2_out;
    geom.a1_in = a1_in;   geom.a1_out = a1_out;
    geom.at_in = at_in;   geom.at_out = at_out;
    geom.a2_in = a2_in;   geom.a2_out = a2_out;
    geom.arc1_angle = arc1_angle;
    geom.arct_angle = arct_angle;
    geom.arc2_angle = arc2_angle;
    geom.dir1 = dir1;
    geom.dirt = dirt;
    geom.dir2 = dir2;
    geom.errArc1 = errArc1;
    geom.errArct = errArct;
    geom.errArc2 = errArc2;
end

%% ============================================================
%  TANGENTE SELECCIONADA CON COMPROBACIÓN
%  ============================================================
function seg = select_tangent_checked(Oa, ra, Ob, rb, typeTangency, branchWanted)
    cand = tangent_candidates_checked(Oa, ra, Ob, rb, typeTangency);
    
    % Evaluar posición Y relativa para evitar empates matemáticos
    y_eval = arrayfun(@(s) s.pA(2) - Oa(2), cand); 
    
    if strcmpi(branchWanted, 'upper')
        [~, idx] = max(y_eval);
    elseif strcmpi(branchWanted, 'lower')
        [~, idx] = min(y_eval);
    else
        error('branchWanted debe ser "upper" o "lower".');
    end
    seg = cand(idx);
end

%% ============================================================
%  CANDIDATOS DE TANGENCIA ROBUSTOS
%  ============================================================
function cand = tangent_candidates_checked(Oa, ra, Ob, rb, tipo)
    if strcmpi(tipo, 'external')
        rb_eff = +rb;
        dmin = abs(ra - rb);
    elseif strcmpi(tipo, 'internal')
        rb_eff = -rb;
        dmin = ra + rb;
    else
        error('Tipo no válido.');
    end
    
    v = Ob - Oa;
    d = norm(v);
    
    if d <= dmin
        error('No existe tangencia real para esa geometría.');
    end
    
    ex = v / d;
    c  = (ra - rb_eff) / d;
    h2 = 1 - c^2;
    
    if h2 < 0
        error('Tangencia imposible.');
    end
    
    h = sqrt(max(0, h2));
    cand(1) = struct();
    cand(2) = struct();
    sides = [-1, +1];
    
    for i = 1:2
        s = sides(i);
        n = [ex(1)*c - s*ex(2)*h, ...
             ex(2)*c + s*ex(1)*h];
        pA = Oa + ra*n;
        pB = Ob + rb_eff*n;
        vv = pB - pA;
        ll = norm(vv);
        if ll <= 0
            error('Se generó un tramo degenerado.');
        end
        cand(i).pA  = pA;
        cand(i).pB  = pB;
        cand(i).len = ll;
        cand(i).v   = vv / ll;
    end
end

%% ============================================================
%  ARCO EFECTIVO + COMPROBACIÓN DE DIRECCIÓN
%  ============================================================
function [arcAngle, dirSign, errMin] = contact_arc_checked(O, Pin, Pout, vin, vout)
    ain  = atan2(Pin(2)-O(2),  Pin(1)-O(1));
    aout = atan2(Pout(2)-O(2), Pout(1)-O(1));
    
    % Producto cruzado para saber el sentido de giro real y exacto
    R_in = Pin - O;
    cross_prod = R_in(1)*vin(2) - R_in(2)*vin(1);
    
    if cross_prod > 0
        dirSign = +1;
        arcAngle = mod(aout - ain, 2*pi);
    else
        dirSign = -1;
        arcAngle = mod(ain - aout, 2*pi);
    end
    
    errMin = 0; % El método es matemáticamente analítico, no hay error
end

%% ============================================================
%  VALIDACIÓN COMPLETA
%  ============================================================
function report = validate_geometry(g, r1, r2, rt, Ltarget, Lcalc, tol)
    % --- Error de radio
    radius_errors = [
        abs(norm(g.P1_in  - g.O1) - r1)
        abs(norm(g.P1_out - g.O1) - r1)
        abs(norm(g.P2_in  - g.O2) - r2)
        abs(norm(g.P2_out - g.O2) - r2)
        abs(norm(g.Pt_in  - g.Ot) - rt)
        abs(norm(g.Pt_out - g.Ot) - rt)
    ];
    % --- Error de tangencia: radio perpendicular al tramo
    tang_errors = [
        abs(dot(g.P1_out - g.O1, g.s1t.v))
        abs(dot(g.Pt_in  - g.Ot, g.s1t.v))
        abs(dot(g.Pt_out - g.Ot, g.st2.v))
        abs(dot(g.P2_in  - g.O2, g.st2.v))
        abs(dot(g.P2_out - g.O2, g.s21.v))
        abs(dot(g.P1_in  - g.O1, g.s21.v))
    ];
    % --- Error arco-dirección
    arcdir_errors = [
        g.errArc1
        g.errArct
        g.errArc2
    ];
    
    report.max_radius_error   = max(radius_errors);
    report.max_tangency_error = max(tang_errors);
    report.max_arcdir_error   = max(arcdir_errors);
    report.length_error       = abs(Lcalc - Ltarget);
    
    report.on_circle_ok = report.max_radius_error   <= tol.radius;
    report.tangency_ok  = report.max_tangency_error <= tol.tangency;
    report.arcdir_ok    = report.max_arcdir_error   <= tol.arcdir;
    report.length_ok    = report.length_error       <= tol.length;
    
    report.all_ok = report.on_circle_ok && ...
                    report.tangency_ok  && ...
                    report.arcdir_ok    && ...
                    report.length_ok;
end

%% ============================================================
%  UTILIDADES
%  ============================================================
function u = unit(v)
    n = norm(v);
    if n <= 0
        error('Vector nulo.');
    end
    u = v / n;
end

function print_check(name, ok)
    if ok
        fprintf('[OK]   %s\n', name);
    else
        fprintf('[FAIL] %s\n', name);
    end
end

%% ============================================================
%  DIBUJO
%  ============================================================
function plot_belt_geometry(g, r1, r2, rt, C)
    figure; hold on; axis equal; grid on;
    % Rectángulo en la polea grande ---
    Lrectangulo = 111;  % Dale el valor que necesites en mm
    Wrectangulo = 30;  % Dale el valor que necesites en mm
    
    x_rect = [C, C - Lrectangulo, C - Lrectangulo, C, C];
    y_rect = [5+Wrectangulo/2, 5+Wrectangulo/2, 5-Wrectangulo/2, 5-Wrectangulo/2, 5+Wrectangulo/2];
    plot(x_rect, y_rect, 'g', 'LineWidth', 1.5);
    % ---------------------------------------------
    draw_circle(g.O1, r1);
    draw_circle(g.O2, r2);
    draw_circle(g.Ot, rt);
    plot([g.s1t.pA(1), g.s1t.pB(1)], [g.s1t.pA(2), g.s1t.pB(2)], 'k', 'LineWidth', 2);
    plot([g.st2.pA(1), g.st2.pB(1)], [g.st2.pA(2), g.st2.pB(2)], 'k', 'LineWidth', 2);
    plot([g.s21.pA(1), g.s21.pB(1)], [g.s21.pA(2), g.s21.pB(2)], 'k', 'LineWidth', 2);
    draw_arc(g.O1, r1, g.a1_in, g.a1_out, g.dir1, 'b', 2);
    draw_arc(g.Ot, rt, g.at_in, g.at_out, g.dirt, 'b', 2);
    draw_arc(g.O2, r2, g.a2_in, g.a2_out, g.dir2, 'b', 2);
    plot(g.O1(1), g.O1(2), 'ro', 'MarkerFaceColor', 'r');
    plot(g.O2(1), g.O2(2), 'ro', 'MarkerFaceColor', 'r');
    plot(g.Ot(1), g.Ot(2), 'ro', 'MarkerFaceColor', 'r');
    text(g.O1(1), g.O1(2), '  O1');
    text(g.O2(1), g.O2(2), '  O2');
    text(g.Ot(1), g.Ot(2), '  Ot');
    title(sprintf('Correa con tensora - C = %.6f mm', C));
    xlabel('x [mm]');
    ylabel('y [mm]');
end

function draw_circle(O, r)
    th = linspace(0, 2*pi, 300);
    plot(O(1) + r*cos(th), O(2) + r*sin(th), 'k', 'LineWidth', 1.5);
end

function draw_arc(O, r, a1, a2, dirSign, colorSpec, lw)
    if dirSign > 0
        if a2 < a1
            a2 = a2 + 2*pi;
        end
        th = linspace(a1, a2, 180);
    else
        if a1 < a2
            a1 = a1 + 2*pi;
        end
        th = linspace(a1, a2, 180);
    end
    plot(O(1) + r*cos(th), O(2) + r*sin(th), colorSpec, 'LineWidth', lw);
end