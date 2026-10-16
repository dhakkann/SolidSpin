"""
Optimization and Controllability Solvers for Asymmetric Weapon Balancer.
Ported from Fusion 360 Asymmetric Weapon Balancer (MIT Licence - Soso Chkhortolia).
Pure Python numerical algorithms:
- 1D optimization: Scan + Illinois root finder
- Multi-D optimization: Minimum-norm Gauss-Newton with Broyden rank-one updates
- SVD / Controllability analysis of the sensitivity Jacobian
"""

import math

# Default numerical thresholds
COM_TOL_MM = 1.0e-4           # Target balance tolerance in mm
SCAN_SAMPLES = 9              # Samples for 1D bracketing
R_TOL_FRACTION = 1.0e-7       # Relative tolerance for 1D root finder
MAX_EVALS = 40                # Max iterations for 1D
FD_STEP = 2.0e-3              # Finite-difference perturbation fraction
MAX_EVALS_MULTI = 160         # Max rebuild limit for multi-dimension solve


class SolveError(Exception):
    def __init__(self, message, report=""):
        super().__init__(message)
        self.report = report


class Cancelled(Exception):
    pass


class RebuildFailed(Exception):
    pass


class _OutOfRebuilds(Exception):
    pass


def _solve_linear(a, b):
    """Solve the small square linear system a * x = b via Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-300:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def _min_norm_step(jac, res, cols, mu):
    """Smallest step dz over the active columns in cols satisfying jac * dz = -res (damped)."""
    m = len(res)
    a = [[sum(jac[p][i] * jac[q][i] for i in cols) for q in range(m)] for p in range(m)]
    lam = mu * max(sum(a[i][i] for i in range(m)), 1e-30)
    for i in range(m):
        a[i][i] += lam
    y = _solve_linear(a, list(res))
    step = {}
    if y is None:
        return step
    for i in cols:
        step[i] = -sum(jac[p][i] * y[p] for p in range(m))
    return step


def probe_sensitivity(evaluate, x0, scale, lo, hi, r0):
    """Compute finite-difference sensitivity of the in-plane offset to each dimension.

    Returns (jac, dead):
      jac[p][i] = d offset_p / d z_i with z_i = (x_i - x0_i) / scale_i
      dead lists indices of dimensions whose perturbation fails to rebuild.
    """
    n, m = len(x0), len(r0)
    jac = [[0.0] * n for _ in range(m)]
    dead = []
    for i in range(n):
        col = None
        for h in (FD_STEP, -FD_STEP):
            x = list(x0)
            x[i] = x0[i] + h * scale[i]
            if not lo[i] <= x[i] <= hi[i]:
                continue
            try:
                rr = evaluate(x)
            except Cancelled:
                raise
            except Exception:
                continue
            col = [(rr[p] - r0[p]) / h for p in range(m)]
            break
        if col is None:
            dead.append(i)
        else:
            for p in range(m):
                jac[p][i] = col[p]
    return jac, dead


def _eig2(a):
    """Eigenvalues (large, small) and unit eigenvector of the larger one for 2x2 symmetric matrix."""
    tr = a[0][0] + a[1][1]
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    disc = math.sqrt(max(tr * tr / 4.0 - det, 0.0))
    l1, l2 = tr / 2.0 + disc, max(tr / 2.0 - disc, 0.0)
    if abs(a[0][1]) > 1e-300:
        v = (l1 - a[1][1], a[0][1])
    else:
        v = (1.0, 0.0) if a[0][0] >= a[1][1] else (0.0, 1.0)
    length = math.hypot(*v)
    return l1, l2, (v[0] / length, v[1] / length)


def controllability(jac, r0, pct, dead=()):
    """Determine whether the candidate dimensions can balance both in-plane directions.

    jac: 2 x n sensitivities per 100% change.
    r0: current offset in mm.
    pct: allowed change per dimension (%).
    """
    n = len(jac[0])
    live = [i for i in range(n) if i not in dead]
    per_pct = [(jac[0][i] / 100.0, jac[1][i] / 100.0) for i in range(n)]
    a = [[sum(per_pct[i][p] * per_pct[i][q] for i in live) for q in range(2)] for p in range(2)]
    l1, l2, u = _eig2(a)
    s1, s2 = math.sqrt(l1), math.sqrt(l2)
    ratio = s2 / s1 if s1 > 0.0 else 0.0
    out = {
        'per_pct': per_pct,
        'ratio': ratio,
        'line': u,
        'across': 0.0,
        'estimate': [0.0] * n,
        'reachable': False
    }

    if s1 * pct < 10 * COM_TOL_MM:
        out['verdict'] = 'none'
        return out

    if ratio >= 0.02:
        out['verdict'] = 'both' if ratio >= 0.1 else 'weak'
        det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
        y = (
            (a[1][1] * -r0[0] - a[0][1] * -r0[1]) / det,
            (-a[1][0] * -r0[0] + a[0][0] * -r0[1]) / det
        )
        for i in live:
            out['estimate'][i] = per_pct[i][0] * y[0] + per_pct[i][1] * y[1]
    else:
        out['verdict'] = 'one line'
        w = (-u[1], u[0])
        out['across'] = abs(r0[0] * w[0] + r0[1] * w[1])
        g = [per_pct[i][0] * u[0] + per_pct[i][1] * u[1] for i in range(n)]
        gg = sum(g[i] ** 2 for i in live)
        along = r0[0] * u[0] + r0[1] * u[1]
        for i in live:
            out['estimate'][i] = -along * g[i] / gg if gg > 0 else 0.0

    out['reachable'] = max((abs(v) for v in out['estimate']), default=0.0) <= pct
    return out


def rate_dimensions(per_pct, r0, pct, dead=()):
    """Rate individual capability of each dimension.

    Returns dict per dimension:
      kind: 'ok' | 'none' | 'dead'
      effect: mm of COM shift per 1% change
      share: fraction of offset that can be cancelled alone (0..1)
      needed: % change that achieves that cancellation
    """
    r = math.hypot(*r0)
    out = []
    for i, (dx, dy) in enumerate(per_pct):
        e = math.hypot(dx, dy)
        if i in dead:
            out.append({'kind': 'dead', 'effect': 0.0, 'share': 0.0, 'needed': 0.0})
        elif e * pct < 10 * COM_TOL_MM:
            out.append({'kind': 'none', 'effect': e, 'share': 0.0, 'needed': 0.0})
        elif r <= COM_TOL_MM:
            out.append({'kind': 'ok', 'effect': e, 'share': 1.0, 'needed': 0.0})
        else:
            dot = r0[0] * dx + r0[1] * dy
            out.append({
                'kind': 'ok',
                'effect': e,
                'share': abs(dot) / (e * r),
                'needed': -dot / (e * e) if e > 0 else 0.0
            })
    return out


def solve(evaluate, lo, hi, x_ref, fmt=str, tick=None):
    """1D optimization: Find single dimension value in [lo, hi] closest to the spin axis."""
    cache = {}

    def ev(x):
        key = round(x, 12)
        if key not in cache:
            if tick:
                tick(len(cache))
            cache[key] = evaluate(x)
        return cache[key]

    # 1. Scan range
    samples = []
    for i in range(SCAN_SAMPLES):
        x = lo + (hi - lo) * i / (SCAN_SAMPLES - 1)
        try:
            samples.append((x, ev(x)))
        except Cancelled:
            raise
        except Exception:
            samples.append((x, None))
    valid = [(x, ab) for x, ab in samples if ab is not None]

    def scan_report(project=None):
        head = 'COM offset along its travel' if project else 'COM distance from axis'
        lines = [f'Scan ({head}, mm):']
        for x, ab in samples:
            if ab is None:
                val = 'rebuild failed'
            elif project:
                val = f'{project(ab):+.6f}'
            else:
                val = f'{math.hypot(*ab):.6f}'
            lines.append(f'  {fmt(x):>14}  ->  {val}')
        return '\n'.join(lines)

    if len(valid) < 2:
        raise SolveError('The model failed to rebuild across the scanned range.', scan_report())

    # 2. Movement direction
    n = len(valid)
    ma = sum(ab[0] for _, ab in valid) / n
    mb = sum(ab[1] for _, ab in valid) / n
    sxx = sum((ab[0] - ma) ** 2 for _, ab in valid)
    syy = sum((ab[1] - mb) ** 2 for _, ab in valid)
    sxy = sum((ab[0] - ma) * (ab[1] - mb) for _, ab in valid)
    travel = max(math.hypot(p[0] - q[0], p[1] - q[1]) for _, p in valid for _, q in valid)
    if travel < 10 * COM_TOL_MM:
        raise SolveError(
            f'Changing this parameter barely moves the centre of mass ({travel:.2e} mm). '
            'Pick a parameter that shapes the mass distribution.',
            scan_report()
        )

    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    u = (math.cos(theta), math.sin(theta))
    first, last = valid[0][1], valid[-1][1]
    if (last[0] - first[0]) * u[0] + (last[1] - first[1]) * u[1] < 0.0:
        u = (-u[0], -u[1])

    def along(ab):
        return ab[0] * u[0] + ab[1] * u[1]

    def across(ab):
        return -ab[0] * u[1] + ab[1] * u[0]

    report = scan_report(along)

    # 3. Bracket zero crossing
    pts = [(x, along(ab)) for x, ab in valid]
    a = fa = b = fb = None
    for (x1, f1), (x2, f2) in zip(pts, pts[1:]):
        if f1 == 0.0 or f1 * f2 < 0.0:
            a, fa, b, fb = x1, f1, x2, f2
            break
    if a is None:
        best = min(pts, key=lambda p: abs(p[1]))
        raise SolveError(
            f'The centre of mass never crosses the axis in this range (closest: {abs(best[1]):.4f} mm at {fmt(best[0])}).\n'
            'Widen the search range or choose a different dimension.',
            report
        )

    # 4. Illinois root finder
    r_tol = max(abs(x_ref), 1e-6) * R_TOL_FRACTION
    side = 0
    root = a if fa == 0.0 else None
    while root is None and len(cache) < MAX_EVALS:
        c = 0.5 * (a + b) if fb == fa else b - fb * (b - a) / (fb - fa)
        span = abs(b - a)
        if not (min(a, b) + 0.01 * span <= c <= max(a, b) - 0.01 * span):
            c = 0.5 * (a + b)
        fc = along(ev(c))
        if abs(fc) <= COM_TOL_MM or span <= r_tol:
            root = c
            break
        if fa * fc < 0.0:
            b, fb = c, fc
            if side == -1:
                fa *= 0.5
            side = -1
        else:
            a, fa = c, fc
            if side == 1:
                fb *= 0.5
            side = 1
    if root is None:
        root = 0.5 * (a + b)

    return {
        'root': root,
        'along': along,
        'across': across,
        'evals': len(cache),
        'report': report
    }


def solve_multi(evaluate, x0, scale, lo, hi, tick=None, initial=None):
    """Multi-D optimization: Minimum-change Gauss-Newton with Broyden rank-one updates."""
    n = len(x0)
    count = [0]
    zlo = [(lo[i] - x0[i]) / scale[i] for i in range(n)]
    zhi = [(hi[i] - x0[i]) / scale[i] for i in range(n)]

    def ev(z):
        if count[0] >= MAX_EVALS_MULTI:
            raise _OutOfRebuilds()
        if tick:
            tick(count[0])
        count[0] += 1
        return tuple(evaluate([x0[i] + z[i] * scale[i] for i in range(n)]))

    def norm(r):
        return math.sqrt(sum(v * v for v in r))

    z = [0.0] * n
    if initial:
        r, jac0, dead0 = initial
        r = tuple(r)
    else:
        try:
            r = ev(z)
        except (Cancelled, _OutOfRebuilds):
            raise
        except Exception as e:
            raise SolveError(f"Model failed to rebuild at baseline values: {e}")
        jac0, dead0 = None, []

    r_start = r
    m = len(r)
    dead, no_effect = list(dead0), []
    active = [i for i in range(n) if i not in dead]

    def jacobian(z, r):
        jac = [[0.0] * n for _ in range(m)]
        for i in list(active):
            col = None
            for h in (FD_STEP, -FD_STEP):
                if not zlo[i] <= z[i] + h <= zhi[i]:
                    continue
                zz = list(z)
                zz[i] += h
                try:
                    rr = ev(zz)
                except (Cancelled, _OutOfRebuilds):
                    raise
                except Exception:
                    continue
                col = [(rr[p] - r[p]) / h for p in range(m)]
                break
            if col is None:
                active.remove(i)
                dead.append(i)
            else:
                for p in range(m):
                    jac[p][i] = col[p]
        return jac

    stop_reason = ""
    try:
        jac = [row[:] for row in jac0] if jac0 else jacobian(z, r)
        for i in list(active):
            reach = math.sqrt(sum(jac[p][i] ** 2 for p in range(m))) * (zhi[i] - zlo[i])
            if reach < 10 * COM_TOL_MM:
                active.remove(i)
                no_effect.append(i)
        if not active:
            raise SolveError("None of the selected dimensions moves the centre of mass.")

        mu, fresh = 1e-10, True
        while norm(r) > COM_TOL_MM:
            cols, dz, target = list(active), [0.0] * n, list(r)
            while cols:
                s = _min_norm_step(jac, target, cols, mu)
                if not s:
                    break
                over = [i for i in cols if not zlo[i] <= z[i] + s[i] <= zhi[i]]
                if not over:
                    for i in cols:
                        dz[i] = s[i]
                    break
                for i in over:
                    dz[i] = min(max(z[i] + s[i], zlo[i]), zhi[i]) - z[i]
                    cols.remove(i)
                    for p in range(m):
                        target[p] += jac[p][i] * dz[i]

            if max(abs(v) for v in dz) < 1e-12:
                if fresh:
                    stop_reason = "No further improvement is possible within parameter limits."
                    break
                jac, fresh = jacobian(z, r), True
                continue

            accepted, t = False, 1.0
            for _ in range(6):
                zt = [min(max(z[i] + t * dz[i], zlo[i]), zhi[i]) for i in range(n)]
                try:
                    rt = ev(zt)
                    accepted = norm(rt) < norm(r)
                except (Cancelled, _OutOfRebuilds):
                    raise
                except Exception:
                    accepted = False
                if accepted:
                    break
                t *= 0.5

            if accepted:
                step = [zt[i] - z[i] for i in range(n)]
                ss = sum(v * v for v in step)
                if ss > 0.0:  # Broyden rank-one update
                    pred = [r[p] + sum(jac[p][i] * step[i] for i in range(n)) for p in range(m)]
                    for p in range(m):
                        k = (rt[p] - pred[p]) / ss
                        for i in active:
                            jac[p][i] += k * step[i]
                z, r = zt, rt
                mu, fresh = max(mu * 0.25, 1e-12), False
            elif fresh:
                stop_reason = "No step improved the balance; solution out of reach or rebuild failed nearby."
                break
            else:
                jac, fresh = jacobian(z, r), True
                mu *= 10.0
    except _OutOfRebuilds:
        stop_reason = f"Reached limit of {MAX_EVALS_MULTI} rebuilds."

    at_limit = [
        i for i in active
        if abs(z[i] - zlo[i]) < 1e-9 or abs(z[i] - zhi[i]) < 1e-9
    ]

    one_line = False
    if m == 2 and active and 'jac' in locals():
        a = [[sum(jac[p][i] * jac[q][i] for i in active) for q in range(2)] for p in range(2)]
        tr = a[0][0] + a[1][1]
        det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
        one_line = tr > 0.0 and det / (0.25 * tr * tr) < 1e-4

    return {
        'x': [x0[i] + z[i] * scale[i] for i in range(n)],
        'residual': r,
        'start': r_start,
        'evals': count[0],
        'no_effect': no_effect,
        'dead': dead,
        'at_limit': at_limit,
        'stop_reason': stop_reason,
        'one_line': one_line
    }


def control_blocked(c):
    return c['verdict'] == 'none' or (c['verdict'] == 'one line' and c['across'] > COM_TOL_MM)


def control_lines(c, names, axis_names, pct, show_effects=True):
    nx, ny = axis_names
    dead = c.get('dead', ())
    live = [i for i in range(len(names)) if i not in dead]
    lines = []
    if show_effects:
        lines.append('Effect of +1% on COM:')
        for i in live:
            dx, dy = c['per_pct'][i]
            lines.append(f'  {names[i]}: {nx} {dx:+.4f}, {ny} {dy:+.4f} mm')
    if dead:
        dead_names = [names[i] for i in dead]
        lines.append(f'Breaks model when changed: {", ".join(dead_names)}')

    v = c['verdict']
    if v == 'none':
        lines.append(f'NOT CONTROLLABLE: No dimension moves the COM measurably within ±{pct}%.')
        return lines
    if v == 'one line':
        ux, uy = c['line']
        lines.append(f'ONE DIRECTION ONLY: Dimensions move COM along line ({nx} {ux:+.2f}, {ny} {uy:+.2f}).')
        if c['across'] > COM_TOL_MM:
            lines.append(
                f'{c["across"]:.4f} mm of offset is perpendicular to this line and cannot be removed. '
                'Add a dimension that moves the COM sideways.'
            )
            return lines
        lines.append('Perpendicular offset is zero; balance is possible.')
    elif v == 'weak':
        lines.append(f'CONTROLLABLE (WEAK): Dimensions move COM in nearly parallel directions.')
    else:
        lines.append(f'CONTROLLABLE IN BOTH {nx} AND {ny}.')

    est_strs = [
        f'{names[i]} {c["estimate"][i]:+.1f}%'
        for i in live if abs(c['estimate'][i]) >= 0.05
    ]
    est = ', '.join(est_strs) or 'none'
    worst = max((abs(c['estimate'][i]) for i in live), default=0.0)
    lines.append(f'Estimated change to balance: {est}.')
    if worst <= 0.7 * pct:
        lines.append(f'Within the ±{pct}% limit.')
    elif worst <= pct:
        lines.append(f'Close to the ±{pct}% limit.')
    else:
        lines.append(f'Beyond the ±{pct}% limit: Raise Max Change or select more effective dimensions.')
    return lines
